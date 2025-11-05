/********************************** (C) COPYRIGHT
 * ******************************* File Name          : main.c Author   : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *USART Print debugging routine:
 *USART1_Tx(PA9).
 *This example demonstrates using USART1(PA9) as a print debug port output.
 *
 */

#include "board.h"
#include "debug.h"
#include "device_hw.h"
#include "device_logic.h" // ★ 新增：设备逻辑层
#include "proto.h"
#include "timer.h"
#include "report_manager.h"
#include "usart2_dma.h"
#include <string.h>

/* Global typedef */

/* Global define */
/* ====== 版本号（随意改） ====== */
#define APP_VERSION "v1.0.0"

/* Global Variable */

static proto_ctx_t g_proto;     // 协议解析上下文
static uint8_t g_rx_chunk[256]; // 从 usart2_pull_chunk 拉出来的块

device_config_t g_dev_cfg;
device_runtime_t g_dev_rt;

// 本机 SensorID（规范里的 SensorID），这里写死一份示例值，后面按实际烧录ID改
static uint8_t g_sensor_id[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};

static void send_control_status_ack(uint8_t status) {
  uint8_t frame[16];
  size_t frame_len =
      proto_build_control_status_ack(g_sensor_id, status, frame, sizeof(frame));
  if (!frame_len)
    return;
  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-ACK-CS] tx status=0x%02X\r\n", status);
}

static void send_control_id_ack(uint8_t req_set, const uint8_t id_payload[6]) {
  uint8_t frame[24];
  size_t frame_len = proto_build_control_id_ack(g_sensor_id, CTRL_ID_QS, req_set,
                                                id_payload, frame,
                                                sizeof(frame));
  if (!frame_len)
    return;
  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-ACK-ID-TX] rs=%u id=%02X%02X%02X%02X%02X%02X\r\n", req_set,
         id_payload[0], id_payload[1], id_payload[2], id_payload[3],
         id_payload[4], id_payload[5]);
}

static void write_le16(uint8_t out[2], uint16_t value) {
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void send_param_query_ack(uint8_t req_set, const proto_frame_t *req) {
  uint8_t upload_payload[2];
  write_le16(upload_payload, g_dev_cfg.dt_upload_period);

  dev_out_mode_t mode[DEV_CH_COUNT];
  device_get_output_mode(&g_dev_cfg, mode);
  uint8_t output_payload[DEV_CH_COUNT];
  for (uint8_t i = 0; i < DEV_CH_COUNT; ++i)
    output_payload[i] = (uint8_t)mode[i];

  uint8_t work_payload[2];
  write_le16(work_payload, device_get_work_time(&g_dev_cfg));

  uint8_t stop_payload[2];
  write_le16(stop_payload, device_get_stop_time(&g_dev_cfg));

  uint8_t freq_payload[2];
  write_le16(freq_payload, device_get_us_freq(&g_dev_cfg));

  proto_param_tx_t catalog[] = {
      {.dtype = DT_UPLOAD_PERIOD,
       .lflag = 0x01,
       .len = sizeof(upload_payload),
       .data = upload_payload},
      {.dtype = DT_OUTPUT_CTRL,
       .lflag = 0x01,
       .len = sizeof(output_payload),
       .data = output_payload},
      {.dtype = DT_T_ON_SET,
       .lflag = 0x01,
       .len = sizeof(work_payload),
       .data = work_payload},
      {.dtype = DT_T_OFF_SET,
       .lflag = 0x01,
       .len = sizeof(stop_payload),
       .data = stop_payload},
      {.dtype = DT_FREQ_SET,
       .lflag = 0x01,
       .len = sizeof(freq_payload),
       .data = freq_payload},
  };

  proto_param_tx_t selected[sizeof(catalog) / sizeof(catalog[0])];
  uint8_t selected_cnt = 0;

  bool query_all = true;
  if (req) {
    if ((req->hdr.param_count != 0x0Fu) && req->param_cnt > 0)
      query_all = false;
  }

  const uint8_t catalog_count =
      (uint8_t)(sizeof(catalog) / sizeof(catalog[0]));

  if (query_all) {
    for (uint8_t i = 0; i < catalog_count; ++i)
      selected[selected_cnt++] = catalog[i];
  } else {
    for (uint8_t i = 0; i < req->param_cnt; ++i) {
      uint16_t dtype = req->params[i].dtype;
      for (uint8_t j = 0; j < catalog_count; ++j) {
        if (catalog[j].dtype == dtype) {
          bool already_added = false;
          for (uint8_t k = 0; k < selected_cnt; ++k) {
            if (selected[k].dtype == dtype) {
              already_added = true;
              break;
            }
          }
          if (!already_added && selected_cnt < catalog_count)
            selected[selected_cnt++] = catalog[j];
          break;
        }
      }
    }
  }

  if (selected_cnt == 0) {
    printf("[CTRL-PARAM] no known params requested\r\n");
    send_control_status_ack(0x00);
    return;
  }

  uint8_t frame[160];
  size_t frame_len =
      proto_build_control_param_ack(g_sensor_id, CTRL_PARAM_QS, req_set,
                                    selected,
                                    selected_cnt, frame, sizeof(frame));
  if (!frame_len) {
    printf("[CTRL-PARAM] build ack failed\r\n");
    return;
  }

  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-ACK-PARAM] cnt=%u rs=%u\r\n", selected_cnt, req_set);
}

static void apply_sensor_id(const uint8_t new_id[6]) {
  if (!new_id)
    return;

  if (memcmp(g_sensor_id, new_id, sizeof(g_sensor_id)) == 0)
    return;

  memcpy(g_sensor_id, new_id, sizeof(g_sensor_id));
  proto_set_local_id(&g_proto, g_sensor_id);
  report_manager_set_sensor_id(g_sensor_id);
  printf("[SENSOR-ID] updated to %02X%02X%02X%02X%02X%02X\r\n", g_sensor_id[0],
         g_sensor_id[1], g_sensor_id[2], g_sensor_id[3], g_sensor_id[4],
         g_sensor_id[5]);
}

/* ====== LED：默认 PC13（可按板子改） ====== */
#define LED_GPIO GPIOB
#define LED_PIN GPIO_Pin_15

__attribute__((section(".app_sig"), used, aligned(4)))
const uint32_t APP_SIGNATURE = 0x41505021u; // "APP!"

static void led_init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef g = {0};
  g.GPIO_Pin = LED_PIN;
  g.GPIO_Speed = GPIO_Speed_50MHz;
  g.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_GPIO, &g);
  GPIO_SetBits(LED_GPIO, LED_PIN); // 默认熄灭（看你板载LED极性）
}

// —— 回调打印：你可以在这里对接业务逻辑 ——
static void on_monitor(proto_ctx_t *ctx, const proto_frame_t *f) {
  (void)ctx;
  char msg[96];
  int m = snprintf(msg, sizeof(msg), "[MON] params=%u raw=%uB\r\n",
                   f->param_cnt, f->raw_len);
  if (m > 0)
    printf("%s", msg);

  // 这里可以按 f->params[i].dtype / len / val[] 去解析具体业务参数
}
static void on_monitor_ack(proto_ctx_t *ctx, const proto_frame_t *f) {
  (void)ctx;
  if (!f->has_cmd_status)
    return;
  report_manager_on_monitor_ack(f);
  char msg[64];
  int m =
      snprintf(msg, sizeof(msg), "[MON-ACK] status=0x%02X\r\n", f->cmd_status);
  if (m > 0)
    printf("%s", msg);
}
// 100 控制请求（如果本机是“终端+从机”，一般用不到，从机更多处理 100→101）
static void on_control(proto_ctx_t *ctx, const proto_frame_t *f) {
  (void)ctx;
  char msg[96];
  int m = snprintf(msg, sizeof(msg),
                   "[CTRL] type=0x%02X rs=%u param_cnt=%u has_id=%u\r\n",
                   f->ctrl_type, f->req_set, f->param_cnt, f->has_id_payload);
  if (m > 0)
    printf("%s", msg);

  if (f->ctrl_type == CTRL_REQ_MONITOR) {
    uint8_t status = (f->req_set == 0) ? 0xFF : 0x00;
    send_control_status_ack(status);
    if (status == 0xFF) {
      report_manager_request_immediate();
    }
  } else if (f->ctrl_type == CTRL_ID_QS) {
    if (f->req_set == 0) {
      send_control_id_ack(f->req_set, g_sensor_id);
    } else {
      if (!f->has_id_payload) {
        printf("[CTRL-ID] missing payload for set request\r\n");
        send_control_status_ack(0x00);
        return;
      }
      apply_sensor_id(f->id_payload);
      send_control_id_ack(f->req_set, g_sensor_id);
    }
  } else if (f->ctrl_type == CTRL_PARAM_QS) {
    if (f->req_set == 0) {
      send_param_query_ack(f->req_set, f);
    } else {
      printf("[CTRL-PARAM] set not supported\r\n");
      send_control_status_ack(0x00);
    }
  }
}
// 101 控制响应
static void on_control_ack(proto_ctx_t *ctx, const proto_frame_t *f) {
  (void)ctx;
  char msg[96];

  if (f->has_id_payload) {
    // ID 查询/设置的响应：CtrlHead + SensorID
    int m = snprintf(
        msg, sizeof(msg),
        "[CTRL-ACK-ID] type=0x%02X rs=%u ID=%02X%02X%02X%02X%02X%02X\r\n",
        f->ctrl_type, f->req_set, f->id_payload[0], f->id_payload[1],
        f->id_payload[2], f->id_payload[3], f->id_payload[4], f->id_payload[5]);
    if (m > 0)
      printf("%s", msg);
  } else if (f->has_cmd_status && f->param_cnt == 0) {
    // 仅状态的响应（例如“请求监测数据”的应答）
    int m = snprintf(msg, sizeof(msg), "[CTRL-ACK-CS] status=0x%02X\r\n",
                     f->cmd_status);
    if (m > 0)
      printf("%s", msg);
  } else {
    // 参数 / 输出 查询/设置 的响应（ParameterList）
    int m = snprintf(msg, sizeof(msg), "[CTRL-ACK] param_cnt=%u raw_len=%u\r\n",
                     f->param_cnt, f->raw_len);
    if (m > 0)
      printf("%s", msg);
  }

  // 这里同样可以按 f->params[i].dtype / len / val[] 做业务处理
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemCoreClockUpdate();
  Delay_Init();
  USART_Printf_Init(115200);
  __enable_irq();
  // printf("SystemClk:%d\r\n", SystemCoreClock);
  // printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
  usart2_dma_idle_init();
  // printf("\r\n[BOOT] USART2 DMA+IDLE ready. Send data to see chunks...\r\n");

  led_init();
  // 协议解析上下文初始化
  proto_init(&g_proto);
  // ★ 设置本机 SensorID（规范中的 SensorID）
  //   后续 proto_feed() 解析的每一帧，会先比对帧头里的 SensorID 与这里的值：
  //   如果不一致，则 f->dropped_by_id=1，且不会进入任何回调，相当于“废弃帧”。
  proto_set_local_id(&g_proto, g_sensor_id);
  report_manager_init(g_sensor_id);

  g_proto.on_monitor = on_monitor;
  g_proto.on_monitor_ack = on_monitor_ack;
  g_proto.on_control = on_control;
  g_proto.on_control_ack = on_control_ack;

  device_logic_init(&g_dev_cfg, &g_dev_rt); // 配置初始化
  device_hw_init();                         // 硬件初始化
  timer_init();                             // 定时器初始化

  printf("\r\n=== APP %s @0x08003000 ===\r\n", APP_VERSION);

  // 设置默认工作时间（30秒）和停止时间（30秒）
  device_set_work_time(&g_dev_cfg, 10); // 默认工作时间 30秒
  device_set_stop_time(&g_dev_cfg, 10); // 默认停止时间 30秒

  // 设置默认模式，启动 LED 闪烁
  // dev_out_mode_t led_mode[DEV_CH_COUNT] = {
  //     DEV_OUT_ON,  DEV_OUT_ON,  DEV_OUT_ON,  DEV_OUT_OFF,
  //     DEV_OUT_OFF, DEV_OUT_OFF, DEV_OUT_OFF, DEV_OUT_OFF};
  // device_set_output_mode(&g_dev_cfg, led_mode);
  // 设置超声波的频率为 25kHz（测试）
  //  hw_usonic_set_freq(25000); // 设置超声波频率为 25kHz

  while (1) {
     uint16_t n = usart2_pull_chunk(g_rx_chunk, sizeof(g_rx_chunk));
     if (n) {
       // 喂给解析器（内部可一次解析多帧）
       proto_feed(&g_proto, g_rx_chunk, n);
     }
     report_manager_process();
  }
}
