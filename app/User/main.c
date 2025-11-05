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
#include "zm32_cmd.h"
#include <string.h>

/* Global typedef */

/* Global define */
/* ====== 版本号（随意改） ====== */
#define APP_VERSION "v1.0.0"
#define REPORT_ERROR_RECOVERY_THRESHOLD 5u
#define ZM32_REJOIN_DELAY_MS 2000u
#define ZM32_POST_RESTORE_DELAY_MS 500u

/* Global Variable */

static proto_ctx_t g_proto;     // 协议解析上下文
static uint8_t g_rx_chunk[256]; // 从 usart2_pull_chunk 拉出来的块

device_config_t g_dev_cfg;
device_runtime_t g_dev_rt;

// 本机 SensorID（规范里的 SensorID），这里写死一份示例值，后面按实际烧录ID改
static uint8_t g_sensor_id[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
static zm32_dev_info_t g_zm32_info;
static bool g_zm32_info_valid = false;
static uint32_t g_last_recovery_err_mark = 0;
static bool g_recovery_in_progress = false;

static void print_mac_line(const char *prefix, const uint8_t mac[8]) {
  if (!prefix)
    prefix = "";
  if (!mac) {
    printf("%s(none)\r\n", prefix);
    return;
  }
  printf("%s%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n", prefix, mac[0],
         mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
}

static void configure_zm32_defaults(void) {
  uint8_t status = 0xFF;
  uint8_t ack_mac[8];

  zm32_cmd_init();

  if (zm32_cmd_set_send_mode(ZM32_SEND_MODE_DATA_UNICAST, &status)) {
    printf("[ZM32] send_mode status=0x%02X%s\r\n", status,
           (status == 0x00) ? " OK" : " ERR");
  } else {
    printf("[ZM32] send_mode command timeout\r\n");
  }

  if (zm32_cmd_set_target_network(0x0000u, &status)) {
    printf("[ZM32] target_network status=0x%02X%s\r\n", status,
           (status == 0x00) ? " OK" : " ERR");
  } else {
    printf("[ZM32] target_network command timeout\r\n");
  }

  if (zm32_cmd_set_target_mac(ZM32_MAC_COORDINATOR, &status, ack_mac)) {
    printf("[ZM32] target_mac status=0x%02X%s\r\n", status,
           (status == 0x00) ? " OK" : " ERR");
    print_mac_line("[ZM32] target_mac echo=", ack_mac);
  } else {
    printf("[ZM32] target_mac command timeout\r\n");
  }

  if (zm32_cmd_set_receive_mode(ZM32_MAC_LOCAL_EXEC, ZM32_RECV_MODE_DATA, &status,
                                ack_mac)) {
    printf("[ZM32] receive_mode status=0x%02X%s\r\n", status,
           (status == 0x00) ? " OK" : " ERR");
    print_mac_line("[ZM32] receive_mode echo=", ack_mac);
  } else {
    printf("[ZM32] receive_mode command timeout\r\n");
  }

  zm32_cmd_flush_rx();
}

static bool fetch_zm32_info(void) {
  zm32_dev_info_t info;
  if (!zm32_cmd_read_local_info(&info)) {
    printf("[ZM32] read dev info failed\r\n");
    return false;
  }

  g_zm32_info = info;
  g_zm32_info_valid = true;

  printf("[ZM32] PAN=0x%04X channel=%u net=0x%04X\r\n", info.pan_id,
         info.channel, info.net_addr);
  print_mac_line("[ZM32] MAC=", info.mac);
  return true;
}

static bool zm32_apply_pan_and_reset(uint16_t pan_id, const char *tag) {
  if (!g_zm32_info_valid)
    return false;

  uint8_t status = 0xFFu;
  if (!zm32_cmd_write_pan_id(g_zm32_info.mac, pan_id, &status)) {
    printf("%s write PAN command failed\r\n", tag ? tag : "[ZM32]");
    return false;
  }
  if (status != ZM32_STATUS_SUCCESS) {
    printf("%s write PAN status=0x%02X\r\n", tag ? tag : "[ZM32]", status);
    return false;
  }

  printf("%s set PAN=0x%04X\r\n", tag ? tag : "[ZM32]", pan_id);

  if (!zm32_cmd_reset_module(g_zm32_info.mac)) {
    printf("%s reset command failed\r\n", tag ? tag : "[ZM32]");
    return false;
  }

  printf("%s reset issued\r\n", tag ? tag : "[ZM32]");
  return true;
}

static void perform_zm32_recovery(void) {
  if (g_recovery_in_progress)
    return;

  g_recovery_in_progress = true;

  if (!g_zm32_info_valid && !fetch_zm32_info()) {
    printf("[RECOVERY] no device info available\r\n");
    g_recovery_in_progress = false;
    return;
  }

  printf("[RECOVERY] monitor ack errors=%lu -> rejoin cycle\r\n",
         (unsigned long)report_manager_get_error_count());

  uint16_t restore_pan = g_zm32_info.pan_id;
  bool ok = zm32_apply_pan_and_reset(0xFFFFu, "[RECOVERY]");
  if (ok) {
    Delay_Ms(ZM32_REJOIN_DELAY_MS);
    ok = zm32_apply_pan_and_reset(restore_pan, "[RECOVERY]");
  }

  if (ok) {
    Delay_Ms(ZM32_POST_RESTORE_DELAY_MS);
    configure_zm32_defaults();
    Delay_Ms(20);
    if (!fetch_zm32_info()) {
      printf("[RECOVERY] refresh dev info failed\r\n");
      g_zm32_info_valid = false;
    }
  } else {
    printf("[RECOVERY] cycle aborted\r\n");
  }

  report_manager_reset_errors();
  g_last_recovery_err_mark = 0;
  g_recovery_in_progress = false;
}

static void handle_report_errors(void) {
  uint32_t err = report_manager_get_error_count();
  if (err < REPORT_ERROR_RECOVERY_THRESHOLD)
    return;
  if (g_recovery_in_progress)
    return;
  if (err == g_last_recovery_err_mark)
    return;

  g_last_recovery_err_mark = err;
  perform_zm32_recovery();
}

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

static bool build_param_payload(uint16_t dtype, proto_param_tx_t *out,
                                uint8_t storage[DEV_CH_COUNT]) {
  if (!out || !storage)
    return false;

  switch (dtype) {
  case DT_UPLOAD_PERIOD: {
    write_le16(storage, g_dev_cfg.dt_upload_period);
    out->dtype = dtype;
    out->lflag = 0x01;
    out->len = 2;
    out->data = storage;
    return true;
  }
  case DT_OUTPUT_CTRL: {
    dev_out_mode_t mode[DEV_CH_COUNT];
    device_get_output_mode(&g_dev_cfg, mode);
    for (uint8_t i = 0; i < DEV_CH_COUNT; ++i)
      storage[i] = (uint8_t)mode[i];
    out->dtype = dtype;
    out->lflag = 0x01;
    out->len = DEV_CH_COUNT;
    out->data = storage;
    return true;
  }
  case DT_T_ON_SET: {
    write_le16(storage, device_get_work_time(&g_dev_cfg));
    out->dtype = dtype;
    out->lflag = 0x01;
    out->len = 2;
    out->data = storage;
    return true;
  }
  case DT_T_OFF_SET: {
    write_le16(storage, device_get_stop_time(&g_dev_cfg));
    out->dtype = dtype;
    out->lflag = 0x01;
    out->len = 2;
    out->data = storage;
    return true;
  }
  case DT_FREQ_SET: {
    write_le16(storage, device_get_us_freq(&g_dev_cfg));
    out->dtype = dtype;
    out->lflag = 0x01;
    out->len = 2;
    out->data = storage;
    return true;
  }
  default:
    return false;
  }
}

static uint16_t clamp_upload_period(uint16_t period) {
  if (period < 1)
    period = 1;
  if (period > 3600)
    period = 3600;
  return period;
}

static void send_param_query_ack(uint8_t req_set, const proto_frame_t *req) {
  const uint16_t supported[] = {DT_UPLOAD_PERIOD, DT_OUTPUT_CTRL, DT_T_ON_SET,
                                DT_T_OFF_SET,    DT_FREQ_SET};
  uint16_t request_list[sizeof(supported) / sizeof(supported[0])];
  uint8_t request_cnt = 0;

  bool query_all = true;
  if (req) {
    if ((req->hdr.param_count != 0x0Fu) && req->param_cnt > 0)
      query_all = false;
  }

  const uint8_t supported_cnt =
      (uint8_t)(sizeof(supported) / sizeof(supported[0]));

  if (query_all) {
    for (uint8_t i = 0; i < supported_cnt; ++i)
      request_list[request_cnt++] = supported[i];
  } else if (req) {
    for (uint8_t i = 0; i < req->param_cnt; ++i) {
      uint16_t dtype = req->params[i].dtype;
      for (uint8_t j = 0; j < supported_cnt; ++j) {
        if (supported[j] == dtype) {
          bool already_added = false;
          for (uint8_t k = 0; k < request_cnt; ++k) {
            if (request_list[k] == dtype) {
              already_added = true;
              break;
            }
          }
          if (!already_added && request_cnt < supported_cnt)
            request_list[request_cnt++] = dtype;
          break;
        }
      }
    }
  }

  if (request_cnt == 0) {
    printf("[CTRL-PARAM] no known params requested\r\n");
    send_control_status_ack(0x00);
    return;
  }

  proto_param_tx_t selected[sizeof(supported) / sizeof(supported[0])];
  uint8_t payload_storage[sizeof(supported) / sizeof(supported[0])][DEV_CH_COUNT];
  uint8_t selected_cnt = 0;

  for (uint8_t i = 0; i < request_cnt; ++i) {
    if (build_param_payload(request_list[i], &selected[selected_cnt],
                            payload_storage[selected_cnt])) {
      selected_cnt++;
    }
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

static void send_output_query_ack(uint8_t req_set, const proto_frame_t *req) {
  bool query_all = true;
  if (req) {
    if ((req->hdr.param_count != 0x0Fu) && req->param_cnt > 0)
      query_all = false;
  }

  bool want_output = query_all;

  if (!want_output && req) {
    for (uint8_t i = 0; i < req->param_cnt; ++i) {
      if (req->params[i].dtype == DT_OUTPUT_CTRL) {
        want_output = true;
        break;
      }
    }
  }

  if (!want_output) {
    printf("[CTRL-OUTPUT] no known params requested\r\n");
    uint8_t frame[32];
    size_t frame_len = proto_build_control_param_ack(
        g_sensor_id, CTRL_OUTPUT_QS, req_set, NULL, 0, frame, sizeof(frame));
    if (frame_len)
      usart2_tx_write_blocking(frame, (uint16_t)frame_len);
    return;
  }

  proto_param_tx_t param;
  uint8_t payload[DEV_CH_COUNT];
  if (!build_param_payload(DT_OUTPUT_CTRL, &param, payload)) {
    printf("[CTRL-OUTPUT] build payload failed\r\n");
    return;
  }

  uint8_t frame[96];
  size_t frame_len = proto_build_control_param_ack(
      g_sensor_id, CTRL_OUTPUT_QS, req_set, &param, 1, frame, sizeof(frame));
  if (!frame_len) {
    printf("[CTRL-OUTPUT] build query ack failed\r\n");
    return;
  }

  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-ACK-OUTPUT] query\r\n");
}

static void handle_output_set(const proto_frame_t *req) {
  if (!req || req->param_cnt == 0) {
    printf("[CTRL-OUTPUT] empty set request\r\n");
    uint8_t frame[32];
    size_t frame_len = proto_build_control_param_ack(
        g_sensor_id, CTRL_OUTPUT_QS, req ? req->req_set : 1, NULL, 0, frame,
        sizeof(frame));
    if (frame_len)
      usart2_tx_write_blocking(frame, (uint16_t)frame_len);
    return;
  }

  bool ok = true;
  bool any_known = false;

  dev_out_mode_t new_modes[DEV_CH_COUNT];
  device_get_output_mode(&g_dev_cfg, new_modes);

  for (uint8_t i = 0; i < req->param_cnt; ++i) {
    const proto_param_t *p = &req->params[i];
    if (p->dtype != DT_OUTPUT_CTRL) {
      printf("[CTRL-OUTPUT] unsupported dtype=0x%04X\r\n", p->dtype);
      ok = false;
      break;
    }
    any_known = true;
    if (p->lflag != 0x01 || p->len != DEV_CH_COUNT) {
      printf("[CTRL-OUTPUT] invalid len=%lu lflag=0x%02X\r\n",
             (unsigned long)p->len, p->lflag);
      ok = false;
      break;
    }
    for (uint8_t ch = 0; ch < DEV_CH_COUNT; ++ch) {
      uint8_t mode = p->val[ch];
      if (mode > DEV_OUT_OFF) {
        printf("[CTRL-OUTPUT] bad mode[%u]=%u\r\n", ch, mode);
        ok = false;
        break;
      }
      new_modes[ch] = (dev_out_mode_t)mode;
    }
    if (!ok)
      break;
  }

  if (!ok || !any_known) {
    uint8_t frame[32];
    size_t frame_len = proto_build_control_param_ack(
        g_sensor_id, CTRL_OUTPUT_QS, req->req_set, NULL, 0, frame,
        sizeof(frame));
    if (frame_len)
      usart2_tx_write_blocking(frame, (uint16_t)frame_len);
    if (!any_known)
      printf("[CTRL-OUTPUT] no supported parameters in set\r\n");
    return;
  }

  device_set_output_mode(&g_dev_cfg, new_modes);

  proto_param_tx_t param;
  uint8_t payload[DEV_CH_COUNT];
  if (!build_param_payload(DT_OUTPUT_CTRL, &param, payload)) {
    printf("[CTRL-OUTPUT] build response payload failed\r\n");
    return;
  }

  uint8_t frame[96];
  size_t frame_len = proto_build_control_param_ack(
      g_sensor_id, CTRL_OUTPUT_QS, req->req_set, &param, 1, frame,
      sizeof(frame));
  if (!frame_len) {
    printf("[CTRL-OUTPUT] build set ack failed\r\n");
    return;
  }

  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-ACK-OUTPUT] set applied\r\n");
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

static void handle_param_set(const proto_frame_t *req) {
  if (!req || req->param_cnt == 0) {
    printf("[CTRL-PARAM] empty set request\r\n");
    uint8_t frame[32];
    size_t frame_len = proto_build_control_param_ack(
        g_sensor_id, CTRL_PARAM_QS, req->req_set, NULL, 0, frame,
        sizeof(frame));
    if (frame_len)
      usart2_tx_write_blocking(frame, (uint16_t)frame_len);
    return;
  }

  bool ok = true;
  bool any_known = false;

  uint16_t new_upload = g_dev_cfg.dt_upload_period;
  uint16_t new_work = device_get_work_time(&g_dev_cfg);
  uint16_t new_stop = device_get_stop_time(&g_dev_cfg);
  uint16_t new_freq = device_get_us_freq(&g_dev_cfg);
  dev_out_mode_t new_modes[DEV_CH_COUNT];
  for (uint8_t i = 0; i < DEV_CH_COUNT; ++i)
    new_modes[i] = DEV_OUT_KEEP;

  bool set_upload = false;
  bool set_work = false;
  bool set_stop = false;
  bool set_freq = false;
  bool set_output = false;

  for (uint8_t i = 0; i < req->param_cnt; ++i) {
    const proto_param_t *p = &req->params[i];
    switch (p->dtype) {
    case DT_UPLOAD_PERIOD:
      any_known = true;
      if (p->lflag != 0x01 || p->len != 2) {
        ok = false;
        printf("[CTRL-PARAM] upload period invalid len=%lu lflag=0x%02X\r\n",
               (unsigned long)p->len, p->lflag);
        break;
      }
      new_upload = (uint16_t)(p->val[0] | (p->val[1] << 8));
      if (new_upload == 0) {
        ok = false;
        printf("[CTRL-PARAM] upload period zero invalid\r\n");
        break;
      }
      new_upload = clamp_upload_period(new_upload);
      set_upload = true;
      break;
    case DT_OUTPUT_CTRL:
      any_known = true;
      if (p->lflag != 0x01 || p->len != DEV_CH_COUNT) {
        ok = false;
        printf("[CTRL-PARAM] output ctrl invalid len=%lu lflag=0x%02X\r\n",
               (unsigned long)p->len, p->lflag);
        break;
      }
      for (uint8_t ch = 0; ch < DEV_CH_COUNT; ++ch) {
        uint8_t mode = p->val[ch];
        if (mode > DEV_OUT_OFF) {
          ok = false;
          printf("[CTRL-PARAM] output ctrl bad mode[%u]=%u\r\n", ch, mode);
          break;
        }
        new_modes[ch] = (dev_out_mode_t)mode;
      }
      if (!ok)
        break;
      set_output = true;
      break;
    case DT_T_ON_SET:
      any_known = true;
      if (p->lflag != 0x01 || p->len != 2) {
        ok = false;
        printf("[CTRL-PARAM] work time invalid len=%lu lflag=0x%02X\r\n",
               (unsigned long)p->len, p->lflag);
        break;
      }
      new_work = (uint16_t)(p->val[0] | (p->val[1] << 8));
      set_work = true;
      break;
    case DT_T_OFF_SET:
      any_known = true;
      if (p->lflag != 0x01 || p->len != 2) {
        ok = false;
        printf("[CTRL-PARAM] stop time invalid len=%lu lflag=0x%02X\r\n",
               (unsigned long)p->len, p->lflag);
        break;
      }
      new_stop = (uint16_t)(p->val[0] | (p->val[1] << 8));
      set_stop = true;
      break;
    case DT_FREQ_SET:
      any_known = true;
      if (p->lflag != 0x01 || p->len != 2) {
        ok = false;
        printf("[CTRL-PARAM] freq invalid len=%lu lflag=0x%02X\r\n",
               (unsigned long)p->len, p->lflag);
        break;
      }
      new_freq = (uint16_t)(p->val[0] | (p->val[1] << 8));
      set_freq = true;
      break;
    default:
      printf("[CTRL-PARAM] unsupported dtype=0x%04X\r\n", p->dtype);
      ok = false;
      break;
    }

    if (!ok)
      break;
  }

  if (!ok || !any_known) {
    uint8_t frame[32];
    size_t frame_len = proto_build_control_param_ack(g_sensor_id, CTRL_PARAM_QS,
                                                     req->req_set, NULL, 0, frame,
                                                     sizeof(frame));
    if (frame_len)
      usart2_tx_write_blocking(frame, (uint16_t)frame_len);
    if (!any_known)
      printf("[CTRL-PARAM] no supported parameters in set\r\n");
    return;
  }

  if (set_upload)
    g_dev_cfg.dt_upload_period = new_upload;
  if (set_work)
    device_set_work_time(&g_dev_cfg, new_work);
  if (set_stop)
    device_set_stop_time(&g_dev_cfg, new_stop);
  if (set_freq)
    device_set_us_freq(&g_dev_cfg, new_freq);
  if (set_output)
    device_set_output_mode(&g_dev_cfg, new_modes);

  proto_param_tx_t params[5];
  uint8_t payloads[5][DEV_CH_COUNT];
  uint8_t param_cnt = 0;

  if (set_upload && build_param_payload(DT_UPLOAD_PERIOD, &params[param_cnt],
                                        payloads[param_cnt]))
    param_cnt++;
  if (set_output && build_param_payload(DT_OUTPUT_CTRL, &params[param_cnt],
                                        payloads[param_cnt]))
    param_cnt++;
  if (set_work && build_param_payload(DT_T_ON_SET, &params[param_cnt],
                                      payloads[param_cnt]))
    param_cnt++;
  if (set_stop && build_param_payload(DT_T_OFF_SET, &params[param_cnt],
                                      payloads[param_cnt]))
    param_cnt++;
  if (set_freq && build_param_payload(DT_FREQ_SET, &params[param_cnt],
                                      payloads[param_cnt]))
    param_cnt++;

  uint8_t frame[160];
  size_t frame_len = proto_build_control_param_ack(
      g_sensor_id, CTRL_PARAM_QS, req->req_set, params, param_cnt, frame,
      sizeof(frame));
  if (!frame_len) {
    printf("[CTRL-PARAM] build set ack failed\r\n");
    return;
  }

  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  printf("[CTRL-PARAM] set applied ack cnt=%u\r\n", param_cnt);
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
      handle_param_set(f);
    }
  } else if (f->ctrl_type == CTRL_OUTPUT_QS) {
    if (f->req_set == 0) {
      send_output_query_ack(f->req_set, f);
    } else {
      handle_output_set(f);
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

  configure_zm32_defaults();
  fetch_zm32_info();

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
    handle_report_errors();
  }
}
