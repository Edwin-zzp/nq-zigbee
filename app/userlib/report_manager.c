#include "report_manager.h"
#include "debug.h"
#include "device_logic.h"
#include "usart2_dma.h"
#include <string.h>


extern device_config_t g_dev_cfg;
extern device_runtime_t g_dev_rt;

typedef enum {
  REPORT_STATE_IDLE = 0,
  REPORT_STATE_WAIT_ACK,
} report_state_t;

typedef struct {
  uint8_t sensor_id[6];
  uint32_t elapsed_s;
  uint8_t send_pending;
  report_state_t state;
  uint32_t wait_ms;
  uint32_t ack_timeout_ms;
  uint32_t timestamp_s;
  uint32_t error_count;
} report_ctx_t;

static report_ctx_t s_report_ctx;

static void build_status_payload(uint8_t *out) {
  uint8_t status = 0;
  if (g_dev_rt.phase == DEV_PHASE_WORK)
    status |= 0x01; // RUN
  if (device_led_should_on(&g_dev_rt, 0))
    status |= 0x02; // LED1
  if (device_led_should_on(&g_dev_rt, 1))
    status |= 0x04; // LED2
  if (device_led_should_on(&g_dev_rt, 2))
    status |= 0x08; // LED3
  if (device_usonic_should_on(&g_dev_rt))
    status |= 0x10; // USONIC
  if (device_buzz_should_on(&g_dev_rt))
    status |= 0x20; // BUZZ
  out[0] = status;
}

static void build_le16(uint8_t out[2], uint16_t v) {
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void build_le32(uint8_t out[4], uint32_t v) {
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
  out[2] = (uint8_t)((v >> 16) & 0xFF);
  out[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void send_monitor_report(void) {
  uint8_t status_payload[1];
  uint8_t freq_payload[2];
  uint8_t work_payload[2];
  uint8_t stop_payload[2];
  uint8_t ts_payload[4];

  build_status_payload(status_payload);
  build_le16(freq_payload, device_get_us_freq(&g_dev_cfg));
  build_le16(work_payload, device_get_work_time(&g_dev_cfg));
  build_le16(stop_payload, device_get_stop_time(&g_dev_cfg));
  build_le32(ts_payload, s_report_ctx.timestamp_s);

  proto_param_tx_t params[5];
  params[0] = (proto_param_tx_t){.dtype = DT_STATE,
                                 .lflag = 0x01,
                                 .len = sizeof(status_payload),
                                 .data = status_payload};
  params[1] = (proto_param_tx_t){.dtype = DT_FREQ_STAT,
                                 .lflag = 0x01,
                                 .len = sizeof(freq_payload),
                                 .data = freq_payload};
  params[2] = (proto_param_tx_t){.dtype = DT_T_ON_R,
                                 .lflag = 0x01,
                                 .len = sizeof(work_payload),
                                 .data = work_payload};
  params[3] = (proto_param_tx_t){.dtype = DT_T_OFF_R,
                                 .lflag = 0x01,
                                 .len = sizeof(stop_payload),
                                 .data = stop_payload};
  params[4] = (proto_param_tx_t){.dtype = DT_TIMESTAMP,
                                 .lflag = 0x00,
                                 .len = sizeof(ts_payload),
                                 .data = ts_payload};

  uint8_t frame[128];
  size_t frame_len = proto_build_monitor_frame(s_report_ctx.sensor_id, params,
                                               5, frame, sizeof(frame));
  if (!frame_len)
    return;

  usart2_tx_write_blocking(frame, (uint16_t)frame_len);
  s_report_ctx.state = REPORT_STATE_WAIT_ACK;
  s_report_ctx.wait_ms = 0;
  s_report_ctx.send_pending = 0;

  printf("[REPORT] monitor frame sent (%uB)\r\n", (unsigned)frame_len);
}

void report_manager_init(const uint8_t sensor_id[6]) {
  memset(&s_report_ctx, 0, sizeof(s_report_ctx));
  if (sensor_id)
    memcpy(s_report_ctx.sensor_id, sensor_id, sizeof(s_report_ctx.sensor_id));
  s_report_ctx.ack_timeout_ms = 1000; // 默认 1s 超时
  s_report_ctx.state = REPORT_STATE_IDLE;
}

void report_manager_tick_1s(void) {
  s_report_ctx.timestamp_s++;

  uint16_t period = g_dev_cfg.dt_upload_period;
  if (period == 0)
    period = 60;

  s_report_ctx.elapsed_s++;
  if (s_report_ctx.elapsed_s >= period) {
    s_report_ctx.elapsed_s = 0;
    s_report_ctx.send_pending = 1;
  }
}

void report_manager_tick_ms(uint16_t delta_ms) {
  if (s_report_ctx.state != REPORT_STATE_WAIT_ACK)
    return;

  s_report_ctx.wait_ms += delta_ms;
  if (s_report_ctx.wait_ms >= s_report_ctx.ack_timeout_ms) {
    s_report_ctx.state = REPORT_STATE_IDLE;
    s_report_ctx.wait_ms = 0;
    s_report_ctx.error_count++;
    printf("[REPORT] monitor ack timeout (err=%lu)\r\n",
           (unsigned long)s_report_ctx.error_count);
  }
}

void report_manager_process(void) {
  if (s_report_ctx.state == REPORT_STATE_IDLE && s_report_ctx.send_pending) {
    send_monitor_report();
  }
}

void report_manager_on_monitor_ack(const proto_frame_t *frame) {
  if (!frame || !frame->has_cmd_status)
    return;

  if (s_report_ctx.state != REPORT_STATE_WAIT_ACK)
    return;

  s_report_ctx.state = REPORT_STATE_IDLE;
  s_report_ctx.wait_ms = 0;

  if (frame->cmd_status != 0xFF) {
    s_report_ctx.error_count++;
    printf("[REPORT] monitor ack error status=0x%02X (err=%lu)\r\n",
           frame->cmd_status, (unsigned long)s_report_ctx.error_count);
  } else {
    printf("[REPORT] monitor ack ok\r\n");
  }
}

uint32_t report_manager_get_error_count(void) {
  return s_report_ctx.error_count;
}