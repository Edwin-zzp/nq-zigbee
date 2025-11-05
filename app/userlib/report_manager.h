// report_manager.h —— 监测数据定时上报与 ACK 管理

#ifndef REPORT_MANAGER_H
#define REPORT_MANAGER_H

#include <stdint.h>
#include "proto.h"

void report_manager_init(const uint8_t sensor_id[6]);
void report_manager_tick_1s(void);
void report_manager_tick_ms(uint16_t delta_ms);
void report_manager_process(void);
void report_manager_on_monitor_ack(const proto_frame_t *frame);
void report_manager_request_immediate(void);
void report_manager_set_sensor_id(const uint8_t sensor_id[6]);
uint32_t report_manager_get_error_count(void);

#endif
