// device_hw.h —— 设备逻辑层 ↔ CH32 实际硬件 的适配层
#ifndef DEVICE_HW_H
#define DEVICE_HW_H

#include "board.h"
#include "device_logic.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化所有与设备相关的硬件：LED GPIO、蜂鸣器 GPIO、超声波 PWM 定时器等
void device_hw_init(void);

// 将当前逻辑状态（g_dev_cfg / g_dev_rt）应用到硬件
// 一般在周期任务里调用，比如 10~20ms 调一次
void device_hw_apply(const device_config_t *cfg, const device_runtime_t *rt);

void hw_usonic_set_freq(uint16_t freq_hz); // 声明超声波频率设置函数

#ifdef __cplusplus
}
#endif

#endif // DEVICE_HW_H
