#include "device_logic.h"
#include "debug.h"
#include <string.h> // for memcpy

/* 内部：一些合理的参数范围（可根据协议表调整） */
#define DEV_WORK_TIME_MIN_S 1
#define DEV_WORK_TIME_MAX_S 3600
#define DEV_STOP_TIME_MIN_S 1
#define DEV_STOP_TIME_MAX_S 3600
#define DEV_US_FREQ_MIN_HZ 18000
#define DEV_US_FREQ_MAX_HZ 45000
#define DEV_US_FREQ_DEFAULT 25000

/* 内部：根据当前 phase 和配置，更新运行时“是否应当开启超声波/蜂鸣器” */
static void update_active_outputs(const device_config_t *cfg,
                                  device_runtime_t *rt) {
  /* 超声波：仅在 WORK 阶段且配置为 ON 时认为“应当开启” */
  if (rt->phase == DEV_PHASE_WORK &&
      cfg->out_mode[DEV_CH_USONIC] == DEV_OUT_ON) {
    rt->usonic_active = true;
  } else {
    rt->usonic_active = false;
  }

  /* 蜂鸣器：同理，按需要也可以改成“与某些告警有关” */
  if (rt->phase == DEV_PHASE_WORK && cfg->out_mode[DEV_CH_BUZZ] == DEV_OUT_ON) {
    rt->buzz_active = true;
  } else {
    rt->buzz_active = false;
  }
}

/* ================== 公共接口实现 ================== */

void device_logic_init(device_config_t *cfg, device_runtime_t *rt) {
  if (!cfg || !rt)
    return;

  /* 默认配置：全部安全关闭，由上位机通过协议设置为 ON */
  cfg->work_time_s = 30;      /* 默认工作 30s */
  cfg->stop_time_s = 30;      /* 默认停止 30s */
  cfg->dt_upload_period = 60; /* 默认上传周期 60s */
  cfg->us_freq_hz = DEV_US_FREQ_DEFAULT;

  for (int i = 0; i < DEV_CH_COUNT; ++i) {
    cfg->out_mode[i] = DEV_OUT_OFF; /* 默认所有通道 OFF */
  }

  /* 默认运行时状态 */
  rt->phase = DEV_PHASE_STOP; /* 上电先处于停止阶段 */
  rt->phase_elapsed_s = 0;

  for (int i = 0; i < 3; ++i) {
    rt->led_on[i] = false;
  }
  rt->led_blink_ms = 200; /* 默认闪烁周期 500ms */
  rt->led_blink_tick = 0;

  rt->usonic_active = false;
  rt->buzz_active = false;

  /* 确保根据当前 phase + 配置刷新一次 active 状态 */
  update_active_outputs(cfg, rt);
}

/* —— 协议设置相关 —— */

void device_set_work_time(device_config_t *cfg, uint16_t sec) {
  if (!cfg)
    return;
  if (sec < DEV_WORK_TIME_MIN_S)
    sec = DEV_WORK_TIME_MIN_S;
  if (sec > DEV_WORK_TIME_MAX_S)
    sec = DEV_WORK_TIME_MAX_S;
  cfg->work_time_s = sec;
}

void device_set_stop_time(device_config_t *cfg, uint16_t sec) {
  if (!cfg)
    return;
  if (sec < DEV_STOP_TIME_MIN_S)
    sec = DEV_STOP_TIME_MIN_S;
  if (sec > DEV_STOP_TIME_MAX_S)
    sec = DEV_STOP_TIME_MAX_S;
  cfg->stop_time_s = sec;
}

void device_set_us_freq(device_config_t *cfg, uint16_t hz) {
  if (!cfg)
    return;
  if (hz < DEV_US_FREQ_MIN_HZ)
    hz = DEV_US_FREQ_MIN_HZ;
  if (hz > DEV_US_FREQ_MAX_HZ)
    hz = DEV_US_FREQ_MAX_HZ;
  cfg->us_freq_hz = hz;
}

/* 注意：mode 数组中的 DEV_OUT_KEEP 表示“保持原配置”，不写入 cfg */
void device_set_output_mode(device_config_t *cfg,
                            const dev_out_mode_t mode[DEV_CH_COUNT]) {
  if (!cfg || !mode)
    return;
  for (int i = 0; i < DEV_CH_COUNT; ++i) {
    if (mode[i] == DEV_OUT_KEEP) {
      /* 保持当前配置，不做改动 */
      continue;
    }
    if (mode[i] == DEV_OUT_ON || mode[i] == DEV_OUT_OFF) {
      cfg->out_mode[i] = mode[i];
    }
    /* 其他值忽略 */
  }
}

/* —— 查询接口 —— */

uint16_t device_get_work_time(const device_config_t *cfg) {
  if (!cfg)
    return 0;
  return cfg->work_time_s;
}

uint16_t device_get_stop_time(const device_config_t *cfg) {
  if (!cfg)
    return 0;
  return cfg->stop_time_s;
}

uint16_t device_get_us_freq(const device_config_t *cfg) {
  if (!cfg)
    return 0;
  return cfg->us_freq_hz;
}

void device_get_output_mode(const device_config_t *cfg,
                            dev_out_mode_t mode_out[DEV_CH_COUNT]) {
  if (!cfg || !mode_out)
    return;
  for (int i = 0; i < DEV_CH_COUNT; ++i) {
    mode_out[i] = cfg->out_mode[i];
  }
}

/* —— 状态机：1 秒节拍 —— */
/* 在 1s 定时中断或 1s 周期任务中调用一次 */
void device_logic_tick_1s(const device_config_t *cfg, device_runtime_t *rt) {
  if (!cfg || !rt)
    return;

  // 打印当前状态（调试用）
  // if (rt->phase == DEV_PHASE_WORK) {
  //   printf("Entering WORK phase\n");
  // } else {
  //   printf("Entering STOP phase\n");
  // }

  rt->phase_elapsed_s++;

  if (rt->phase == DEV_PHASE_WORK) {
    /* 在工作阶段，超过工作时间则切换到停止阶段 */
    if (cfg->work_time_s > 0 && rt->phase_elapsed_s >= cfg->work_time_s) {
      rt->phase = DEV_PHASE_STOP;
      rt->phase_elapsed_s = 0;
    }
  } else { /* DEV_PHASE_STOP */
    /* 在停止阶段，超过停止时间则切换到工作阶段 */
    if (cfg->stop_time_s > 0 && rt->phase_elapsed_s >= cfg->stop_time_s) {
      rt->phase = DEV_PHASE_WORK;
      rt->phase_elapsed_s = 0;
    }
  }

  /* 每秒根据当前阶段 + 配置刷新一次超声波/蜂鸣器的 active 状态 */
  update_active_outputs(cfg, rt);
}

/* —— 状态机：毫秒节拍（LED 闪烁逻辑） —— */
/* 建议在 10ms 或 20ms 周期定时中调用，delta_ms 即该调用间隔时间 */
void device_logic_tick_ms(const device_config_t *cfg, device_runtime_t *rt,
                          uint16_t delta_ms) {
  if (!cfg || !rt)
    return;

  /* 非工作阶段：所有 LED 熄灭，闪烁计时清零 */
  if (rt->phase != DEV_PHASE_WORK) {
    for (int i = 0; i < 3; ++i) {
      rt->led_on[i] = false;
    }
    rt->led_blink_tick = 0;
    return;
  }

  /* 检查是否有任何 LED 配置为 ON（参与闪烁） */
  bool any_led_blink = false;
  for (int i = 0; i < 3; ++i) {
    if (cfg->out_mode[i] == DEV_OUT_ON) {
      any_led_blink = true;
      break;
    }
  }

  if (!any_led_blink) {
    /* 没有 LED 需要闪烁：全部关，计数清零 */
    for (int i = 0; i < 3; ++i) {
      rt->led_on[i] = false;
    }
    rt->led_blink_tick = 0;
    return;
  }

  /* 有 LED 需要闪烁：按 led_blink_ms 周期翻转 */
  if (rt->led_blink_ms == 0) {
    /* 防御：周期为 0 时，直接常亮 */
    for (int i = 0; i < 3; ++i) {
      rt->led_on[i] = (cfg->out_mode[i] == DEV_OUT_ON);
    }
    return;
  }

  rt->led_blink_tick += delta_ms;
  if (rt->led_blink_tick >= rt->led_blink_ms) {
    rt->led_blink_tick = 0;
    /* 翻转所有配置为 ON 的 LED，其他保持关 */
    for (int i = 0; i < 3; ++i) {
      if (cfg->out_mode[i] == DEV_OUT_ON) {
        rt->led_on[i] = !rt->led_on[i];
      } else {
        rt->led_on[i] = false;
      }
    }
  }
}

/* —— 控制层查询接口：根据 runtime 判断“现在该不该开” —— */

bool device_led_should_on(const device_runtime_t *rt, uint8_t led_index) {
  if (!rt)
    return false;
  if (led_index >= 3)
    return false; /* 仅支持 LED1~3 */

  return rt->led_on[led_index];
}

bool device_usonic_should_on(const device_runtime_t *rt) {
  if (!rt)
    return false;
  return rt->usonic_active;
}

bool device_buzz_should_on(const device_runtime_t *rt) {
  if (!rt)
    return false;
  return rt->buzz_active;
}
