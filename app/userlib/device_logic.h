// device_logic.h  —— 设备逻辑层（不直接操作硬件）

#ifndef DEVICE_LOGIC_H
#define DEVICE_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

// 工作模式枚举
typedef enum {
  DEV_PHASE_STOP = 0, // 停止阶段（T_OFF）
  DEV_PHASE_WORK = 1  // 工作阶段（T_ON）
} dev_phase_t;

// 输出通道枚举（对应 OUTPUT_CTRL 的 8 个通道）
typedef enum {
  DEV_CH_LED1 = 0,
  DEV_CH_LED2,
  DEV_CH_LED3,
  DEV_CH_USONIC, // 超声波
  DEV_CH_BUZZ,   // 蜂鸣器
  DEV_CH_RES6,
  DEV_CH_RES7,
  DEV_CH_RES8,
  DEV_CH_COUNT
} dev_channel_t;

// 通道控制模式：00 保持，01 强制开，02 强制关
typedef enum {
  DEV_OUT_KEEP = 0,
  DEV_OUT_ON = 1,
  DEV_OUT_OFF = 2
} dev_out_mode_t;

// ---------------- 配置参数（协议设置的量） ----------------
typedef struct {
  uint16_t work_time_s; // 工作时间 T_ON（秒）
  uint16_t stop_time_s; // 停止时间 T_OFF（秒）
  uint16_t us_freq_hz;  // 超声波频率设置
  dev_out_mode_t
      out_mode[DEV_CH_COUNT]; // 各通道目标模式（来自 OUTPUT_CTRL 参数）
  uint16_t dt_upload_period;  // 上传周期（秒），由上位机配置
} device_config_t;

// ---------------- 运行时状态（状态机内部使用） ----------------
typedef struct {
  dev_phase_t phase;        // 当前阶段：WORK / STOP
  uint16_t phase_elapsed_s; // 在当前阶段已运行的秒数
  // LED 闪烁相关
  bool led_on[3];          // LED1/2/3 当前亮灭状态（逻辑）
  uint16_t led_blink_ms;   // 闪烁周期（例如 500ms）
  uint16_t led_blink_tick; // 闪烁计时（ms 累加，达到周期翻转）

  // 超声波/蜂鸣器逻辑状态（是否应当工作）
  bool usonic_active; // 此刻应当开启超声波？
  bool buzz_active;   // 此刻应当开启蜂鸣器？
} device_runtime_t;

// 监测数据报文结构体（上报的监测数据）
typedef struct {
  uint16_t status_word; // 综合状态字
  uint16_t usonic_freq; // 超声波工作频率（状态）
  uint32_t work_time;   // 当前设置的工作时长（回读）
  uint32_t stop_time;   // 当前设置的停止时长（回读）
  uint32_t timestamp;   // 采集时间戳
} report_data_t;

// ================== 接口函数 ==================

// 初始化：填默认配置和状态（仅逻辑，不操作硬件）
void device_logic_init(device_config_t *cfg, device_runtime_t *rt);

// 协议层调用：设置参数（来自 ParameterList）
void device_set_work_time(device_config_t *cfg, uint16_t sec);
void device_set_stop_time(device_config_t *cfg, uint16_t sec);
void device_set_us_freq(device_config_t *cfg, uint16_t hz);
void device_set_output_mode(device_config_t *cfg,
                            const dev_out_mode_t mode[DEV_CH_COUNT]);

// 协议层查询：把当前配置打包给上报使用
uint16_t device_get_work_time(const device_config_t *cfg);
uint16_t device_get_stop_time(const device_config_t *cfg);
uint16_t device_get_us_freq(const device_config_t *cfg);
void device_get_output_mode(const device_config_t *cfg,
                            dev_out_mode_t mode_out[DEV_CH_COUNT]);

// 状态机：每 1 秒调用一次（驱动 WORK/STOP 周期）
// 在 main 的 1s 定时中断或定时任务里调用
void device_logic_tick_1s(const device_config_t *cfg, device_runtime_t *rt);

// 状态机：每 10~20ms 调用一次（用于 LED 闪烁逻辑）
void device_logic_tick_ms(const device_config_t *cfg, device_runtime_t *rt,
                          uint16_t delta_ms);

// 供控制层使用：根据当前 runtime 得到“应该输出什么”
bool device_led_should_on(const device_runtime_t *rt, uint8_t led_index);
bool device_usonic_should_on(const device_runtime_t *rt);
bool device_buzz_should_on(const device_runtime_t *rt);

#endif
