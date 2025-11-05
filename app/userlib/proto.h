// =============================================================
// 文件：inc/proto.h —— 协议帧数据结构（v0.6，对齐规范字段名的注释）
// 说明：不改任何变量名，仅用注释标注其与规范字段的对应关系。
// =============================================================
#ifndef PROTO_H
#define PROTO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- 报文类型（规范：PacketType / MSG_TYPE） ----------------
#define PKT_MONITOR_DATA 0x00 // 000：监测数据（Data=ParameterList）
#define PKT_MONITOR_ACK 0x01  // 001：监测响应（Data=CommandStatus 1B）
#define PKT_CONTROL                                                            \
  0x04 // 100：控制请求（Data=CtrlHead(+可选ID) / CtrlHead+ParameterList）
#define PKT_CONTROL_ACK                                                        \
  0x05 // 101：控制响应（Data=见各分支：CommandStatus 或 ID 回显 或
       // ParameterList）

// ---------------- 控制类型（规范：CrtlType） ----------------
#define CTRL_REQ_MONITOR 0x01 // 请求监测数据
#define CTRL_ID_QS 0x06       // ID 查询/设置（不走 ParameterList）
#define CTRL_PARAM_QS 0x64    // 参数 查询/设置（走 ParameterList）
#define CTRL_OUTPUT_QS 0x65   // 输出 查询/设置（走 ParameterList）

// ---------------- DataType（规范：ParameterList 的 14-bit DataType）
// ---------------- 提示：以下编号仅为示例，请替换为你们文档 v0.6 的最终编号。
#define DT_STATE 0x3800         // 综合状态字（常用 LF=01, len=1）
#define DT_FREQ_STAT 0x3801     // 频率(状态) U16（LF=01, len=2）
#define DT_T_ON_R 0x3802        // 工作时长(回读) U16（LF=01, len=2）
#define DT_T_OFF_R 0x3803       // 停止时长(回读) U16（LF=01, len=2）
#define DT_TIMESTAMP 0x3804     // 时间戳 U32（LF=00, 无length, 固定4B）
#define DT_UPLOAD_PERIOD 0x3805 // 上传周期 U16（LF=01, len=2）
#define DT_OUTPUT_CTRL 0x3806   // 输出控制 Byte[8]（LF=01, len=8）
#define DT_T_ON_SET 0x3807      // 工作时长(设置) U16（LF=01, len=2）
#define DT_T_OFF_SET 0x3808     // 停止时长(设置) U16（LF=01, len=2）
#define DT_FREQ_SET 0x3809      // 频率(设置) U16（LF=01, len=2）

// ---------------- ParameterList 单项结构 ----------------
// 规范对应：
//   - dtype  ↔ DataType(14b)
//   - lflag  ↔ LengthFlag(2b) 语义：00=无length固定4B；01=length 1B；10=length
//   2B；11=length 3B
//   - len    ↔ length(xB)      （当 lflag==00 时，本字段为解析出的数据长度 4）
//   - val    ↔ Data(lenB)
// ⚠ 不改变量名，仅用注释标注其语义。
typedef struct {
  uint16_t dtype; // 规范：DataType(14b)。存放在低14位，发送时 (dtype<<2)|lflag
                  // → TypeLen(2B,LE)
  uint8_t
      lflag; // 规范：LengthFlag(2b)。00:无length=固定4B；01:len1B；10:len2B；11:len3B
  uint32_t len;    // 规范：length。lflag==00 时，解析器赋值为
                   // 4；其余为读取到的长度（LE）
  uint8_t val[64]; // 规范：Data（内容长度见 len）
} proto_param_t;

// ParameterList 发送端的描述结构
typedef struct {
  uint16_t dtype;      // ↔ DataType(14b)
  uint8_t lflag;       // ↔ LengthFlag(2b)
  uint32_t len;        // ↔ length(xB)
  const uint8_t *data; // ↔ Data 指针
} proto_param_tx_t;

// ---------------- SensorID（规范：SensorID，6 字节） ----------------
// 此结构仅保留原始 6B；若需拆分厂商/版本/序列，可在业务层解析
// ⚠ 本机的 SensorID 由 proto_set_local_id() 进行配置，用于帧过滤

typedef struct {
  uint8_t raw[6]; // 规范：SensorID(6B)
} proto_sensor_id_t;

// ---------------- 帧头（规范：帧头的 3 个字段） ----------------
//   - ctrl_raw    ↔ 控制字节（未拆位，供调试）
//   - param_count ↔ DataLen(4b)（参数项个数，仅对携带 ParameterList 的帧有效）
//   - frag        ↔ FragInd(1b)
//   - pkt_type    ↔ PacketType(3b)

typedef struct {
  proto_sensor_id_t id; // SensorID(6B)
  uint8_t ctrl_raw;     // 控制字节（原值）
  uint8_t param_count;  // DataLen(4b)：参数项个数
  uint8_t frag;         // FragInd(1b)
  uint8_t pkt_type;     // PacketType(3b)
} proto_hdr_t;

// ---------------- 完整解析结果（与规范字段映射标注） ----------------
typedef struct {
  proto_hdr_t hdr; // ↔ 帧头三字段 + SensorID

  // 控制头（规范：CtrlHead= CtlType[7:1] | RequestSetFlag[0]）
  uint8_t has_ctrl;  // Data 中是否携带 CtrlHead（100 必有；101 仅 ID 回应有）
  uint8_t ctrl_type; // ↔ CtlType（7b）
  uint8_t req_set;   // ↔ RequestSetFlag（1b，0=查询/1=设置）

  // ID 负载（仅 CTRL_ID_QS 场景）
  uint8_t has_id_payload; // Data 中是否带 6B ID 载荷
  uint8_t id_payload[6];  // ↔ SensorID（查询/设置的目标或新ID）

  // CommandStatus（规范：001 或 101 的“仅状态”回应）
  uint8_t has_cmd_status; // 是否包含 1B 状态
  uint8_t cmd_status;     // ↔ CommandStatus（0xFF/0x00）

  // ParameterList（规范：N 项参数）
  uint8_t param_cnt;        // 实际解析得到的参数项个数
  proto_param_t params[16]; // 每项见上方 proto_param_t 注释

  // 过滤与调试
  uint8_t dropped_by_id; // 本机 ID 不匹配而被丢弃（但仍完整解析长度以跳过）

  // 原始帧缓存
  uint16_t raw_len; // 原始帧总长（含 CRC）
  uint8_t raw[256];
} proto_frame_t;

// ---------------- 解析上下文（含“本机 SensorID”配置） ----------------
typedef struct proto_ctx_s proto_ctx_t;
typedef void (*proto_on_frame_cb)(proto_ctx_t *, const proto_frame_t *);

typedef struct proto_ctx_s {
  uint8_t acc[256]; // 累积缓冲（流式解析）
  uint16_t acc_len;

  uint8_t my_id[6];  // ★ 本机 SensorID（用于过滤） ↔ 规范：SensorID
  uint8_t my_id_set; // 是否启用本机 ID 匹配检查

  // 回调（按 PacketType 分派）
  proto_on_frame_cb on_monitor;     // 000
  proto_on_frame_cb on_monitor_ack; // 001
  proto_on_frame_cb on_control;     // 100
  proto_on_frame_cb on_control_ack; // 101
} proto_ctx_t;

// API —— 初始化/设置本机 ID/喂入分片数据
void proto_init(proto_ctx_t *ctx);
void proto_set_local_id(proto_ctx_t *ctx,
                        const uint8_t id6[6]); // ★ 设置本机 SensorID 的位置
void proto_feed(proto_ctx_t *ctx, const uint8_t *data, uint16_t len);

// 发送端工具：编码一项 ParameterList / 构造 000 监测数据报文
size_t proto_encode_param(uint8_t *out, size_t maxlen,
                          const proto_param_tx_t *param);
size_t proto_build_monitor_frame(const uint8_t sensor_id[6],
                                 const proto_param_tx_t *params,
                                 uint8_t param_cnt, uint8_t *out,
                                 size_t maxlen);

#ifdef __cplusplus
}
#endif
#endif // PROTO_H