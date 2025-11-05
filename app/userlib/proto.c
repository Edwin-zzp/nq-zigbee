// =============================================================
// 文件：src/proto.c —— 解析实现（v0.6，带与规范字段映射的注释）
// =============================================================
#include "proto.h"
#include "crc16.h"
#include <string.h>

// ---- 发送端：编码 ParameterList 项 ----
size_t proto_encode_param(uint8_t *out, size_t maxlen,
                          const proto_param_tx_t *param) {
  if (!out || !param)
    return 0;
  if (maxlen < 2)
    return 0;

  uint16_t type_len = (uint16_t)((param->dtype << 2) | (param->lflag & 0x03));
  out[0] = (uint8_t)(type_len & 0xFF);
  out[1] = (uint8_t)((type_len >> 8) & 0xFF);

  size_t pos = 2;
  uint32_t payload_len = param->len;

  if (param->lflag == 0) {
    // 无 length 字段，数据固定 4B
    if (payload_len != 4)
      return 0;
  } else {
    uint8_t len_bytes =
        (param->lflag == 1) ? 1 : (param->lflag == 2 ? 2 : 3);
    if (maxlen < pos + len_bytes)
      return 0;
    for (uint8_t i = 0; i < len_bytes; ++i)
      out[pos + i] = (uint8_t)((payload_len >> (8 * i)) & 0xFF);
    pos += len_bytes;
  }

  if (payload_len > 0) {
    if (!param->data)
      return 0;
    if (maxlen < pos + payload_len)
      return 0;
    memcpy(&out[pos], param->data, payload_len);
    pos += payload_len;
  }

  return pos;
}

size_t proto_build_monitor_frame(const uint8_t sensor_id[6],
                                 const proto_param_tx_t *params,
                                 uint8_t param_cnt,
                                 uint8_t *out,
                                 size_t maxlen) {
  if (!sensor_id || !out)
    return 0;
  if (param_cnt > 0x0F)
    return 0; // DataLen 仅 4 bit

  size_t pos = 0;
  if (maxlen < 6)
    return 0;
  memcpy(out, sensor_id, 6);
  pos += 6;

  if (maxlen < pos + 1)
    return 0;
  uint8_t ctrl = (uint8_t)((param_cnt & 0x0F) << 4); // DataLen & Frag=0
  ctrl |= PKT_MONITOR_DATA;                          // PacketType=000
  out[pos++] = ctrl;

  for (uint8_t i = 0; i < param_cnt; ++i) {
    size_t used = proto_encode_param(&out[pos], maxlen - pos, &params[i]);
    if (!used)
      return 0;
    pos += used;
  }

  if (maxlen < pos + 2)
    return 0;
  uint16_t crc = crc16_modbus(out, pos);
  out[pos++] = (uint8_t)(crc & 0xFF);
  out[pos++] = (uint8_t)((crc >> 8) & 0xFF);

  return pos;
}

size_t proto_build_control_status_ack(const uint8_t sensor_id[6],
                                      uint8_t status,
                                      uint8_t *out,
                                      size_t maxlen) {
  if (!sensor_id || !out)
    return 0;
  const size_t needed = 6u + 1u + 1u + 2u; // SensorID + ctrl + status + CRC
  if (maxlen < needed)
    return 0;

  size_t pos = 0;
  memcpy(out, sensor_id, 6);
  pos += 6;

  out[pos++] = PKT_CONTROL_ACK; // DataLen=0, Frag=0, PacketType=101b
  out[pos++] = status;

  uint16_t crc = crc16_modbus(out, pos);
  out[pos++] = (uint8_t)(crc & 0xFF);
  out[pos++] = (uint8_t)((crc >> 8) & 0xFF);

  return pos;
}

size_t proto_build_control_id_ack(const uint8_t sensor_id[6],
                                  uint8_t ctrl_type,
                                  uint8_t req_set,
                                  const uint8_t id_payload[6],
                                  uint8_t *out,
                                  size_t maxlen) {
  if (!sensor_id || !id_payload || !out)
    return 0;

  const size_t needed = 6u + 1u + 1u + 6u + 2u;
  if (maxlen < needed)
    return 0;

  size_t pos = 0;
  memcpy(out, sensor_id, 6);
  pos += 6;

  uint8_t ctrl = (uint8_t)(((uint8_t)1u << 4) | PKT_CONTROL_ACK);
  out[pos++] = ctrl;

  uint8_t head = (uint8_t)((ctrl_type << 1) & 0xFEu);
  head |= (req_set & 0x01u);
  out[pos++] = head;

  memcpy(&out[pos], id_payload, 6);
  pos += 6;

  uint16_t crc = crc16_modbus(out, pos);
  out[pos++] = (uint8_t)(crc & 0xFF);
  out[pos++] = (uint8_t)((crc >> 8) & 0xFF);

  return pos;
}

size_t proto_build_control_param_ack(const uint8_t sensor_id[6],
                                     uint8_t ctrl_type,
                                     uint8_t req_set,
                                     const proto_param_tx_t *params,
                                     uint8_t param_cnt,
                                     uint8_t *out,
                                     size_t maxlen) {
  if (!sensor_id || !out)
    return 0;
  if (param_cnt > 0x0Fu)
    return 0;
  if (!params && param_cnt > 0)
    return 0;

  size_t pos = 0;
  if (maxlen < 6)
    return 0;
  memcpy(out, sensor_id, 6);
  pos += 6;

  if (maxlen < pos + 1)
    return 0;
  uint8_t ctrl = (uint8_t)((param_cnt & 0x0Fu) << 4);
  ctrl |= PKT_CONTROL_ACK;
  out[pos++] = ctrl;

  if (maxlen < pos + 1)
    return 0;
  uint8_t head = (uint8_t)((ctrl_type << 1) & 0xFEu);
  head |= (req_set & 0x01u);
  out[pos++] = head;

  for (uint8_t i = 0; i < param_cnt; ++i) {
    size_t used = proto_encode_param(&out[pos], maxlen - pos, &params[i]);
    if (!used)
      return 0;
    pos += used;
  }

  if (maxlen < pos + 2)
    return 0;
  uint16_t crc = crc16_modbus(out, pos);
  out[pos++] = (uint8_t)(crc & 0xFF);
  out[pos++] = (uint8_t)((crc >> 8) & 0xFF);

  return pos;
}

// ---- 工具：从控制字节拆位（规范：DataLen/FragInd/PacketType） ----
static inline uint8_t _get_param_count(uint8_t ctrl) {
  return (ctrl >> 4) & 0x0F;
} // ↔ DataLen(4b)
static inline uint8_t _get_frag(uint8_t ctrl) {
  return (ctrl >> 3) & 0x01;
} // ↔ FragInd(1b)
static inline uint8_t _get_pkt(uint8_t ctrl) {
  return (ctrl & 0x07);
} // ↔ PacketType(3b)

// ---- API：上下文初始化与本机 SensorID 配置 ----
void proto_init(proto_ctx_t *ctx) { memset(ctx, 0, sizeof(*ctx)); }
void proto_set_local_id(proto_ctx_t *ctx, const uint8_t id6[6]) {
  memcpy(ctx->my_id, id6, 6);
  ctx->my_id_set = 1;
}

// ---- 解析 ParameterList 的一项（规范：TypeLen + length(可选) + Data） ----
// LengthFlag 语义：00=无length固定4B；01=len1B；10=len2B；11=len3B
static uint16_t parse_one_param(const uint8_t *p, uint16_t remain,
                                proto_param_t *out) {
  if (remain < 2)
    return 0;
  uint16_t typeLen = (uint16_t)(p[0] | (p[1] << 8)); // LE
  out->lflag = (uint8_t)(typeLen & 0x03);            // ↔ LengthFlag(2b)
  out->dtype = (uint16_t)(typeLen >> 2);             // ↔ DataType(14b)

  const uint8_t *q = p + 2;
  uint16_t left = remain - 2;

  uint32_t dlen = 0; // ↔ length(xB)
  if (out->lflag == 0) {
    dlen = 4;
  } // 00：无 length，固定 4 字节
  else {
    uint8_t nlen = (out->lflag == 1) ? 1 : (out->lflag == 2 ? 2 : 3);
    if (left < nlen)
      return 0;
    dlen = 0;
    for (uint8_t i = 0; i < nlen; i++)
      dlen |= ((uint32_t)q[i] << (8 * i)); // LE
    q += nlen;
    left -= nlen;
  }
  if (left < dlen)
    return 0;

  out->len = dlen; // ↔ 数据长度
  uint32_t cp = (dlen > sizeof(out->val)) ? sizeof(out->val) : dlen;
  memcpy(out->val, q, cp);           // ↔ Data
  return (uint16_t)((q - p) + dlen); // 消费总字节数（TypeLen + length? + Data）
}

// ---- 尝试解析一整帧（规范：SensorID → 控制字节 → Data → CRC16） ----
static uint16_t try_parse_one(proto_ctx_t *ctx, const uint8_t *buf,
                              uint16_t len, proto_frame_t *f) {
  if (len < 10)
    return 0; // 最小：6(ID)+1(ctrl)+1(data最小)+2(CRC)
  memset(f, 0, sizeof(*f));
  uint16_t pos = 0;

  // SensorID(6B)
  memcpy(f->hdr.id.raw, &buf[pos], 6);
  pos += 6; // ↔ SensorID

  // ★ 本机 ID 过滤（不匹配即标记丢弃，但仍要解析长度确保丢弃后能继续同步）
  if (ctx && ctx->my_id_set && memcmp(f->hdr.id.raw, ctx->my_id, 6) != 0) {
    f->dropped_by_id = 1;
  }

  // 控制字节（DataLen/FragInd/PacketType）
  uint8_t ctrl = buf[pos++];
  f->hdr.ctrl_raw = ctrl;                      // ↔ 控制字节原值
  f->hdr.param_count = _get_param_count(ctrl); // ↔ DataLen(4b)：参数项个数
  f->hdr.frag = _get_frag(ctrl);               // ↔ FragInd
  f->hdr.pkt_type = _get_pkt(ctrl);            // ↔ PacketType

  // Data 区按 PacketType 解析
  if (f->hdr.pkt_type ==
      PKT_MONITOR_DATA) { // 000：监测上报 → ParameterList(N项)
    uint8_t expect = f->hdr.param_count;
    if (expect == 0x0Fu)
      expect = 0;
    for (uint8_t i = 0; i < expect; i++) {
      if (f->param_cnt >= 16)
        return 0;
      uint16_t used = parse_one_param(&buf[pos], (uint16_t)(len - pos),
                                      &f->params[f->param_cnt]);
      if (!used)
        return 0;
      pos += used;
      f->param_cnt++;
    }
  } else if (f->hdr.pkt_type ==
             PKT_MONITOR_ACK) { // 001：监测响应 → 1B CommandStatus
    if (len - pos < 1)
      return 0;
    f->has_cmd_status = 1;
    f->cmd_status = buf[pos++];
  } else if (f->hdr.pkt_type == PKT_CONTROL) { // 100：控制请求
    if (len - pos < 1)
      return 0;
    f->has_ctrl = 1; // CtrlHead 必有
    uint8_t head = buf[pos++];
    f->ctrl_type = (head >> 1);
    f->req_set = (head & 0x01); // ↔ CtlType/RequestSetFlag
    if (f->ctrl_type ==
        CTRL_ID_QS) { // ID 查询/设置：后跟 6B 负载（查询可为全0）
      if (len - pos >= 6) {
        memcpy(f->id_payload, &buf[pos], 6);
        f->has_id_payload = 1;
        pos += 6;
      }
    } else { // 参数/输出：ParameterList（N项，可为0）
      uint8_t expect = f->hdr.param_count;
      if (expect == 0x0Fu)
        expect = 0;
      for (uint8_t i = 0; i < expect; i++) {
        if (f->param_cnt >= 16)
          return 0;
        uint16_t used = parse_one_param(&buf[pos], (uint16_t)(len - pos),
                                        &f->params[f->param_cnt]);
        if (!used)
          return 0;
        pos += used;
        f->param_cnt++;
      }
    }
  } else if (f->hdr.pkt_type == PKT_CONTROL_ACK) { // 101：控制响应
    // ① 仅状态：CommandStatus(1B)
    // ② ID 回应：CtrlHead(1B) + SensorID(6B)
    // ③ 参数/输出回应：可选 CtrlHead + ParameterList（N 项）
    uint16_t rem_no_crc = (uint16_t)(len - pos - 2);
    if ((int)rem_no_crc == 1) {
      f->has_cmd_status = 1;
      f->cmd_status = buf[pos++];
    } else {
      uint8_t maybe_head = buf[pos];
      uint8_t mt = (maybe_head >> 1);
      if ((mt == CTRL_ID_QS) && (rem_no_crc >= 7)) {
        f->has_ctrl = 1;
        f->ctrl_type = mt;
        f->req_set = (maybe_head & 0x01);
        pos++;
        memcpy(f->id_payload, &buf[pos], 6);
        pos += 6;
        f->has_id_payload = 1;
      } else {
        if (rem_no_crc >= 1) {
          if ((mt == CTRL_PARAM_QS) || (mt == CTRL_OUTPUT_QS)) {
            f->has_ctrl = 1;
            f->ctrl_type = mt;
            f->req_set = (maybe_head & 0x01);
            pos++;
          }
        }
        uint8_t expect = f->hdr.param_count;
        if (expect == 0x0Fu)
          expect = 0;
        for (uint8_t i = 0; i < expect; i++) {
          if (f->param_cnt >= 16)
            return 0;
          uint16_t used = parse_one_param(&buf[pos], (uint16_t)(len - pos),
                                          &f->params[f->param_cnt]);
          if (!used)
            return 0;
          pos += used;
          f->param_cnt++;
        }
      }
    }
  } else {
    return 0;
  }

  // CRC16/MODBUS（校验区间：从 SensorID 到 Data 末尾）
  if (len - pos < 2)
    return 0;
  uint16_t crc_rx = (uint16_t)(buf[pos] | (buf[pos + 1] << 8));
  uint16_t crc_calc = crc16_modbus(buf, pos);
  if (crc_rx != crc_calc)
    return 0;

  f->raw_len = (uint16_t)(pos + 2);
  if (f->raw_len <= sizeof(f->raw))
    memcpy(f->raw, buf, f->raw_len);
  return f->raw_len;
}

// ---- 流式喂入：从 DMA/IDLE 拉到的数据块交给解析器 ----
void proto_feed(proto_ctx_t *ctx, const uint8_t *data, uint16_t len) {
  if (len) {
    uint16_t space = (sizeof(ctx->acc) > ctx->acc_len)
                         ? (sizeof(ctx->acc) - ctx->acc_len)
                         : 0;
    uint16_t cp = (len < space) ? len : space;
    memcpy(&ctx->acc[ctx->acc_len], data, cp);
    ctx->acc_len += cp;
  }
  while (ctx->acc_len >= 10) {
    proto_frame_t f;
    uint16_t used = try_parse_one(ctx, ctx->acc, ctx->acc_len, &f);
    if (!used) {
      memmove(ctx->acc, &ctx->acc[1], ctx->acc_len - 1);
      ctx->acc_len -= 1;
      continue;
    }
    if (!f.dropped_by_id) {
      switch (f.hdr.pkt_type) {
      case PKT_MONITOR_DATA:
        if (ctx->on_monitor)
          ctx->on_monitor(ctx, &f);
        break;
      case PKT_MONITOR_ACK:
        if (ctx->on_monitor_ack)
          ctx->on_monitor_ack(ctx, &f);
        break;
      case PKT_CONTROL:
        if (ctx->on_control)
          ctx->on_control(ctx, &f);
        break;
      case PKT_CONTROL_ACK:
        if (ctx->on_control_ack)
          ctx->on_control_ack(ctx, &f);
        break;
      default:
        break;
      }
    }
    memmove(ctx->acc, &ctx->acc[used], ctx->acc_len - used);
    ctx->acc_len -= used;
  }
}
