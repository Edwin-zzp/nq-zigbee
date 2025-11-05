#include "zm32_cmd.h"
#include "debug.h"
#include "usart2_dma.h"

#include <stdio.h>
#include <string.h>

const uint8_t ZM32_MAC_LOCAL_EXEC[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
const uint8_t ZM32_MAC_COORDINATOR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ZM32_PROTOCOL_TYPE 0x0004u
#define ZM32_MAX_FRAME 256u
#define ZM32_FRAME_MAX_TOTAL                                                   \
  (3u + 1u + 8u + 2u + 2u + ZM32_FRAME_TX_MAX_PAYLOAD + 1u + 1u)
#define ZM32_FRAME_MAX_TOTAL_MULTICAST                                          \
  (3u + 1u + 2u + 2u + ZM32_FRAME_TX_MAX_PAYLOAD_MULTICAST + 1u + 1u)

// 计算 ZM32 帧的逐字节累加校验值
static uint8_t zm32_frame_checksum(const uint8_t *buf, uint16_t len) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < len; ++i)
    sum += buf[i];
  return (uint8_t)(sum & 0xFFu);
}

// 描述不同命令协议头格式及结束方式的表项
typedef struct {
  uint8_t header[3];
  uint8_t header_len;
  uint8_t min_len;
  uint8_t idle_terminate_ms;
  uint8_t tail;
  bool has_tail;
} zm32_proto_desc_t;

static const zm32_proto_desc_t kProtoTemp = {
    {0xDEu, 0xDFu, 0xEFu}, 3u, 4u, ZM32_CMD_IDLE_GAP_MS, 0u, false};

static const zm32_proto_desc_t kProtoPerm = {
    {0xABu, 0xBCu, 0xCDu}, 3u, 5u, ZM32_CMD_IDLE_GAP_MS, 0xAAu, true};

static bool zm32_frame_validate_header(const uint8_t *frame, uint16_t frame_len) {
  if (!frame || frame_len < 6u)
    return false;
  return frame[0] == ZM32_FRAME_HEADER0 && frame[1] == ZM32_FRAME_HEADER1 &&
         frame[2] == ZM32_FRAME_HEADER2;
}

// 发送临时/永久配置命令帧，payload 根据描述符自动拼接
static void zm32_send_frame(const zm32_proto_desc_t *proto, uint8_t cmd,
                            const uint8_t *payload, uint16_t payload_len) {
  if (!proto)
    return;

  uint8_t frame[ZM32_MAX_FRAME];
  uint16_t pos = 0;

  for (uint8_t i = 0; i < proto->header_len && pos < ZM32_MAX_FRAME; ++i)
    frame[pos++] = proto->header[i];

  if (pos < ZM32_MAX_FRAME)
    frame[pos++] = cmd;

  if (payload_len && payload && pos < ZM32_MAX_FRAME) {
    uint16_t available = ZM32_MAX_FRAME - pos - (proto->has_tail ? 1u : 0u);
    if (payload_len > available)
      payload_len = available;
    memcpy(&frame[pos], payload, payload_len);
    pos = (uint16_t)(pos + payload_len);
  }

  if (proto->has_tail && pos < ZM32_MAX_FRAME)
    frame[pos++] = proto->tail;

  if (pos)
    usart2_tx_write_blocking(frame, pos);
}

// 阻塞等待响应帧，支持按协议头对齐及空闲间隔判定帧结束
static bool zm32_wait_frame(const zm32_proto_desc_t *proto, uint8_t cmd,
                            uint8_t *out, uint16_t *io_len, uint16_t min_len,
                            bool min_is_exact, uint32_t timeout_ms) {
  if (!proto || !out || !io_len)
    return false;

  uint16_t capacity = *io_len;
  if (capacity < proto->min_len)
    return false;

  if (min_len < proto->min_len)
    min_len = proto->min_len;
  if (capacity < min_len)
    return false;

  uint16_t pos = 0;
  uint8_t state = 0;
  bool capturing = false;
  uint32_t waited = 0;
  uint32_t idle = 0;

  while (waited <= timeout_ms) {
    uint8_t chunk[32];
    uint16_t got = usart2_pull_chunk(chunk, sizeof(chunk));
    if (got == 0) {
      Delay_Ms(1);
      waited++;
      if (capturing && !proto->has_tail && !min_is_exact && pos >= min_len) {
        idle++;
        if (idle >= proto->idle_terminate_ms) {
          *io_len = pos;
          return true;
        }
      }
      continue;
    }

    idle = 0;
    for (uint16_t i = 0; i < got; ++i) {
      uint8_t b = chunk[i];

      if (!capturing) {
        if (state < proto->header_len) {
          if (b == proto->header[state]) {
            if (pos >= capacity)
              return false;
            out[pos++] = b;
            state++;
          } else {
            if (b == proto->header[0]) {
              pos = 0;
              state = 1;
              out[pos++] = b;
            } else {
              pos = 0;
              state = 0;
            }
          }
          continue;
        }

        if (b == cmd) {
          if (pos >= capacity)
            return false;
          out[pos++] = b;
          capturing = true;
          state++;
          if (!proto->has_tail && min_is_exact && pos >= min_len) {
            *io_len = pos;
            return true;
          }
          continue;
        }

        if (b == proto->header[0]) {
          pos = 0;
          state = 1;
          out[pos++] = b;
        } else {
          pos = 0;
          state = 0;
        }
        continue;
      }

      if (pos >= capacity)
        return false;
      out[pos++] = b;

      if (proto->has_tail) {
        if (b == proto->tail) {
          *io_len = pos;
          return pos >= min_len;
        }
      } else if (min_is_exact && pos >= min_len) {
        *io_len = pos;
        return true;
      }
    }
  }

  return false;
}

void zm32_cmd_flush_rx(void) {
  uint8_t buf[64];
  uint8_t idle = 0;
  while (idle < 2) {
    uint16_t got = usart2_pull_chunk(buf, sizeof(buf));
    if (got == 0) {
      idle++;
      Delay_Ms(1);
    } else {
      idle = 0;
    }
  }
}

void zm32_cmd_init(void) { zm32_cmd_flush_rx(); }

bool zm32_cmd_temp_exchange(uint8_t cmd, const uint8_t *payload,
                            uint16_t payload_len, uint8_t *resp_buf,
                            uint16_t *resp_len, uint16_t min_resp_len,
                            bool resp_len_exact, uint32_t timeout_ms) {
  zm32_cmd_flush_rx();
  zm32_send_frame(&kProtoTemp, cmd, payload, payload_len);

  if (!resp_buf || !resp_len || *resp_len == 0)
    return true;

  uint16_t len = *resp_len;
  if (!zm32_wait_frame(&kProtoTemp, cmd, resp_buf, &len, min_resp_len,
                       resp_len_exact, timeout_ms))
    return false;

  *resp_len = len;
  return true;
}

bool zm32_cmd_perm_exchange(uint8_t cmd, const uint8_t *payload,
                            uint16_t payload_len, uint8_t *resp_buf,
                            uint16_t *resp_len, uint16_t min_resp_len,
                            bool resp_len_exact, uint32_t timeout_ms) {
  zm32_cmd_flush_rx();
  zm32_send_frame(&kProtoPerm, cmd, payload, payload_len);

  if (!resp_buf || !resp_len || *resp_len == 0)
    return true;

  uint16_t len = *resp_len;
  if (!zm32_wait_frame(&kProtoPerm, cmd, resp_buf, &len, min_resp_len,
                       resp_len_exact, timeout_ms))
    return false;

  *resp_len = len;
  return true;
}

bool zm32_cmd_set_send_mode(uint8_t mode, uint8_t *status_out) {
  uint8_t payload = mode;
  uint8_t resp[8];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_temp_exchange(0xD9u, &payload, sizeof(payload), resp, &resp_len,
                              5u, true, ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] set_send_mode timeout\r\n");
    return false;
  }
  if (resp_len < 5u)
    return false;
  if (status_out)
    *status_out = resp[4];
  return true;
}

bool zm32_cmd_set_target_network(uint16_t net_addr, uint8_t *status_out) {
  uint8_t payload[2];
  payload[0] = (uint8_t)(net_addr & 0xFFu);
  payload[1] = (uint8_t)((net_addr >> 8) & 0xFFu);
  uint8_t resp[8];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_temp_exchange(0xD2u, payload, sizeof(payload), resp, &resp_len,
                              5u, true, ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] set_target_network timeout\r\n");
    return false;
  }
  if (resp_len < 5u)
    return false;
  if (status_out)
    *status_out = resp[4];
  return true;
}

bool zm32_cmd_set_target_mac(const uint8_t mac[8], uint8_t *status_out,
                             uint8_t ack_mac_out[8]) {
  if (!mac)
    return false;
  uint8_t resp[16];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_temp_exchange(0x74u, mac, 8u, resp, &resp_len, 13u, true,
                              ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] set_target_mac timeout\r\n");
    return false;
  }
  if (resp_len < 13u)
    return false;
  if (ack_mac_out)
    memcpy(ack_mac_out, &resp[4], 8);
  if (status_out)
    *status_out = resp[12];
  return true;
}

bool zm32_cmd_set_receive_mode(const uint8_t mac[8], uint8_t mode,
                               uint8_t *status_out, uint8_t ack_mac_out[8]) {
  if (!mac)
    return false;
  uint8_t payload[9];
  memcpy(payload, mac, 8);
  payload[8] = mode;
  uint8_t resp[16];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_temp_exchange(0x73u, payload, sizeof(payload), resp, &resp_len,
                              13u, true, ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] set_receive_mode timeout\r\n");
    return false;
  }
  if (resp_len < 13u)
    return false;
  if (ack_mac_out)
    memcpy(ack_mac_out, &resp[4], 8);
  if (status_out)
    *status_out = resp[12];
  return true;
}

static bool zm32_parse_dev_info(const uint8_t *dev_info, uint16_t len,
                                zm32_dev_info_t *out) {
  if (!dev_info || len < 68u || !out)
    return false;

  memset(out, 0, sizeof(*out));
  out->channel = dev_info[33];
  out->pan_id = (uint16_t)((dev_info[34] << 8) | dev_info[35]);
  out->net_addr = (uint16_t)((dev_info[36] << 8) | dev_info[37]);
  memcpy(out->mac, &dev_info[38], 8u);
  out->recv_mode = dev_info[56];
  out->power_level = dev_info[57];
  return true;
}

bool zm32_cmd_read_local_info(zm32_dev_info_t *info) {
  if (!info)
    return false;

  uint8_t resp[96];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_perm_exchange(0xD1u, NULL, 0, resp, &resp_len, 78u, false,
                              ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] read_local_info timeout\r\n");
    return false;
  }
  if (resp_len < 78u)
    return false;
  if (resp[0] != 0xABu || resp[1] != 0xBCu || resp[2] != 0xCDu)
    return false;
  if (resp[3] != 0xD1u)
    return false;
  if (resp[resp_len - 1] != 0xAAu)
    return false;

  const uint8_t *dev_info = &resp[4];
  return zm32_parse_dev_info(dev_info, 68u, info);
}

bool zm32_cmd_write_pan_id(const uint8_t mac[8], uint16_t pan_id,
                           uint8_t *status_out) {
  if (!mac)
    return false;

  uint8_t payload[13];
  memcpy(payload, mac, 8u);
  payload[8] = 0x01u; // write
  payload[9] = (uint8_t)((pan_id >> 8) & 0xFFu);
  payload[10] = (uint8_t)(pan_id & 0xFFu);
  payload[11] = 0x00u;
  payload[12] = 0x00u;

  uint8_t resp[20];
  uint16_t resp_len = sizeof(resp);
  if (!zm32_cmd_perm_exchange(0x41u, payload, sizeof(payload), resp, &resp_len,
                              18u, true, ZM32_CMD_DEFAULT_TIMEOUT_MS)) {
    printf("[ZM32] write_pan_id timeout\r\n");
    return false;
  }
  if (resp_len < 18u)
    return false;
  if (status_out)
    *status_out = resp[16];
  return true;
}

bool zm32_cmd_reset_module(const uint8_t mac[8]) {
  if (!mac)
    return false;

  uint8_t payload[10];
  memcpy(payload, mac, 8u);
  payload[8] = (uint8_t)((ZM32_PROTOCOL_TYPE >> 8) & 0xFFu);
  payload[9] = (uint8_t)(ZM32_PROTOCOL_TYPE & 0xFFu);

  return zm32_cmd_perm_exchange(0x59u, payload, sizeof(payload), NULL, NULL,
                                0u, false, ZM32_CMD_DEFAULT_TIMEOUT_MS);
}

bool zm32_frame_build(const zm32_frame_tx_t *tx, uint8_t *out,
                      uint16_t *inout_len) {
  if (!tx || !out || !inout_len)
    return false;

  uint16_t capacity = *inout_len;
  uint16_t pos = 0;
  uint16_t payload_len = tx->payload_len;

  if (tx->payload_len && !tx->payload)
    return false;

  switch (tx->type) {
  case ZM32_FRAME_TX_UNICAST: {
    if (payload_len > ZM32_FRAME_TX_MAX_PAYLOAD)
      return false;
    uint16_t required = (uint16_t)(3u + 1u + 8u + 2u + 2u + payload_len + 1u + 1u);
    if (capacity < required)
      return false;

    out[pos++] = ZM32_FRAME_HEADER0;
    out[pos++] = ZM32_FRAME_HEADER1;
    out[pos++] = ZM32_FRAME_HEADER2;
    out[pos++] = (uint8_t)ZM32_FRAME_TX_UNICAST;

    memcpy(&out[pos], tx->target.unicast.mac, 8u);
    pos = (uint16_t)(pos + 8u);

    out[pos++] = (uint8_t)((tx->target.unicast.net_addr >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(tx->target.unicast.net_addr & 0xFFu);

    out[pos++] = (uint8_t)((payload_len >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(payload_len & 0xFFu);

    if (payload_len) {
      memcpy(&out[pos], tx->payload, payload_len);
      pos = (uint16_t)(pos + payload_len);
    }

    out[pos++] = zm32_frame_checksum(out, pos);
    out[pos++] = ZM32_FRAME_TAIL;
    break;
  }
  case ZM32_FRAME_TX_MULTICAST: {
    if (payload_len > ZM32_FRAME_TX_MAX_PAYLOAD_MULTICAST)
      return false;
    uint16_t required =
        (uint16_t)(3u + 1u + 2u + 2u + payload_len + 1u + 1u);
    if (capacity < required)
      return false;

    out[pos++] = ZM32_FRAME_HEADER0;
    out[pos++] = ZM32_FRAME_HEADER1;
    out[pos++] = ZM32_FRAME_HEADER2;
    out[pos++] = (uint8_t)ZM32_FRAME_TX_MULTICAST;

    out[pos++] = (uint8_t)((tx->target.multicast.group_id >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(tx->target.multicast.group_id & 0xFFu);

    out[pos++] = (uint8_t)((payload_len >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(payload_len & 0xFFu);

    if (payload_len) {
      memcpy(&out[pos], tx->payload, payload_len);
      pos = (uint16_t)(pos + payload_len);
    }

    out[pos++] = zm32_frame_checksum(out, pos);
    out[pos++] = ZM32_FRAME_TAIL;
    break;
  }
  default:
    return false;
  }

  *inout_len = pos;
  return true;
}

bool zm32_frame_send(const zm32_frame_tx_t *tx) {
  if (!tx)
    return false;

  uint8_t buffer[ZM32_FRAME_MAX_TOTAL];
  uint16_t buf_len = sizeof(buffer);

  if (tx->type == ZM32_FRAME_TX_MULTICAST) {
    uint8_t mc_buffer[ZM32_FRAME_MAX_TOTAL_MULTICAST];
    buf_len = sizeof(mc_buffer);
    if (!zm32_frame_build(tx, mc_buffer, &buf_len))
      return false;
    usart2_tx_write_blocking(mc_buffer, buf_len);
    return true;
  }

  if (!zm32_frame_build(tx, buffer, &buf_len))
    return false;
  usart2_tx_write_blocking(buffer, buf_len);
  return true;
}

bool zm32_frame_parse(const uint8_t *frame, uint16_t frame_len,
                      zm32_frame_rx_t *out) {
  if (!frame || frame_len < 6u)
    return false;
  if (!out)
    return false;
  if (!zm32_frame_validate_header(frame, frame_len))
    return false;

  uint8_t flag = frame[3];
  if (frame[frame_len - 1] != ZM32_FRAME_TAIL)
    return false;

  switch (flag) {
  case ZM32_FRAME_RX_UNICAST: {
    if (frame_len < 18u)
      return false;
    uint16_t payload_len =
        (uint16_t)((frame[14] << 8) | frame[15]);
    if (payload_len > ZM32_FRAME_TX_MAX_PAYLOAD)
      return false;
    uint16_t expected = (uint16_t)(18u + payload_len);
    if (frame_len != expected)
      return false;

    uint16_t checksum_index = (uint16_t)(16u + payload_len);
    uint8_t checksum = zm32_frame_checksum(frame, checksum_index);
    if (checksum != frame[checksum_index])
      return false;

    out->type = ZM32_FRAME_RX_UNICAST;
    memcpy(out->src_mac, &frame[4], 8u);
    out->src_net = (uint16_t)((frame[12] << 8) | frame[13]);
    out->group_id = 0u;
    out->payload = (payload_len) ? &frame[16] : NULL;
    out->payload_len = payload_len;
    return true;
  }
  case ZM32_FRAME_RX_MULTICAST: {
    if (frame_len < 20u)
      return false;
    uint16_t payload_len =
        (uint16_t)((frame[16] << 8) | frame[17]);
    if (payload_len > ZM32_FRAME_TX_MAX_PAYLOAD_MULTICAST)
      return false;
    uint16_t expected = (uint16_t)(20u + payload_len);
    if (frame_len != expected)
      return false;

    uint16_t checksum_offset = (uint16_t)(18u + payload_len);
    uint8_t checksum = zm32_frame_checksum(frame, checksum_offset);
    if (checksum != frame[checksum_offset])
      return false;

    out->type = ZM32_FRAME_RX_MULTICAST;
    out->group_id = (uint16_t)((frame[4] << 8) | frame[5]);
    memcpy(out->src_mac, &frame[6], 8u);
    out->src_net = (uint16_t)((frame[14] << 8) | frame[15]);
    out->payload = (payload_len) ? &frame[18] : NULL;
    out->payload_len = payload_len;
    return true;
  }
  default:
    return false;
  }
}
