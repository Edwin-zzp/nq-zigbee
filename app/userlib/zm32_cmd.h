#ifndef ZM32_CMD_H
#define ZM32_CMD_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Special MAC designators defined by ZM32 temporary command set */
extern const uint8_t ZM32_MAC_LOCAL_EXEC[8];
extern const uint8_t ZM32_MAC_COORDINATOR[8];

/* Command timing helpers */
#define ZM32_CMD_DEFAULT_TIMEOUT_MS 200u
#define ZM32_CMD_IDLE_GAP_MS 3u

/* Status helpers */
#define ZM32_STATUS_SUCCESS 0x00u

/* Send-mode bit definitions (Table 14.12) */
#define ZM32_SEND_MODE_TX_MASK 0x07u
#define ZM32_SEND_MODE_TX_UNICAST 0x00u
#define ZM32_SEND_MODE_TX_BROADCAST_ALL 0x01u
#define ZM32_SEND_MODE_TX_BROADCAST_NON_SLEEP 0x02u
#define ZM32_SEND_MODE_TX_BROADCAST_COORD_ROUTER 0x03u
#define ZM32_SEND_MODE_TX_MULTICAST 0x04u
#define ZM32_SEND_MODE_SRC_MAC (1u << 3)
#define ZM32_SEND_MODE_DST_USE_MAC (1u << 4)
#define ZM32_SEND_MODE_FMT_SHIFT 5u
#define ZM32_SEND_MODE_FMT_MASK (0x03u << ZM32_SEND_MODE_FMT_SHIFT)
#define ZM32_SEND_MODE_FMT_DATA (0u << ZM32_SEND_MODE_FMT_SHIFT)
#define ZM32_SEND_MODE_FMT_TMP_NET (1u << ZM32_SEND_MODE_FMT_SHIFT)
#define ZM32_SEND_MODE_FMT_TMP_MAC (2u << ZM32_SEND_MODE_FMT_SHIFT)
#define ZM32_SEND_MODE_FMT_FRAME (3u << ZM32_SEND_MODE_FMT_SHIFT)

/* Receive-mode bit definitions (Table 14.30/14.200) */
#define ZM32_RECV_MODE_FMT_MASK 0x07u
#define ZM32_RECV_MODE_FMT_DATA 0x00u
#define ZM32_RECV_MODE_FMT_SRC_NET 0x01u
#define ZM32_RECV_MODE_FMT_SRC_MAC 0x02u
#define ZM32_RECV_MODE_FMT_SRC_BOTH 0x03u
#define ZM32_RECV_MODE_FMT_FRAME 0x04u

/* Convenient presets */
#define ZM32_SEND_MODE_DATA_UNICAST                                                \
  (ZM32_SEND_MODE_TX_UNICAST | ZM32_SEND_MODE_FMT_DATA)
#define ZM32_RECV_MODE_DATA ZM32_RECV_MODE_FMT_DATA

void zm32_cmd_init(void);
void zm32_cmd_flush_rx(void);

typedef struct {
  uint8_t mac[8];
  uint16_t pan_id;
  uint16_t net_addr;
  uint8_t channel;
  uint8_t recv_mode;
  uint8_t power_level;
} zm32_dev_info_t;

bool zm32_cmd_temp_exchange(uint8_t cmd, const uint8_t *payload,
                            uint16_t payload_len, uint8_t *resp_buf,
                            uint16_t *resp_len, uint16_t min_resp_len,
                            bool resp_len_exact, uint32_t timeout_ms);
bool zm32_cmd_perm_exchange(uint8_t cmd, const uint8_t *payload,
                            uint16_t payload_len, uint8_t *resp_buf,
                            uint16_t *resp_len, uint16_t min_resp_len,
                            bool resp_len_exact, uint32_t timeout_ms);

bool zm32_cmd_set_send_mode(uint8_t mode, uint8_t *status_out);
bool zm32_cmd_set_target_network(uint16_t net_addr, uint8_t *status_out);
bool zm32_cmd_set_target_mac(const uint8_t mac[8], uint8_t *status_out,
                             uint8_t ack_mac_out[8]);
bool zm32_cmd_set_receive_mode(const uint8_t mac[8], uint8_t mode,
                               uint8_t *status_out, uint8_t ack_mac_out[8]);
bool zm32_cmd_read_local_info(zm32_dev_info_t *info);
bool zm32_cmd_write_pan_id(const uint8_t mac[8], uint16_t pan_id,
                           uint8_t *status_out);
bool zm32_cmd_reset_module(const uint8_t mac[8]);

/* Frame transport helpers (Table 14.213/14.215/14.217/14.219) */
#define ZM32_FRAME_HEADER0 0xADu
#define ZM32_FRAME_HEADER1 0xDBu
#define ZM32_FRAME_HEADER2 0xBEu
#define ZM32_FRAME_TAIL 0xAAu

typedef enum {
  ZM32_FRAME_TX_UNICAST = 0x10u,
  ZM32_FRAME_TX_MULTICAST = 0x11u,
} zm32_frame_tx_type_t;

typedef enum {
  ZM32_FRAME_RX_UNICAST = 0x90u,
  ZM32_FRAME_RX_MULTICAST = 0x91u,
} zm32_frame_rx_type_t;

#define ZM32_FRAME_TX_MAX_PAYLOAD 255u
#define ZM32_FRAME_TX_MAX_PAYLOAD_MULTICAST 73u

typedef struct {
  zm32_frame_tx_type_t type;
  union {
    struct {
      uint8_t mac[8];
      uint16_t net_addr;
    } unicast;
    struct {
      uint16_t group_id;
    } multicast;
  } target;
  const uint8_t *payload;
  uint16_t payload_len;
} zm32_frame_tx_t;

typedef struct {
  zm32_frame_rx_type_t type;
  uint8_t src_mac[8];
  uint16_t src_net;
  uint16_t group_id;
  const uint8_t *payload;
  uint16_t payload_len;
} zm32_frame_rx_t;

bool zm32_frame_build(const zm32_frame_tx_t *tx, uint8_t *out,
                      uint16_t *inout_len);
bool zm32_frame_send(const zm32_frame_tx_t *tx);
bool zm32_frame_parse(const uint8_t *frame, uint16_t frame_len,
                      zm32_frame_rx_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZM32_CMD_H */
