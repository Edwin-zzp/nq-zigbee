// =============================================================
// 文件：inc/crc16.h —— CRC16/MODBUS（低字节在前）
// =============================================================
#ifndef CRC16_H
#define CRC16_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C"{
#endif

// 计算 CRC-16/MODBUS（多项式 0xA001，初值 0xFFFF）
uint16_t crc16_modbus(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
#endif // CRC16_H
