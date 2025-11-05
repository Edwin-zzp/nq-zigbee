// =============================================================
// 文件：src/crc16.c
// =============================================================
#include "crc16.h"

uint16_t crc16_modbus(const uint8_t *data, size_t len){
    uint16_t crc = 0xFFFF;
    for (size_t i=0;i<len;i++){
        crc ^= data[i];
        for (int b=0;b<8;b++){
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc = (crc >> 1);
        }
    }
    return crc; // 注意：发送/接收时“低字节在前”
}

