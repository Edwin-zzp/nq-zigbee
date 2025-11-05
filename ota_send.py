#!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# 最小串口OTA发送脚本
# 协议:
#   HDR: u32 SYNC(0x55AA55AA), u32 image_size, u32 image_crc32
#   DATA: [u32 offset][u16 len][payload][u16 crc16]
# Boot 回应:
#   "HDR\r\n" -> 接收到了头
#   每片返回 "ACK\r\n"
#   结束返回 "OK\r\n" 或 "FAIL\r\n"/"CRCFAIL\r\n"
# """

import sys, struct, time, zlib
import serial

SYNC = 0x55AA55AA
CHUNK = 128

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def read_until(s: serial.Serial, expect: bytes, timeout=5.0):
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        if s.in_waiting:
            buf += s.read(s.in_waiting)
            if expect in buf:
                return buf
        time.sleep(0.01)
    return buf

def main():
    if len(sys.argv) < 3:
        print("用法: python ota_send.py <串口号> <app.bin> [波特率115200]")
        print("例如: python ota_send.py COM7 app.bin")
        return

    port = sys.argv[1]
    path = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    with open(path, "rb") as f:
        img = f.read()

    size = len(img)
    crc32 = zlib.crc32(img) & 0xFFFFFFFF

    print(f"[INFO] BIN size={size}, crc32=0x{crc32:08X}")

    s = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(0.1)

    # 发 HDR
    hdr = struct.pack("<III", SYNC, size, crc32)
    s.write(hdr)

    resp = read_until(s, b"\r\n", timeout=2.0)
    print(f"[BOOT] {resp!r}")
    if b"NOHDR" in resp or b"BADHDR" in resp or b"HDR" not in resp:
        print("[ERR ] Boot未接受HDR")
        return
    
    # 再等 "ERASED\r\n"（擦除完成信号）
    resp2 = read_until(s, b"\r\n", timeout=10.0)   # 擦除要花点时间，超时给大一点
    print(f"[BOOT] {resp2!r}")
    if b"ERASED" not in resp2:
        print("[ERR ] 未收到 ERASED，Boot 还没准备好接收数据")
        return

    # 清空输入缓冲，避免 ERASED 的残留影响后续 ACK 读取
    s.reset_input_buffer()
    time.sleep(0.05)

    # 分片发送
    offset = 0
    t0 = time.time()
    while offset < size:
        chunk = img[offset:offset+CHUNK]
        pkt_head = struct.pack("<IH", offset, len(chunk))
        c16 = crc16_ccitt(chunk)
        s.write(pkt_head)
        s.write(chunk)
        s.write(struct.pack("<H", c16))

        ack = read_until(s, b"\r\n", timeout=8.0)
        if b"ACK" not in ack:
            print(f"[ERR ] 未收到ACK, resp={ack!r}")
            return

        offset += len(chunk)
        if (offset // CHUNK) % 8 == 0:
            done = 100.0 * offset / size
            print(f"[PROG] {done:.1f}%")

    # 结尾应返回 OK/FAIL
    end = read_until(s, b"\r\n", timeout=3.0)
    print(f"[BOOT] {end!r}")
    if b"OK" in end:
        dt = time.time() - t0
        print(f"[DONE] 成功! 用时 {dt:.2f}s")
    else:
        print("[FAIL] 升级失败")

if __name__ == "__main__":
    main()
