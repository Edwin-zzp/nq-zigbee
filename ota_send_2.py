#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, struct, time, binascii
import serial

SYNC = 0x55AA55AA
CHUNK = 220
ACK_TIMEOUT_S = 2.0
ERASE_TIMEOUT_S = 15.0
LINE_TIMEOUT_S = 5.0
RETRY_MAX = 3

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

def read_line(ser: serial.Serial, timeout_s: float) -> bytes:
    """读到 \n 为止；兼容 '\r\n' 与尾随 0。"""
    end = time.time() + timeout_s
    buf = bytearray()
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if b'\n' in buf:
                line, _, _ = buf.partition(b'\n')
                return line.rstrip(b'\r\0')
        else:
            time.sleep(0.01)
    return b''

def wait_for_token(ser: serial.Serial, token: bytes, timeout_s: float, label: str) -> bool:
    """等到包含 token 的一行（大小写不敏感），返回 True/False 并打印行内容"""
    line = read_line(ser, timeout_s)
    print(f"[BOOT] {line!r}")
    return token.lower() in line.lower()

def open_serial(port: str, baud=115200) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,            # 读超时（短）
        write_timeout=2.0,       # 写超时
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    # 重要：关闭 DTR/RTS，清空缓冲，给设备稳定时间
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
    return ser

def send_file(ser: serial.Serial, bin_data: bytes) -> bool:
    size = len(bin_data)
    crc32 = binascii.crc32(bin_data) & 0xFFFFFFFF
    print(f"[INFO] BIN size={size}, crc32=0x{crc32:08X}")

    # 1) 发送 HDR
    hdr = struct.pack("<III", SYNC, size, crc32)
    ser.write(hdr); ser.flush()

    # 2) 等 "HDR" 回执
    if not wait_for_token(ser, b"HDR", LINE_TIMEOUT_S, "HDR"):
        print("[ERR ] Boot未接受HDR")
        return False

    # 3) 等 "ERASED"
    if not wait_for_token(ser, b"ERASED", ERASE_TIMEOUT_S, "ERASED"):
        print("[ERR ] 未收到 ERASED（擦除未完成或回执格式不匹配）")
        return False

    # 3.1) 清空输入缓冲 + 小延时，避免 ERASED 与首包 ACK 混淆
    ser.reset_input_buffer()
    time.sleep(0.02)
    print("[INFO] 开始发送分包数据")
    # 4) 分片发送
    offset = 0
    first_packet = True
    t_start = time.time()

    while offset < size:
        chunk = bin_data[offset : offset + CHUNK]
        ph    = struct.pack("<IH", offset, len(chunk))
        c16   = struct.pack("<H", crc16_ccitt(chunk) & 0xFFFF)

        # 一次性写入一个 buffer，避免分多次 write 被串口驱动碎片化
        frame = ph + chunk + c16
        ser.write(frame)
        ser.flush()

        # 首包的 ACK 超时更宽松（例如 3.0s；后续包 2.0s）
        ack_to = 3.0 if first_packet else 2.0
        first_packet = False

        ok = False
        for attempt in range(1, RETRY_MAX+1):
            line = read_line(ser, ack_to)
            if line:
                print(f"[BOOT] {line!r}")
            low = line.lower()
            if low.endswith(b"ack") or low == b"ack":
                ok = True
                break
            if low.endswith(b"timeout") or low == b"timeout":
                print("[WARN] Boot回TIMEOUT，重发本片")
            else:
                print(f"[WARN] 未收到ACK，重试 {attempt}")
            # 重发同一帧
            ser.write(frame)
            ser.flush()

        if not ok:
            print("[ERR ] 未收到ACK，放弃")
            return False

        offset += len(chunk)
        print(f"[PROG] {100.0*offset/size:.1f}%")

    # 5) 等最终 OK
    line = read_line(ser, 5.0)
    print(f"[BOOT] {line!r}")
    if b"OK" not in line.upper():
        print("[ERR ] 末尾未收到OK/OKBOOT")
        return False

    print(f"[DONE] 成功! 用时 {time.time()-t_start:.2f}s")
    return True

def main():
    if len(sys.argv) < 3:
        print(f"用法: {sys.argv[0]} COMx app.bin")
        sys.exit(1)
    port, path = sys.argv[1], sys.argv[2]
    with open(path, "rb") as f:
        data = f.read()

    ser = open_serial(port, 115200)
    try:
        ok = send_file(ser, data)
        sys.exit(0 if ok else 2)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
