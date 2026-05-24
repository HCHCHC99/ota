#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_fw.py — 从 ota data4.txt 提取固件 bin 数据
支持两种格式:
  1. PRINTF_BIN hex dump (=== BIN DUMP START === ... === BIN DUMP END ===)
  2. ISO-TP OTA 帧日志中的 TransferData CF/FF 数据
"""

import re
import os
import sys

# ========== 路径配置（在此修改）==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE = os.path.join(SCRIPT_DIR, "ota data4.txt")
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "extracted_firmware.txt")
# =========================================


def safe_input(prompt=""):
    """安全 input，非交互模式下不报错"""
    try:
        return input(prompt)
    except EOFError:
        return ""


def extract_print_bin(lines):
    """提取 PRINTF_BIN 格式的 hex dump 数据"""
    in_dump = False
    hex_bytes = []

    for line in lines:
        line = line.strip()
        if "=== BIN DUMP START" in line:
            in_dump = True
            hex_bytes = []
            continue
        if "=== BIN DUMP END" in line:
            in_dump = False
            continue
        if in_dump:
            # 匹配 hex 字节: XX XX XX ...
            parts = line.split()
            if parts and parts[0].startswith("00>"):
                parts = parts[1:]
            for p in parts:
                if re.match(r'^[0-9A-Fa-f]{2}$', p):
                    hex_bytes.append(int(p, 16))

    return bytes(hex_bytes)


def extract_ota_frames(lines):
    """从 OTA 帧日志中提取 TransferData 数据"""
    # 正则匹配 OTA 日志行
    pattern = re.compile(
        r'\[OTA\]\s+seq=\d+\s*,\s*time=\s*[\d.]+\s*s,\s*'
        r'\[RX\]\s+0x18DA03F1,\s*'
        r'([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+'
        r'([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+'
        r'([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})'
    )

    blocks = {}  # block_seq -> bytearray
    current_block_seq = None
    current_block_data = None

    for line in lines:
        m = pattern.search(line)
        if not m:
            continue

        b = [int(m.group(i), 16) for i in range(1, 9)]
        pci = b[0]
        frame_type = pci & 0xF0

        if frame_type == 0x10:  # First Frame
            # FF 结构: [PCI_HI][PCI_LO][SID][block_seq][data...]
            if len(b) >= 4:
                sid = b[2]
                if sid == 0x36:  # TransferData
                    block_seq = b[3]
                    data_bytes = bytes(b[4:])  # 最多4字节数据
                    block_seq = int(block_seq)
                    if block_seq not in blocks:
                        blocks[block_seq] = bytearray()
                    blocks[block_seq].extend(data_bytes)
                    current_block_seq = block_seq

        elif frame_type == 0x20:  # Consecutive Frame
            # CF 结构: [PCI][data x7]
            data_bytes = bytes(b[1:])
            if current_block_seq is not None and current_block_seq in blocks:
                blocks[current_block_seq].extend(data_bytes)

    # 按 block_seq 排序拼接
    result = bytearray()
    for seq in sorted(blocks.keys()):
        result.extend(blocks[seq])

    return bytes(result)


def main():
    print("=" * 60)
    print("  Firmware Binary Extractor - extract_fw.py")
    print("=" * 60)
    print()

    if not os.path.exists(INPUT_FILE):
        print(f"[ERROR] 文件不存在: {INPUT_FILE}")
        safe_input("\n按 Enter 键退出...")
        return

    print(f"[INFO] 读取: {INPUT_FILE}")

    with open(INPUT_FILE, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    print(f"[INFO] 共 {len(lines)} 行")
    print()

    # 方法1: 提取 PRINTF_BIN dump 数据
    bin_data = extract_print_bin(lines)
    method = "PRINTF_BIN BIN DUMP"

    # 方法2: 如果 PRINTF_BIN 没有数据，从 OTA 帧提取
    if len(bin_data) == 0:
        print("[INFO] 未检测到 BIN DUMP 标记，尝试从 OTA 帧提取...")
        bin_data = extract_ota_frames(lines)
        method = "OTA TransferData 帧"

    if len(bin_data) == 0:
        print("[WARN] 未提取到任何 bin 数据！")
        print("[INFO] 请确认日志中包含 TransferData RX 帧或 BIN DUMP 标记")
        safe_input("\n按 Enter 键退出...")
        return

    # 写入格式化的 hex 文本文件 (参照 readbin.py 格式)
    bytes_per_line = 16
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write("偏移地址(Hex)  Hex内容                          ASCII\n")
        f.write("-" * 62 + "\n")
        for i in range(0, len(bin_data), bytes_per_line):
            chunk = bin_data[i:i+bytes_per_line]
            offset = f"{i:08X}"
            hex_part = ' '.join(f'{b:02X}' for b in chunk)
            hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
            ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
            f.write(f"{offset}  {hex_part}  {ascii_part}\n")

    print(f"[INFO] 提取方式: {method}")
    print(f"[INFO] 提取数据: {len(bin_data)} bytes ({len(bin_data)/1024:.2f} KB)")
    print(f"[INFO] 输出文件: {OUTPUT_FILE}")
    print()

    # 打印前 128 字节预览
    preview_len = min(128, len(bin_data))
    print(f"[PREVIEW] 前 {preview_len} 字节:")
    print("偏移地址(Hex)  Hex内容                          ASCII")
    print("-" * 62)
    for i in range(0, preview_len, 16):
        chunk = bin_data[i:i+16]
        offset = f"{i:08X}"
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        hex_part = hex_part.ljust(16 * 3 - 1)
        ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        print(f"{offset}  {hex_part}  {ascii_part}")

    if len(bin_data) > preview_len:
        print(f"  ... (共 {len(bin_data)} bytes)")

    print()
    print("=" * 60)
    print(f"  提取完成! 数据已保存到: {OUTPUT_FILE}")
    print("=" * 60)

    safe_input("\n按 Enter 键退出...")


if __name__ == "__main__":
    main()
