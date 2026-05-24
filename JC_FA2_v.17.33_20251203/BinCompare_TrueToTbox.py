#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_bin_tbox_send.py — 从 PCAN-View .trc 文件提取 TBOX 下发的固件数据，与 app1.bin 对比误码率

用法: 直接运行，会读取 true_data.trc 并对比 app1.bin
"""

import os
import re
import sys
import struct

# ========== 路径配置 ==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TRC_FILE = os.path.join(SCRIPT_DIR, "true_data.trc")
APP1_BIN_PATH = r"F:\Enterprise WeChat\WXWork\1688858205719851\Cache\File\2026-04\app1.bin"
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "tbox_extracted_firmware.txt")

# ========== CAN / ISO-TP 常量 ==========
TBOX_CAN_ID = 0x18DA03F1       # TBOX → ECU
UDS_SID_TRANSFER_DATA = 0x36   # 固件数据传输

# ========== ISO-TP PCI 帧类型 ==========
ISOTP_SINGLE = 0x00
ISOTP_FIRST  = 0x10
ISOTP_CONSEC = 0x20
ISOTP_FC     = 0x30


def parse_trc_frames(filepath):
    """解析 PCAN-View .trc 文件，返回 CAN 帧列表 [(can_id, data_bytes), ...]"""
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return None

    frames = []
    # .trc 数据行格式: "     N)     TimeOffset  Rx/Tr  CAN_ID  DLC  b0 b1 b2 b3 b4 b5 b6 b7"
    pattern = re.compile(
        r'^\s*\d+\)\s+[\d.]+\s+Rx\s+([0-9A-Fa-f]{8})\s+(\d+)\s+'
        r'([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+'
        r'([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})'
    )

    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            m = pattern.match(line)
            if not m:
                continue
            can_id = int(m.group(1), 16)
            dlc = int(m.group(2))
            data = bytes(int(m.group(i), 16) for i in range(3, 3 + dlc))
            frames.append((can_id, data))

    return frames


def reassemble_isotp(frames):
    """从 CAN 帧中重组 ISO-TP 完整消息，返回 [(can_id, payload_bytes), ...]"""
    messages = []
    # ISO-TP 状态
    rx_state = "IDLE"
    rx_total_len = 0
    rx_buffer = bytearray()
    rx_received = 0

    for can_id, data in frames:
        if can_id != TBOX_CAN_ID:
            continue
        if len(data) < 1:
            continue

        pci = data[0]
        frame_type = pci & 0xF0

        if frame_type == ISOTP_SINGLE:
            length = pci & 0x0F
            payload = data[1:1 + length]
            messages.append((can_id, bytes(payload)))

        elif frame_type == ISOTP_FIRST:
            length_hi = pci & 0x0F
            length_lo = data[1]
            total_len = (length_hi << 8) | length_lo
            payload_start = data[2:]
            first_data_len = min(6, total_len)

            rx_state = "ACTIVE"
            rx_total_len = total_len
            rx_buffer = bytearray(payload_start[:first_data_len])
            rx_received = first_data_len

            if rx_received >= rx_total_len:
                messages.append((can_id, bytes(rx_buffer)))
                rx_state = "IDLE"

        elif frame_type == ISOTP_CONSEC:
            if rx_state != "ACTIVE":
                continue
            remaining = rx_total_len - rx_received
            cf_data = data[1:]
            take = min(len(cf_data), remaining)
            rx_buffer.extend(cf_data[:take])
            rx_received += take

            if rx_received >= rx_total_len:
                messages.append((can_id, bytes(rx_buffer)))
                rx_state = "IDLE"

    return messages


def extract_transfer_data(messages):
    """从 UDS 消息中提取 TransferData (0x36) 数据块，返回 {block_seq: data_bytes}"""
    blocks = {}

    for can_id, payload in messages:
        if len(payload) < 2:
            continue
        sid = payload[0]
        if sid != UDS_SID_TRANSFER_DATA:
            continue
        block_seq = payload[1]
        data = payload[2:]
        if block_seq in blocks:
            print(f"[WARN] 重复的 block_seq={block_seq}，将被覆盖")
        blocks[block_seq] = bytes(data)
        print(f"[INFO] TransferData block_seq={block_seq}, data_len={len(data)}")

    return blocks


def assemble_firmware(blocks):
    """按 block_seq 排序并拼接固件数据"""
    if not blocks:
        return None
    result = bytearray()
    for seq in sorted(blocks.keys()):
        result.extend(blocks[seq])
    return bytes(result)


def save_hex_dump(data, filepath):
    """保存 hex dump 文件 (readbin.py 格式)"""
    bytes_per_line = 16
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("偏移地址(Hex)  Hex内容                          ASCII\n")
        f.write("-" * 62 + "\n")
        for i in range(0, len(data), bytes_per_line):
            chunk = data[i:i + bytes_per_line]
            offset = f"{i:08X}"
            hex_part = ' '.join(f'{b:02X}' for b in chunk)
            hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
            ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
            f.write(f"{offset}  {hex_part}  {ascii_part}\n")


def read_bin(filepath):
    """读取二进制文件"""
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return None
    with open(filepath, 'rb') as f:
        return f.read()


def count_bit_errors(byte1, byte2):
    """计算两个字节之间的 bit 差异数"""
    xor = byte1 ^ byte2
    return bin(xor).count('1')


def compare_data(extracted, reference, ref_offset=0):
    """对比 extracted 和 reference[ref_offset:] 的误码率"""
    compare_len = min(len(extracted), len(reference) - ref_offset)
    if compare_len <= 0:
        return None

    total_bits = compare_len * 8
    error_bits = 0
    error_bytes = 0

    for i in range(compare_len):
        eb = extracted[i]
        rb = reference[ref_offset + i]
        if eb != rb:
            error_bits += count_bit_errors(eb, rb)
            error_bytes += 1

    bit_error_rate = (error_bits / total_bits) * 100 if total_bits > 0 else 0
    byte_error_rate = (error_bytes / compare_len) * 100 if compare_len > 0 else 0

    return {
        'compare_len': compare_len,
        'total_bits': total_bits,
        'error_bits': error_bits,
        'error_bytes': error_bytes,
        'bit_error_rate': bit_error_rate,
        'byte_error_rate': byte_error_rate,
        'ref_offset': ref_offset,
    }


def find_best_offset(extracted, reference, search_start=0, search_range=0x10000):
    """在 reference 中搜索最佳匹配偏移"""
    signature = extracted[:min(32, len(extracted))]
    best_offset = search_start
    best_errors = len(signature) * 8

    start = max(0, search_start - 0x1000)
    end = min(len(reference) - len(signature), search_start + search_range)

    for offset in range(start, end):
        errors = sum(count_bit_errors(signature[i], reference[offset + i])
                     for i in range(len(signature)))
        if errors < best_errors:
            best_errors = errors
            best_offset = offset
            if errors == 0:
                break

    return best_offset


def print_hex_diff(extracted, reference, ref_offset):
    """打印差异对比 — 按16字节对齐行显示，差异字节标为 XX[YY]"""
    compare_len = min(len(extracted), len(reference) - ref_offset)
    diff_count = 0

    rows_with_diff = set()
    for i in range(compare_len):
        if extracted[i] != reference[ref_offset + i]:
            diff_count += 1
            rows_with_diff.add(i >> 4)

    print("\n[DIFF] 差异详情 (全部):")
    print("-" * 78)
    print("  格式: XX[YY] = CAN发送数据[app1.bin参考数据]")
    print()

    for row in sorted(rows_with_diff):
        row_start = row << 4
        row_end = min(compare_len, row_start + 16)
        parts = []
        for j in range(row_start, row_end):
            if extracted[j] != reference[ref_offset + j]:
                parts.append(f"{extracted[j]:02X}[{reference[ref_offset + j]:02X}]")
            else:
                parts.append(f"{extracted[j]:02X}")
        line = ' '.join(parts)
        print(f"  {row_start:08X}: {line}")

    if diff_count == 0:
        print("  (无差异 — 完全匹配)")
    else:
        print(f"\n  --- 共 {diff_count} 处差异 ---")


def safe_input(prompt=""):
    """安全 input，非交互模式下不报错"""
    try:
        return input(prompt)
    except EOFError:
        return ""


def main():
    print("=" * 62)
    print("  TBOX 下发固件对比工具 - compare_bin_tbox_send.py")
    print("=" * 62)
    print()

    # 1. 解析 .trc 文件
    print(f"[INFO] 解析 TRC 文件: {TRC_FILE}")
    frames = parse_trc_frames(TRC_FILE)
    if frames is None:
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      共解析 {len(frames)} 帧 (全部 CAN ID)")
    tbox_frames = [f for f in frames if f[0] == TBOX_CAN_ID]
    print(f"      TBOX (0x{TBOX_CAN_ID:08X}) 帧数: {len(tbox_frames)}")
    print()

    # 2. ISO-TP 重组
    print("[INFO] ISO-TP 重组...")
    messages = reassemble_isotp(frames)
    print(f"      重组完成，共 {len(messages)} 条完整 UDS 消息")
    print()

    # 3. 提取 TransferData
    print("[INFO] 提取 TransferData (0x36) 数据块...")
    blocks = extract_transfer_data(messages)
    print(f"      提取到 {len(blocks)} 个数据块")
    print()

    # 4. 拼接固件
    print("[INFO] 拼接固件数据...")
    firmware = assemble_firmware(blocks)
    if firmware is None or len(firmware) == 0:
        print("[ERROR] 未提取到任何固件数据!")
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      固件大小: {len(firmware)} bytes ({len(firmware)/1024:.2f} KB)")
    print()

    # 5. 保存 hex dump
    save_hex_dump(firmware, OUTPUT_FILE)
    print(f"[INFO] 固件 hex dump 已保存: {OUTPUT_FILE}")
    print()

    # 6. 加载参考固件
    print("[INFO] 加载参考固件...")
    reference = read_bin(APP1_BIN_PATH)
    if reference is None:
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      文件: {APP1_BIN_PATH}")
    print(f"      大小: {len(reference)} bytes ({len(reference)/1024:.2f} KB)")
    print()

    # 7. 搜索最佳对齐偏移
    print("[INFO] 在参考文件中搜索最佳对齐偏移...")
    best_offset = find_best_offset(firmware, reference, 0)
    print(f"      最佳偏移: 0x{best_offset:08X} ({best_offset} bytes)")
    print()

    # 8. 计算误码率
    print("[INFO] 计算误码率...")
    result = compare_data(firmware, reference, best_offset)
    if result is None:
        print("[ERROR] 无法对比：数据范围不重叠")
        safe_input("\n按 Enter 键退出...")
        return

    print()
    print("=" * 62)
    print("  对比结果")
    print("=" * 62)
    print(f"  对比长度:       {result['compare_len']} bytes ({result['compare_len']/1024:.2f} KB)")
    print(f"  总 bit 数:      {result['total_bits']}")
    print(f"  错误 bit 数:    {result['error_bits']}")
    print(f"  错误 byte 数:   {result['error_bytes']}")
    print(f"  Bit 误码率:     {result['bit_error_rate']:.6f}%")
    print(f"  Byte 误码率:    {result['byte_error_rate']:.4f}%")
    print(f"  参考偏移:       0x{result['ref_offset']:08X}")
    print()

    # 9. 错误评级
    ber = result['bit_error_rate']
    if ber == 0:
        grade = "完美 - 完全匹配!"
    elif ber < 0.001:
        grade = "优秀 - 极低误码率"
    elif ber < 0.01:
        grade = "良好"
    elif ber < 0.1:
        grade = "一般 - 可能存在问题"
    elif ber < 1.0:
        grade = "较差 - 需要检查传输质量"
    else:
        grade = "很差 - 严重误码!"

    print(f"  评级: {grade}")
    print()

    # 10. 打印差异详情
    if result['error_bytes'] > 0:
        print_hex_diff(firmware, reference, best_offset)

    print()
    print("=" * 62)
    print("  检测完成!")
    print("=" * 62)

    safe_input("\n按 Enter 键退出...")


if __name__ == "__main__":
    main()
