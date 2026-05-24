#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_rec.py — 对比 extracted_firmware.txt 和 app1.bin / app2.bin，检测误码率
"""

import os
import re
import sys

# ========== 路径配置（在此修改）==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXTRACTED_FILE = os.path.join(SCRIPT_DIR, "extracted_firmware.txt")
APP1_BIN_PATH = os.path.join(SCRIPT_DIR, "app1.bin")
APP2_BIN_PATH = os.path.join(SCRIPT_DIR, "app2.bin")
# 选择对比哪个参考固件: "app1" 或 "app2"
REF_BIN = "app1"
# =========================================

def safe_input(prompt=""):
    """安全 input，非交互模式下不报错"""
    try:
        return input(prompt)
    except EOFError:
        return ""


# ========== UDS 下载参数 (从 ota data4.txt 日志提取) ==========
TARGET_FLASH_ADDR = 0x08004000   # RequestDownload 目标地址
FLASH_BASE = 0x08000000          # Flash 基地址


def parse_extracted_hex(filepath):
    """解析 extracted_firmware.txt 格式的 hex 文件"""
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return None

    data = bytearray()
    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.strip()
            # 跳过表头和分隔线
            if not line or line.startswith('偏移') or line.startswith('---'):
                continue
            # 匹配: 00000000  XX XX XX ...  ascii
            m = re.match(r'^[0-9A-Fa-f]{8}\s+([0-9A-Fa-f\s]+)\s{2,}', line)
            if m:
                hex_str = m.group(1)
                for b in hex_str.split():
                    data.append(int(b, 16))

    return bytes(data) if len(data) > 0 else None


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


def compare_data(extracted, reference, ref_offset):
    """对比 extracted 和 reference[ref_offset:] 的误码率"""
    compare_len = min(len(extracted), len(reference) - ref_offset)
    if compare_len <= 0:
        return None

    total_bits = compare_len * 8
    error_bits = 0
    error_bytes = 0
    error_positions = []  # (offset_in_extracted, extracted_byte, ref_byte)

    for i in range(compare_len):
        eb = extracted[i]
        rb = reference[ref_offset + i]
        if eb != rb:
            bit_errs = count_bit_errors(eb, rb)
            error_bits += bit_errs
            error_bytes += 1
            error_positions.append((i, eb, rb, bit_errs))

    bit_error_rate = (error_bits / total_bits) * 100 if total_bits > 0 else 0
    byte_error_rate = (error_bytes / compare_len) * 100 if compare_len > 0 else 0

    return {
        'compare_len': compare_len,
        'total_bits': total_bits,
        'error_bits': error_bits,
        'error_bytes': error_bytes,
        'bit_error_rate': bit_error_rate,
        'byte_error_rate': byte_error_rate,
        'error_positions': error_positions,
        'ref_offset': ref_offset,
    }


def find_best_offset(extracted, reference, search_start, search_range=0x10000):
    """在 reference 中搜索最佳匹配偏移"""
    # 取 extracted 前 32 字节作为特征
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

    # 按16字节对齐行收集所有差异的行号
    rows_with_diff = set()
    for i in range(compare_len):
        if extracted[i] != reference[ref_offset + i]:
            diff_count += 1
            rows_with_diff.add(i >> 4)  # i // 16

    print("\n[DIFF] 差异详情 (全部):")
    print("-" * 78)
    print("  格式: XX[YY] = 提取数据[app1.bin参考数据]")
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


def analyze_error_patterns(extracted, reference, ref_offset):
    """分析误码规律：逐bit统计 + 全部字节映射"""
    compare_len = min(len(extracted), len(reference) - ref_offset)

    bit_stats = {}
    for b in range(8):
        bit_stats[b] = {"0→1": 0, "1→0": 0, "total": 0}

    byte_map = {}

    for i in range(compare_len):
        eb = extracted[i]
        rb = reference[ref_offset + i]
        if eb != rb:
            key = (eb, rb)
            byte_map[key] = byte_map.get(key, 0) + 1
            for b in range(8):
                e_bit = (eb >> b) & 1
                r_bit = (rb >> b) & 1
                if e_bit != r_bit:
                    bit_stats[b]["total"] += 1
                    if e_bit == 0 and r_bit == 1:
                        bit_stats[b]["0→1"] += 1
                    else:
                        bit_stats[b]["1→0"] += 1

    # ── 逐 bit 误码统计 ──
    print("\n" + "=" * 62)
    print("  逐 Bit 误码统计 (提取 → 参考)")
    print("=" * 62)
    print(f"  {'Bit':<6} {'0→1':>8} {'1→0':>8} {'总误码':>8} {'占比':>8}")
    print("-" * 42)
    total_bit_errors = sum(bit_stats[b]["total"] for b in range(8))
    for b in range(8):
        t = bit_stats[b]["total"]
        pct = f"{t / total_bit_errors * 100:.1f}%" if total_bit_errors > 0 else "0%"
        print(f"  Bit{b:<3} {bit_stats[b]['0→1']:>8} {bit_stats[b]['1→0']:>8} {t:>8} {pct:>8}")

    # ── 全部字节映射 ──
    print("\n" + "=" * 62)
    print("  全部字节映射 (提取 → 参考)  出现次数 / 占比")
    print("=" * 62)
    sorted_patterns = sorted(byte_map.items(), key=lambda x: x[1], reverse=True)
    total_byte_errors = sum(v for _, v in sorted_patterns)

    for (eb, rb), count in sorted_patterns:
        pct = count / total_byte_errors * 100
        bar = "█" * max(1, int(pct / 2))
        print(f"  {eb:02X} → {rb:02X}    {count:>5}  ({pct:5.1f}%)  {bar}")

    print()


def main():
    # 选择参考固件
    if REF_BIN == "app2":
        ref_path = APP2_BIN_PATH
        ref_name = "app2.bin"
    else:
        ref_path = APP1_BIN_PATH
        ref_name = "app1.bin"

    print("=" * 62)
    print("  误码率检测工具 - compare_rec.py")
    print("=" * 62)
    print(f"  参考固件: {ref_name}")
    print()

    # 1. 加载 extracted_firmware.txt
    print("[INFO] 加载提取的固件数据...")
    extracted = parse_extracted_hex(EXTRACTED_FILE)
    if extracted is None:
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      文件: {EXTRACTED_FILE}")
    print(f"      大小: {len(extracted)} bytes ({len(extracted)/1024:.2f} KB)")
    print()

    # 2. 加载参考固件
    print("[INFO] 加载参考固件...")
    reference = read_bin(ref_path)
    if reference is None:
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      文件: {ref_path}")
    print(f"      大小: {len(reference)} bytes ({len(reference)/1024:.2f} KB)")
    print()

    # 3. 自动搜索最佳对齐偏移
    print(f"[INFO] UDS 下载目标地址: 0x{TARGET_FLASH_ADDR:08X}")
    print(f"[INFO] 提取数据大小:     {len(extracted)} bytes")
    print(f"[INFO] 参考文件大小:     {len(reference)} bytes")
    print()
    print("[INFO] 在参考文件中搜索最佳对齐偏移...")
    best_offset = find_best_offset(extracted, reference, 0)
    print(f"      最佳偏移: 0x{best_offset:08X} ({best_offset} bytes)")
    if best_offset == 0:
        print(f"      推测: {ref_name} 起始地址 = 0x{TARGET_FLASH_ADDR:08X}")
    print()

    # 4. 对比误码率
    print("[INFO] 计算误码率...")
    result = compare_data(extracted, reference, best_offset)
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

    # 6. 错误评级
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

    # 7. 打印差异详情
    if result['error_bytes'] > 0:
        print_hex_diff(extracted, reference, best_offset)
        analyze_error_patterns(extracted, reference, best_offset)

    # 8. 打印对比区域的首尾预览
    print("\n" + "=" * 62)
    print("  对比区域头部 (提取 vs 参考)")
    print("=" * 62)
    bytes_per_line = 16
    head_lines = 5
    print("偏移(提取)  提取Hex                           参考Hex")
    print("-" * 62)
    for line_num in range(head_lines):
        start = line_num * bytes_per_line
        if start >= result['compare_len']:
            break
        end = min(start + bytes_per_line, result['compare_len'])
        exc = extracted[start:end]
        refc = reference[best_offset + start:best_offset + end]
        ex_hex = ' '.join(f'{b:02X}' for b in exc).ljust(bytes_per_line * 3 - 1)
        rf_hex = ' '.join(f'{b:02X}' for b in refc)
        marker = "  <--" if any(exc[j] != refc[j] for j in range(len(exc))) else ""
        print(f"  {start:08X}     {ex_hex}  {rf_hex}{marker}")

    print()
    print("=" * 62)
    print("  检测完成!")
    print("=" * 62)

    safe_input("\n按 Enter 键退出...")


if __name__ == "__main__":
    main()
