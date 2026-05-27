#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hexdump.py — 将 app1.bin / app2.bin 以 hex dump 格式输出为 txt 文件
"""

import os

# ========== 路径配置（在此修改）==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APP1_BIN_PATH = os.path.join(SCRIPT_DIR, "app1.bin")
APP2_BIN_PATH = os.path.join(SCRIPT_DIR, "app2.bin")
# =========================================


def read_bin(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return None
    with open(filepath, 'rb') as f:
        return f.read()


def save_hex_dump(data, filepath):
    bytes_per_line = 16
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("  Offset    Hex                                              ASCII\n")
        f.write("  --------  -----------------------------------------------  ----------------\n")
        for i in range(0, len(data), bytes_per_line):
            chunk = data[i:i + bytes_per_line]
            offset = f"{i:08X}"
            first8 = chunk[:8]
            last8 = chunk[8:]
            hex_first = ' '.join(f'{b:02X}' for b in first8)
            hex_last = ' '.join(f'{b:02X}' for b in last8)
            hex_part = f"{hex_first:<23}  {hex_last:<23}"
            ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
            f.write(f"  {offset}  {hex_part}  {ascii_part}\n")


def process_bin(bin_path, label):
    """处理单个 bin 文件，生成对应的 txt"""
    txt_name = label.replace(".bin", "_bin.txt")
    txt_path = os.path.join(SCRIPT_DIR, txt_name)

    print(f"[INFO] 读取: {bin_path}")
    data = read_bin(bin_path)
    if data is None:
        return False
    print(f"      大小: {len(data)} bytes ({len(data)/1024:.2f} KB)")

    save_hex_dump(data, txt_path)
    print(f"      输出: {txt_name} ({(len(data) + 15) // 16} 行)")
    return True


def safe_input(prompt=""):
    try:
        return input(prompt)
    except EOFError:
        return ""


def main():
    print("=" * 62)
    print("  Bin Hex Dump 工具 - hexdump.py")
    print("=" * 62)
    print()

    ok = process_bin(APP1_BIN_PATH, "app1.bin")
    print()
    ok |= process_bin(APP2_BIN_PATH, "app2.bin")

    print()
    print("=" * 62)
    print("  完成!" if ok else "  部分文件未找到，已跳过")
    print("=" * 62)

    safe_input("\n按 Enter 键退出...")


if __name__ == "__main__":
    main()
