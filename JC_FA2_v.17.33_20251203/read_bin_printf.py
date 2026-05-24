#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
read_bin_printf.py — 读取 app1.bin 并以 hex dump 格式输出到 app1_bin.txt
"""

import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APP1_BIN_PATH = r"F:\Enterprise WeChat\WXWork\1688858205719851\Cache\File\2026-04\app1.bin"
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "app1_bin.txt")


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


def safe_input(prompt=""):
    try:
        return input(prompt)
    except EOFError:
        return ""


def main():
    print("=" * 62)
    print("  app1.bin Hex Dump 工具 - read_bin_printf.py")
    print("=" * 62)
    print()

    print(f"[INFO] 读取: {APP1_BIN_PATH}")
    data = read_bin(APP1_BIN_PATH)
    if data is None:
        safe_input("\n按 Enter 键退出...")
        return
    print(f"      大小: {len(data)} bytes ({len(data)/1024:.2f} KB)")
    print()

    print(f"[INFO] 保存 hex dump: {OUTPUT_FILE}")
    save_hex_dump(data, OUTPUT_FILE)
    print(f"      完成! 共 {(len(data) + 15) // 16} 行 (每行16字节)")

    print()
    print("=" * 62)
    print("  完成!")
    print("=" * 62)

    safe_input("\n按 Enter 键退出...")


if __name__ == "__main__":
    main()
