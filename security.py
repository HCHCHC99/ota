#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Security Access - Seed to Key Calculator
基于提供的C代码实现的种子到密钥计算工具
"""

import sys

# 常量定义
CN_LEN = 6
LV1_CODE = 0xE738

def crc8(data):
    """
    计算CRC8校验值
    算法: 初始值0xFF, 多项式0x1D, 最终异或0xFF
    """
    crc = 0xff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if (crc & 0x80) != 0:
                crc <<= 1
                crc ^= 0x1d
            else:
                crc <<= 1
            crc &= 0xff  # 确保在8位范围内
    return (~crc) & 0xff

def compute_key_from_seed(seed):
    """
    根据种子计算密钥的核心函数
    返回4字节的密钥
    """
    # 复制种子数据
    buf_byte = list(seed[:CN_LEN])
    crc_byte = [0] * 7
    
    # 第1次CRC8计算
    crc_byte[0] = crc8(buf_byte)
    buf_byte[0] = crc_byte[0]
    
    # 第2次CRC8计算
    crc_byte[1] = crc8(buf_byte)
    
    # 重置缓冲区
    buf_byte = list(seed[:CN_LEN])
    buf_byte[0] = seed[0]
    buf_byte[1] = crc_byte[1]
    crc_byte[2] = crc8(buf_byte)
    
    buf_byte[1] = seed[1]
    buf_byte[2] = crc_byte[2]
    crc_byte[3] = crc8(buf_byte)
    
    buf_byte[2] = seed[2]
    buf_byte[3] = crc_byte[3]
    crc_byte[4] = crc8(buf_byte)
    
    buf_byte[3] = seed[3]
    buf_byte[4] = crc_byte[4]
    crc_byte[5] = crc8(buf_byte)
    
    buf_byte[4] = seed[4]
    buf_byte[5] = crc_byte[5]
    crc_byte[6] = crc8(buf_byte)
    
    # 根据条件选择密钥
    key = [0] * 4
    if crc_byte[3] == 0 and crc_byte[4] == 0 and crc_byte[5] == 0 and crc_byte[6] == 0:
        key[0] = crc_byte[1]
        key[1] = crc_byte[2]
        key[2] = crc_byte[3]
        key[3] = crc_byte[4]
    else:
        key[0] = crc_byte[3]
        key[1] = crc_byte[4]
        key[2] = crc_byte[5]
        key[3] = crc_byte[6]
    
    return key

def seedkey_calc_lv1_key(seed_4bytes):
    """
    计算Level 1密钥
    输入: 4字节的种子
    输出: 4字节的密钥
    """
    if len(seed_4bytes) != 4:
        raise ValueError(f"种子必须是4字节，当前长度: {len(seed_4bytes)}")
    
    # 组合种子和固定值0xE738
    new_seed = [0] * CN_LEN
    for i in range(4):
        new_seed[i] = seed_4bytes[i]
    new_seed[4] = (LV1_CODE >> 8) & 0xff   # 0xE7
    new_seed[5] = LV1_CODE & 0xff          # 0x38
    
    return compute_key_from_seed(new_seed)

def bytes_to_hex_string(byte_list):
    """将字节列表转换为十六进制字符串"""
    return ' '.join(f'{b:02X}' for b in byte_list)

def hex_string_to_bytes(hex_str):
    """将十六进制字符串转换为字节列表"""
    # 移除空格和常见分隔符
    hex_str = hex_str.strip().replace(' ', '').replace(',', '').replace('-', '')
    # 确保长度为偶数
    if len(hex_str) % 2 != 0:
        raise ValueError(f"无效的十六进制字符串: {hex_str}")
    
    # 转换为字节列表
    bytes_list = []
    for i in range(0, len(hex_str), 2):
        bytes_list.append(int(hex_str[i:i+2], 16))
    return bytes_list

def main():
    print("=" * 60)
    print("Security Access - Seed to Key Calculator")
    print("基于ISO 14229 UDS安全访问算法实现")
    print("=" * 60)
    print()
    
    # ================================================================
    # 手动修改这里的输入参数
    # ================================================================
    
    # 种子输入方式：
    # 方式1: 使用十六进制字节列表 (推荐)
    # 例如: [0x12, 0x34, 0x56, 0x78] 或 [0x12, 0x34, 0x56, 0x78]
    DEFAULT_SEED_BYTES = [0x12, 0x34, 0x56, 0x78]
    
    # 方式2: 使用十六进制字符串 (如果上面的列表为None，则使用字符串)
    # 例如: "12 34 56 78" 或 "12345678" 或 "12-34-56-78"
    DEFAULT_SEED_STRING = "12 34 56 78"
    
    # 选择使用哪种方式: "bytes" 或 "string"
    INPUT_MODE = "bytes"  # 修改为 "string" 可使用字符串输入
    
    # ================================================================
    
    print("输入参数配置:")
    print(f"  输入模式: {INPUT_MODE}")
    
    # 获取种子
    try:
        if INPUT_MODE == "bytes":
            seed_bytes = DEFAULT_SEED_BYTES
            print(f"  种子(字节): {bytes_to_hex_string(seed_bytes)}")
        else:
            seed_bytes = hex_string_to_bytes(DEFAULT_SEED_STRING)
            print(f"  种子(字符串): {DEFAULT_SEED_STRING}")
            print(f"  种子(字节): {bytes_to_hex_string(seed_bytes)}")
        
        # 验证种子长度
        if len(seed_bytes) != 4:
            print(f"\n错误: 种子必须是4字节，当前为{len(seed_bytes)}字节: {bytes_to_hex_string(seed_bytes)}")
            print("请修改DEFAULT_SEED_BYTES或DEFAULT_SEED_STRING为4字节数据")
        else:
            # 计算密钥
            print(f"\n正在计算密钥...")
            print(f"  固定常量 LV1_CODE: 0x{LV1_CODE:04X}")
            print(f"  扩展种子: {bytes_to_hex_string(seed_bytes)} + {LV1_CODE >> 8:02X} {LV1_CODE & 0xFF:02X}")
            
            key = seedkey_calc_lv1_key(seed_bytes)
            
            print("\n" + "=" * 60)
            print("计算结果:")
            print(f"  种子: {bytes_to_hex_string(seed_bytes)}")
            print(f"  密钥: {bytes_to_hex_string(key)}")
            print("=" * 60)
            
            # 显示详细的计算过程信息
            print("\n详细计算过程:")
            print(f"  完整种子(6字节): {bytes_to_hex_string(seed_bytes)} + {LV1_CODE >> 8:02X} {LV1_CODE & 0xFF:02X}")
            
    except ValueError as e:
        print(f"\n输入解析错误: {e}")
        print("请检查种子输入格式")
    except Exception as e:
        print(f"\n未知错误: {e}")
    
    print("\n" + "-" * 60)
    print("提示: 如需修改种子，请编辑脚本中的 DEFAULT_SEED_BYTES 或 DEFAULT_SEED_STRING")
    print("按任意键退出...")
    input()  # 等待用户输入，保持窗口打开

if __name__ == "__main__":
    main()