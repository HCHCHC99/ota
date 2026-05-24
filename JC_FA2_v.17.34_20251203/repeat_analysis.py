#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repeat_analysis.py — OTA/UDS 日志去重分析工具
合并重复的下载周期，输出压缩后的分析报告

用法: py repeat_analysis.py [输入文件] [输出文件]
默认: ota data5.txt → ota data5_compressed.txt
"""

import os
import re
import sys
from collections import Counter, defaultdict

# ========== 路径配置 ==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INPUT = os.path.join(SCRIPT_DIR, "ota data5.txt")
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "ota data5_compressed.txt")
# =============================

# ========== UDS 常量 ==========
UDS_SID = {
    0x10: "SessionControl",
    0x11: "ECUReset",
    0x22: "ReadDataById",
    0x27: "SecurityAccess",
    0x2E: "WriteDataById",
    0x31: "RoutineControl",
    0x34: "RequestDownload",
    0x36: "TransferData",
    0x37: "RequestTransferExit",
    0x3E: "TesterPresent",
}

NRC_NAMES = {
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLength",
    0x21: "busyRepeatRequest",
    0x22: "conditionsNotCorrect",
    0x24: "requestSequenceError",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied",
    0x35: "invalidKey",
    0x36: "exceedNumberOfAttempts",
    0x37: "requiredTimeDelayNotExpired",
    0x78: "responsePending",
}


def hex_to_bytes(hex_str):
    """将纯 hex 字符串(空格分隔)解析为字节列表"""
    result = []
    for part in hex_str.split():
        part = part.strip()
        if re.match(r'^[0-9A-Fa-f]{2}$', part):
            result.append(int(part, 16))
        else:
            break
    return result


def parse_ota_line(line):
    """解析一行 OTA 日志，返回 dict 或 None"""
    line = line.strip()
    if not line:
        return {"_type": "empty", "_raw": line}

    # [FW] 日志
    m = re.match(r"00> \[FW\] (.+)", line)
    if m:
        msg = m.group(1).strip()
        r = {"_type": "fw", "_raw": line, "_msg": msg}

        if "OnRequestDownload" in msg:
            r["event"] = "req_dl"
            m2 = re.search(r'addr=0x([0-9A-Fa-f]+), size=(\d+)', msg)
            if m2:
                r["addr"] = int(m2.group(1), 16)
                r["size"] = int(m2.group(2))
        elif "OnTransferData" in msg:
            r["event"] = "transfer_data"
            m2 = re.search(r'seq=(\d+), len=(\d+)', msg)
            if m2:
                r["block_seq"] = int(m2.group(1))
        elif "OnTransferExit" in msg:
            r["event"] = "transfer_exit"
        elif "Firmware Update Complete" in msg:
            r["event"] = "complete"
        elif "Wrong state for download" in msg:
            r["event"] = "wrong_state_dl"
        elif "Wrong state for transfer exit" in msg:
            r["event"] = "wrong_state_exit"
        elif "Calculated CRC" in msg:
            r["event"] = "crc"
        elif "State:" in msg:
            r["event"] = "state_change"
        elif "Flash write DISABLED" in msg:
            r["event"] = "write_disabled"
        elif "Verifying and writing" in msg:
            r["event"] = "verifying"
        elif "Sectors erased" in msg or "Address:" in msg or "CRC: 0x" in msg:
            r["event"] = "summary"
        return r

    # [OTA] 日志
    m = re.match(
        r"00> \[OTA\] seq=(\d+), time=\s*([\d.]+)s, (\[RX\]|\[TX\]) (0x[0-9A-Fa-f]+), (.+)",
        line
    )
    if not m:
        # FF:/FC: 标注行
        if "FF: response_id" in line or "FC: dst_id" in line:
            return {"_type": "ota_meta", "_raw": line}
        return {"_type": "other", "_raw": line}

    seq = int(m.group(1))
    time_s = float(m.group(2))
    direction = "TX" if "[TX]" in m.group(3) else "RX"
    can_id_str = m.group(4)
    tail = m.group(5).strip()

    # 分离 hex 数据和注释
    comment = ""
    if '<--' in tail:
        idx = tail.index('<--')
        comment = tail[idx:].strip()
        hex_part = tail[:idx].strip()
    else:
        hex_part = tail

    data = hex_to_bytes(hex_part)
    if not data:
        return {"_type": "ota", "_raw": line, "seq": seq, "time": time_s,
                "dir": direction, "can_id": can_id_str, "comment": comment}

    pci = data[0]
    pci_type = pci & 0xF0

    r = {
        "_type": "ota",
        "_raw": line,
        "seq": seq,
        "time": time_s,
        "dir": direction,
        "can_id": can_id_str,
        "data": data,
        "comment": comment,
        "pci_type": pci_type,
    }

    # ISO-TP 解析 + UDS SID 提取
    if pci_type == 0x00:  # SF
        r["isotp"] = "SF"
        r["sf_datalen"] = pci & 0x0F
        uds_data = data[1:] if len(data) > 1 else []
        if len(uds_data) > 0:
            r["uds_sid"] = uds_data[0]
            r["uds_name"] = UDS_SID.get(uds_data[0], "")
            if uds_data[0] == 0x7F and len(uds_data) >= 3:
                r["nrc_sid"] = uds_data[1]
                r["nrc"] = uds_data[2]
                r["nrc_name"] = NRC_NAMES.get(uds_data[2], "")

    elif pci_type == 0x10:  # FF
        r["isotp"] = "FF"
        if len(data) >= 2:
            r["ff_total_len"] = ((pci & 0x0F) << 8) | data[1]
        if len(data) >= 3:
            r["uds_sid"] = data[2]
            r["uds_name"] = UDS_SID.get(data[2], "")

    elif pci_type == 0x20:  # CF
        r["isotp"] = "CF"
        r["cf_seq"] = pci & 0x0F

    elif pci_type == 0x30:  # FC
        r["isotp"] = "FC"

    # TX 正响应 SID 提取
    if direction == "TX" and len(data) > 0 and data[0] >= 0x40:
        orig_sid = data[0] - 0x40
        r["tx_pos_sid"] = orig_sid
        r["tx_pos_name"] = UDS_SID.get(orig_sid, "")

    return r


def detect_cycles(parsed):
    """识别下载周期: Session(0x10)→Security(0x27)→Routine(0x31)→Download(0x34)→..."""
    cycles = []
    current = None

    for i, p in enumerate(parsed):
        if p is None:
            continue

        # 新周期: RX SessionControl + ProgrammingSession (subfunc 0x02)
        if (p["_type"] == "ota" and p["dir"] == "RX" and
            p.get("uds_sid") == 0x10 and
            len(p.get("data", [])) > 2 and p["data"][2] == 0x02):

            if current and len(current["events"]) > 0:
                current["end"] = i
                cycles.append(current)

            current = {"start": i, "end": None, "events": [], "lines": []}

        if current is not None:
            current["lines"].append(i)
            ev = event_key(p)
            if ev:
                current["events"].append(ev)

    if current and len(current["events"]) > 0:
        current["end"] = len(parsed)
        cycles.append(current)

    return cycles


def event_key(p):
    """将解析行转为简短事件标签(用于周期签名)"""
    if p["_type"] == "fw":
        return p.get("event")

    if p["_type"] != "ota":
        return None

    # TX 负响应
    nrc = p.get("nrc")
    if nrc is not None:
        return f"NRC_0x{nrc:02X}"

    # TX 正响应
    tx_sid = p.get("tx_pos_sid")
    if tx_sid is not None:
        return f"POS_0x{tx_sid:02X}"

    # RX UDS SID
    sid = p.get("uds_sid")
    isotp = p.get("isotp", "")
    if sid is not None and isotp in ("SF", "FF"):
        return f"RX_0x{sid:02X}_{isotp}"

    # CF 帧
    if isotp == "CF":
        return "CF"

    # FC 帧
    if isotp == "FC":
        return "FC"

    return None


def cycle_signature(cycle):
    """周期签名: 去重连续事件, 去 CF"""
    sig = []
    for e in cycle["events"]:
        if e in ("CF",):
            if sig and sig[-1] == "CF":
                continue  # 合并连续CF
            sig.append("CF×N")
            continue
        if sig and e == sig[-1]:
            continue
        sig.append(e)
    return tuple(sig)


def cycle_fingerprint(cycle):
    """周期指纹: 只保留控制面事件(不含CF和TransferData)"""
    fp = []
    for e in cycle["events"]:
        if e in ("CF", "CF×N"):
            continue
        if e and e.startswith("transfer_data"):
            continue
        if fp and e == fp[-1]:
            continue
        fp.append(e)
    return tuple(fp)


def format_event_list(events):
    """格式化事件列表为可读字符串"""
    compact = []
    cf_count = 0
    for e in events:
        if e == "CF":
            cf_count += 1
            continue
        if cf_count > 0:
            compact.append(f"CF×{cf_count}")
            cf_count = 0
        compact.append(e)
    if cf_count > 0:
        compact.append(f"CF×{cf_count}")
    return " → ".join(compact)


def compress(parsed, cycles, output_path):
    """生成压缩报告"""
    # 按指纹分组
    groups = defaultdict(list)
    for i, cyc in enumerate(cycles):
        fp = cycle_fingerprint(cyc)
        groups[fp].append((i, cyc))

    # 排序：出现次数多的在前
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("=" * 72 + "\n")
        f.write(f"  OTA 日志压缩分析报告\n")
        f.write(f"  原始行数: {len(parsed):,}  识别周期: {len(cycles)}\n")
        f.write(f"  不同模式: {len(groups)}\n")
        f.write("=" * 72 + "\n")

        for gp_idx, (fp, members) in enumerate(sorted_groups, 1):
            num = len(members)
            first_idx, first = members[0]
            f.write(f"\n{'─' * 72}\n")
            f.write(f"【模式 {gp_idx}】 出现 {num} 次\n")
            f.write(f"  指纹: {format_event_list(list(fp))}\n")

            # 时间范围
            all_times = []
            for _, cyc in members:
                for li in cyc["lines"]:
                    p = parsed[li]
                    if p["_type"] == "ota" and "time" in p:
                        all_times.append(p["time"])
            if all_times:
                f.write(f"  时间: {min(all_times):.2f}s ~ {max(all_times):.2f}s"
                        f"  (跨度 {max(all_times)-min(all_times):.1f}s)\n")

            # 首次出现的详细日志
            f.write(f"\n  ── 首次出现 (行 {first['start']+1}~{first['end']}) ──\n")
            cf_run = 0
            for li in first["lines"]:
                p = parsed[li]
                if p["_type"] == "ota" and p.get("isotp") == "CF":
                    cf_run += 1
                    continue
                # 输出被跳过的 CF 块
                if cf_run > 0:
                    f.write(f"    ... (CF 数据帧 ×{cf_run}) ...\n")
                    cf_run = 0

                raw = p.get("_raw", "")
                if p["_type"] in ("fw", "empty"):
                    f.write(f"    {raw}\n")
                elif p["_type"] == "ota":
                    comment = p.get("comment", "")
                    isotp = p.get("isotp", "")
                    if isotp in ("SF", "FF", "FC") or p.get("uds_sid") is not None:
                        hex_str = ' '.join(f'{b:02X}' for b in p.get("data", []))
                        f.write(f"    [{p['dir']}] {hex_str}  {comment}\n")
                elif p["_type"] == "ota_meta":
                    f.write(f"    {raw}\n")

            # 后续出现(只列出时间)
            if num > 1:
                f.write(f"\n  ── 后续 {num-1} 次 (时间/行号) ──\n")
                for j, (_, cyc) in enumerate(members[1:], 2):
                    cyc_times = [parsed[li]["time"] for li in cyc["lines"]
                                 if parsed[li]["_type"] == "ota" and "time" in parsed[li]]
                    t_str = f"{cyc_times[0]:.1f}s" if cyc_times else "?"
                    f.write(f"    第{j}次: {t_str}  行{cyc['start']+1}~{cyc['end']}\n")

        # ============ 统计摘要 ============
        f.write(f"\n\n{'=' * 72}\n")
        f.write(f"  统计摘要\n")
        f.write(f"{'=' * 72}\n")

        fw_events = Counter()
        ota_stats = {"TX": 0, "RX": 0, "SF": 0, "FF": 0, "CF": 0, "FC": 0}
        nrc_stats = Counter()
        uds_rx = Counter()
        uds_tx = Counter()

        for p in parsed:
            if p["_type"] == "fw":
                fw_events[p.get("event", "other")] += 1
            elif p["_type"] == "ota":
                ota_stats[p["dir"]] += 1
                isotp = p.get("isotp", "")
                if isotp in ota_stats:
                    ota_stats[isotp] += 1
                nrc = p.get("nrc")
                if nrc is not None:
                    nrc_stats[f"0x{nrc:02X}"] += 1
                if p["dir"] == "RX":
                    sid = p.get("uds_sid")
                    if sid is not None:
                        uds_rx[sid] += 1
                elif p["dir"] == "TX":
                    sid = p.get("tx_pos_sid")
                    if sid is not None:
                        uds_tx[sid] += 1

        f.write(f"\n  行数统计:\n")
        f.write(f"    总行数:      {len(parsed):,}\n")
        f.write(f"    CF 数据帧:   {ota_stats['CF']:,}\n")
        f.write(f"    SF/FF/FC:    {ota_stats['SF']}/{ota_stats['FF']}/{ota_stats['FC']}\n")
        f.write(f"    TX/RX:       {ota_stats['TX']}/{ota_stats['RX']}\n")

        f.write(f"\n  下载周期:\n")
        f.write(f"    总计:        {len(cycles)}\n")
        f.write(f"    不同模式:    {len(groups)}\n")
        f.write(f"    成功完成:    {fw_events.get('complete', 0)}\n")
        f.write(f"    状态错误DL:  {fw_events.get('wrong_state_dl', 0)}\n")
        f.write(f"    状态错误Exit:{fw_events.get('wrong_state_exit', 0)}\n")
        f.write(f"    TransferData: {fw_events.get('transfer_data', 0)}\n")
        f.write(f"    TransferExit: {fw_events.get('transfer_exit', 0)}\n")

        f.write(f"\n  NRC 负响应统计:\n")
        for nrc, cnt in nrc_stats.most_common():
            name = NRC_NAMES.get(int(nrc, 16), "")
            f.write(f"    {nrc} ({name}): {cnt}\n")

        f.write(f"\n  RX UDS 服务:\n")
        for sid, cnt in sorted(uds_rx.items()):
            name = UDS_SID.get(sid, "")
            f.write(f"    0x{sid:02X} ({name}): {cnt}\n")

        f.write(f"\n  TX UDS 正响应:\n")
        for sid, cnt in sorted(uds_tx.items()):
            name = UDS_SID.get(sid, "")
            f.write(f"    0x{sid:02X} ({name}): {cnt}\n")

        # 模式分布图
        f.write(f"\n  模式分布:\n")
        for gp_idx, (fp, members) in enumerate(sorted_groups, 1):
            num = len(members)
            bar = "█" * min(num, 50)
            f.write(f"    [{gp_idx}] {num:>2}次  {bar}\n")

    print(f"[INFO] 压缩报告已保存: {output_path}")


def main():
    input_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INPUT
    output_file = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUTPUT

    if not os.path.exists(input_file):
        print(f"[ERROR] 文件不存在: {input_file}")
        return

    print(f"[INFO] 读取: {input_file}")
    with open(input_file, 'r', encoding='gbk', errors='replace') as f:
        raw_lines = f.readlines()

    print(f"[INFO] 解析 {len(raw_lines):,} 行...")
    parsed = [parse_ota_line(line) for line in raw_lines]

    # 统计解析覆盖率
    ota = sum(1 for p in parsed if p["_type"] == "ota")
    fw = sum(1 for p in parsed if p["_type"] == "fw")
    meta = sum(1 for p in parsed if p["_type"] == "ota_meta")
    empty = sum(1 for p in parsed if p["_type"] == "empty")
    other = sum(1 for p in parsed if p["_type"] == "other")
    print(f"[INFO] 解析结果: OTA={ota}, FW={fw}, meta={meta}, empty={empty}, other={other}")

    print(f"[INFO] 识别下载周期...")
    cycles = detect_cycles(parsed)
    print(f"[INFO] 找到 {len(cycles)} 个下载周期")

    if len(cycles) > 0:
        print(f"[INFO] 生成压缩报告...")
        compress(parsed, cycles, output_file)
    else:
        print("[WARN] 未找到周期，输出全部关键行")
        with open(output_file, 'w', encoding='utf-8') as f:
            for p in parsed:
                if p["_type"] == "ota" and p.get("isotp") == "CF":
                    continue
                f.write(p.get("_raw", "") + "\n")
        print(f"[INFO] 已保存非CF行: {output_file}")

    original_size = os.path.getsize(input_file)
    compressed_size = os.path.getsize(output_file)
    ratio = (1 - compressed_size / original_size) * 100
    print(f"[INFO] {original_size:,} → {compressed_size:,} bytes ({ratio:.1f}% 缩减)")

    try:
        input("\n按 Enter 键退出...")
    except EOFError:
        pass


if __name__ == "__main__":
    main()
