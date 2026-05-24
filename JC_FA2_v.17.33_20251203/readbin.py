import os
import binascii

# ========== 配置区域 - 请在这里修改文件路径 ==========
# app1.bin 文件路径（请修改为实际路径）
APP1_BIN_PATH = r"F:\Enterprise WeChat\WXWork\1688858205719851\Cache\File\2026-04\app1.bin"

# app2.bin 文件路径（请修改为实际路径）
APP2_BIN_PATH = r"F:\Enterprise WeChat\WXWork\1688858205719851\Cache\File\2026-04\app2.bin"
# =================================================

def read_bin_to_formatted_hex(file_path, bytes_per_line=16):
    """读取二进制文件并返回格式化的十六进制显示"""
    try:
        with open(file_path, 'rb') as f:
            data = f.read()
            
        result = []
        file_size = len(data)
        
        for i in range(0, file_size, bytes_per_line):
            # 偏移地址
            offset = f"{i:08X}"
            
            # 十六进制部分
            chunk = data[i:i+bytes_per_line]
            hex_part = ' '.join(f"{b:02X}" for b in chunk)
            hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
            
            # ASCII部分（可打印字符显示，否则显示.）
            ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
            
            result.append(f"{offset}  {hex_part}  {ascii_part}")
        
        return '\n'.join(result), file_size, data
        
    except FileNotFoundError:
        return f"错误：文件未找到 - {file_path}", 0, None
    except Exception as e:
        return f"读取文件时出错：{str(e)}", 0, None

def get_position_percentage(offset, file_size):
    """计算位置百分比"""
    if file_size == 0:
        return 0.0
    percentage = (offset / file_size) * 100
    return percentage

def search_keyword_in_binary(data, keyword):
    """在二进制数据中搜索关键词（支持中文、空格等）"""
    matches = []
    
    try:
        # 将关键词转换为字节
        keyword_bytes = keyword.encode('utf-8')
        
        # 在数据中搜索
        pos = 0
        while pos < len(data):
            pos = data.find(keyword_bytes, pos)
            if pos == -1:
                break
            matches.append(pos)
            pos += 1
            
        return matches, keyword_bytes
    except Exception as e:
        return [], None

def search_hex_pattern(data, hex_pattern):
    """搜索十六进制模式（例如：AA BB CC）"""
    matches = []
    
    try:
        # 移除空格并转换为字节
        hex_str = hex_pattern.replace(' ', '')
        if len(hex_str) % 2 != 0:
            return [], None
        
        pattern_bytes = bytes.fromhex(hex_str)
        
        # 在数据中搜索
        pos = 0
        while pos < len(data):
            pos = data.find(pattern_bytes, pos)
            if pos == -1:
                break
            matches.append(pos)
            pos += 1
            
        return matches, pattern_bytes
    except Exception as e:
        return [], None

def print_surrounding_lines(data, offset, keyword_len, file_name, file_size, context_lines=10):
    """打印搜索位置周围的上下若干行"""
    bytes_per_line = 16
    lines_per_side = context_lines
    
    # 计算起始和结束的偏移地址（按字节对齐到行首）
    start_line = max(0, (offset // bytes_per_line) - lines_per_side)
    end_line = min((len(data) + bytes_per_line - 1) // bytes_per_line, 
                   (offset + keyword_len + bytes_per_line - 1) // bytes_per_line + lines_per_side)
    
    # 计算位置百分比
    percentage = get_position_percentage(offset, file_size)
    end_percentage = get_position_percentage(offset + keyword_len, file_size)
    
    print(f"\n{'=' * 80}")
    print(f"[{file_name}] 搜索结果周围 {context_lines} 行内容")
    print(f"关键词位置: 0x{offset:08X} - 0x{offset + keyword_len - 1:08X}")
    print(f"文件位置: {percentage:.2f}% - {end_percentage:.2f}%")
    print(f"显示范围: 第 {start_line + 1} 行 到 第 {end_line} 行")
    print(f"{'=' * 80}")
    
    # 打印表头
    print("偏移地址(Hex)  Hex内容                          ASCII")
    print("-" * 80)
    
    # 生成并打印每一行
    for line_num in range(start_line, end_line):
        line_start = line_num * bytes_per_line
        line_end = min(line_start + bytes_per_line, len(data))
        chunk = data[line_start:line_end]
        
        # 格式化输出
        offset_str = f"{line_start:08X}"
        hex_part = ' '.join(f"{b:02X}" for b in chunk)
        hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
        ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        
        # 标记包含关键词的行
        if line_start <= offset < line_start + bytes_per_line or \
           line_start <= offset + keyword_len - 1 < line_start + bytes_per_line:
            print(f">>> {offset_str}  {hex_part}  {ascii_part}  <<<")
        else:
            print(f"    {offset_str}  {hex_part}  {ascii_part}")
    
    print("=" * 80)
    
    # 添加进度条可视化
    print(f"\n文件位置进度: [{get_progress_bar(percentage)}] {percentage:.2f}%")
    if end_percentage > percentage:
        print(f"关键词结束位置: [{get_progress_bar(end_percentage)}] {end_percentage:.2f}%")

def get_progress_bar(percentage, length=50):
    """生成进度条"""
    filled_length = int(length * percentage // 100)
    bar = '█' * filled_length + '░' * (length - filled_length)
    return bar

def display_search_results(matches, keyword, data, keyword_bytes, file_name, file_path, file_size):
    """显示搜索结果"""
    if not matches:
        print(f"\n[{file_name}] 未找到关键词: \"{keyword}\"")
        return False
    
    print(f"\n[{file_name}] ({file_path})")
    print(f"文件大小: {file_size} 字节")
    print(f"找到 {len(matches)} 处匹配:")
    print("-" * 80)
    
    for i, offset in enumerate(matches, 1):
        # 计算位置百分比
        percentage = get_position_percentage(offset, file_size)
        
        print(f"{i}. 偏移地址: 0x{offset:08X} ({offset} 字节)")
        print(f"   文件位置: {percentage:.2f}%")
        print(f"   进度条: [{get_progress_bar(percentage)}]")
        
        # 显示关键词周围的上下文（前后各8字节）
        start = max(0, offset - 8)
        end = min(len(data), offset + len(keyword_bytes) + 8)
        context = data[start:end]
        
        # 显示十六进制上下文
        hex_context = ' '.join(f"{b:02X}" for b in context)
        # 显示ASCII上下文
        ascii_context = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in context)
        
        # 标记关键词位置
        kw_start_in_context = offset - start
        kw_end_in_context = kw_start_in_context + len(keyword_bytes)
        
        print(f"   上下文(Hex): {hex_context}")
        print(f"   上下文(ASC): {ascii_context}")
        print(f"   关键词位置: 从字节 {kw_start_in_context} 到 {kw_end_in_context-1}")
        
        # 询问是否打印周围10行
        print()
        print_choice = input(f"是否打印此匹配项周围的上下10行？(y/n，默认n): ").strip().lower()
        if print_choice == 'y':
            print_surrounding_lines(data, offset, len(keyword_bytes), file_name, file_size, 10)
        
        print()
    
    return True

def print_file_head_tail(data, file_name, file_size, head_lines=10, tail_lines=10):
    """打印文件的首尾若干行"""
    if data is None:
        print(f"[{file_name}] 文件数据为空，无法显示")
        return
    
    bytes_per_line = 16
    total_lines = (file_size + bytes_per_line - 1) // bytes_per_line
    
    print(f"\n{'=' * 80}")
    print(f"[{file_name}] 文件信息")
    print(f"文件大小: {file_size} 字节")
    print(f"总行数: {total_lines} 行 (每行16字节)")
    print(f"{'=' * 80}")
    
    # 打印头部
    print(f"\n【头部 {head_lines} 行】")
    print("偏移地址(Hex)  Hex内容                          ASCII")
    print("-" * 80)
    
    for line_num in range(min(head_lines, total_lines)):
        line_start = line_num * bytes_per_line
        line_end = min(line_start + bytes_per_line, file_size)
        chunk = data[line_start:line_end]
        
        offset_str = f"{line_start:08X}"
        hex_part = ' '.join(f"{b:02X}" for b in chunk)
        hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
        ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        
        print(f"{offset_str}  {hex_part}  {ascii_part}")
    
    # 如果文件足够大，显示省略号
    if total_lines > head_lines + tail_lines:
        print(f"\n... (中间省略 {total_lines - head_lines - tail_lines} 行) ...\n")
    
    # 打印尾部
    print(f"\n【尾部 {tail_lines} 行】")
    print("偏移地址(Hex)  Hex内容                          ASCII")
    print("-" * 80)
    
    for line_num in range(max(0, total_lines - tail_lines), total_lines):
        line_start = line_num * bytes_per_line
        line_end = min(line_start + bytes_per_line, file_size)
        chunk = data[line_start:line_end]
        
        offset_str = f"{line_start:08X}"
        hex_part = ' '.join(f"{b:02X}" for b in chunk)
        hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
        ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        
        # 标记这是尾部
        marker = "  [尾部区域]" if line_num >= total_lines - tail_lines else ""
        print(f"{offset_str}  {hex_part}  {ascii_part}{marker}")
    
    print("=" * 80)

def load_file(file_path, file_name):
    """加载单个文件"""
    print(f"\n正在加载 [{file_name}]: {file_path}")
    
    if not os.path.exists(file_path):
        print(f"警告：文件不存在 - {file_path}")
        return None, 0
    
    file_size = os.path.getsize(file_path)
    print(f"文件大小：{file_size} 字节 ({file_size} B)")
    
    _, _, file_data = read_bin_to_formatted_hex(file_path)
    
    if file_data is None:
        print(f"读取文件失败：{file_path}")
        return None, 0
    
    return file_data, file_size

def display_file_content(file_data, file_name, max_lines=30):
    """显示文件内容"""
    if file_data is None:
        return
    
    # 生成格式化的内容
    lines = []
    for i in range(0, len(file_data), 16):
        offset = f"{i:08X}"
        chunk = file_data[i:i+16]
        hex_part = ' '.join(f"{b:02X}" for b in chunk)
        hex_part = hex_part.ljust(16 * 3 - 1)
        ascii_part = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
        lines.append(f"{offset}  {hex_part}  {ascii_part}")
    
    print(f"\n[{file_name}] 文件内容预览:")
    print("=" * 80)
    print("偏移地址(Hex)  Hex内容                          ASCII")
    print("=" * 80)
    
    for line in lines[:max_lines]:
        print(line)
    
    if len(lines) > max_lines:
        print(f"\n... (共{len(lines)}行，仅显示前{max_lines}行) ...")
    
    print("=" * 80)

def check_file_exists(file_path, file_name):
    """检查文件是否存在并显示信息"""
    if os.path.exists(file_path):
        size = os.path.getsize(file_path)
        print(f"✓ {file_name}: 找到 (大小: {size} 字节)")
        return True
    else:
        print(f"✗ {file_name}: 未找到 - {file_path}")
        return False

def main():
    print("=" * 80)
    print("二进制文件搜索工具 - 支持 app1.bin 和 app2.bin")
    print("=" * 80)
    
    # 显示配置信息
    print("\n【当前文件配置】")
    print(f"app1.bin 路径：{APP1_BIN_PATH}")
    print(f"app2.bin 路径：{APP2_BIN_PATH}")
    print("\n提示：如需修改路径，请编辑脚本开头的配置区域")
    
    # 检查文件是否存在
    print("\n【检查文件】")
    app1_exists = check_file_exists(APP1_BIN_PATH, "app1.bin")
    app2_exists = check_file_exists(APP2_BIN_PATH, "app2.bin")
    
    if not app1_exists and not app2_exists:
        print("\n错误：没有找到任何有效的文件！")
        print("请修改脚本开头的 APP1_BIN_PATH 和 APP2_BIN_PATH 变量")
        input("\n按 Enter 键退出...")
        return
    
    # 加载两个文件
    app1_data, app1_size = load_file(APP1_BIN_PATH, "app1.bin")
    app2_data, app2_size = load_file(APP2_BIN_PATH, "app2.bin")
    
    # 检查是否有文件成功加载
    if app1_data is None and app2_data is None:
        print("\n错误：没有找到任何有效的文件！")
        input("\n按 Enter 键退出...")
        return
    
    # 询问是否显示文件内容
    show_content = input("\n是否显示文件内容？(y/n，默认n): ").strip().lower()
    if show_content == 'y':
        if app1_data:
            display_file_content(app1_data, "app1.bin", 30)
        if app2_data:
            display_file_content(app2_data, "app2.bin", 30)
    
    # 搜索功能循环
    while True:
        print("\n" + "=" * 80)
        print("搜索功能菜单")
        print("=" * 80)
        print("1. 搜索字符串（支持中文、空格等）")
        print("2. 搜索十六进制值（例如：55 AA FF）")
        print("3. 显示文件首尾10行")
        print("4. 退出程序")
        
        choice = input("\n请选择 (1/2/3/4): ").strip()
        
        if choice == '4':
            break
        
        elif choice == '3':
            print("\n" + "=" * 80)
            print("显示文件首尾10行")
            print("=" * 80)
            
            # 选择要显示的文件
            print("请选择要显示的文件：")
            print("1. app1.bin")
            print("2. app2.bin")
            print("3. 同时显示两个文件")
            
            file_choice = input("\n请选择 (1/2/3): ").strip()
            
            if file_choice == '1':
                if app1_data:
                    print_file_head_tail(app1_data, "app1.bin", app1_size, 10, 10)
                else:
                    print("app1.bin 未加载成功")
            
            elif file_choice == '2':
                if app2_data:
                    print_file_head_tail(app2_data, "app2.bin", app2_size, 10, 10)
                else:
                    print("app2.bin 未加载成功")
            
            elif file_choice == '3':
                if app1_data:
                    print_file_head_tail(app1_data, "app1.bin", app1_size, 10, 10)
                else:
                    print("app1.bin 未加载成功")
                
                if app2_data:
                    print_file_head_tail(app2_data, "app2.bin", app2_size, 10, 10)
                else:
                    print("app2.bin 未加载成功")
            
            else:
                print("无效选择")
        
        elif choice == '1':
            keyword = input("\n请输入要搜索的字符串（可包含空格）: ")
            if not keyword:
                print("输入不能为空！")
                continue
            
            print("\n" + "=" * 80)
            print(f"正在搜索关键词: \"{keyword}\"")
            print("=" * 80)
            
            found_any = False
            
            if app1_data:
                matches1, keyword_bytes1 = search_keyword_in_binary(app1_data, keyword)
                if display_search_results(matches1, keyword, app1_data, keyword_bytes1, "app1.bin", APP1_BIN_PATH, app1_size):
                    found_any = True
            
            if app2_data:
                matches2, keyword_bytes2 = search_keyword_in_binary(app2_data, keyword)
                if display_search_results(matches2, keyword, app2_data, keyword_bytes2, "app2.bin", APP2_BIN_PATH, app2_size):
                    found_any = True
            
            if not found_any:
                print(f"\n未在 app1.bin 和 app2.bin 中找到关键词: \"{keyword}\"")
            
            # 询问是否继续搜索
            cont = input("\n是否继续搜索？(y/n，默认y): ").strip().lower()
            if cont == 'n':
                break
        
        elif choice == '2':
            hex_pattern = input("\n请输入要搜索的十六进制（例如：55 AA FF 或 55AAFF）: ")
            if not hex_pattern:
                print("输入不能为空！")
                continue
            
            print("\n" + "=" * 80)
            print(f"正在搜索十六进制: {hex_pattern}")
            print("=" * 80)
            
            found_any = False
            
            if app1_data:
                matches1, pattern_bytes1 = search_hex_pattern(app1_data, hex_pattern)
                if display_search_results(matches1, hex_pattern, app1_data, pattern_bytes1, "app1.bin", APP1_BIN_PATH, app1_size):
                    found_any = True
            
            if app2_data:
                matches2, pattern_bytes2 = search_hex_pattern(app2_data, hex_pattern)
                if display_search_results(matches2, hex_pattern, app2_data, pattern_bytes2, "app2.bin", APP2_BIN_PATH, app2_size):
                    found_any = True
            
            if not found_any:
                print(f"\n未在 app1.bin 和 app2.bin 中找到十六进制: {hex_pattern}")
            
            # 询问是否继续搜索
            cont = input("\n是否继续搜索？(y/n，默认y): ").strip().lower()
            if cont == 'n':
                break
        
        else:
            print("无效选择，请输入 1、2、3 或 4")
    
    # 保持窗口打开
    print("\n感谢使用！")
    input("\n按 Enter 键退出...")

if __name__ == "__main__":
    main()