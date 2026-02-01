#!/usr/bin/env python3
"""
启动脚本：同时启动 Webots 模拟器和网页可视化服务器
Usage: python3 launch_with_visualization.py
"""

import subprocess
import sys
import time
import os
import platform

def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 60)
    print("🚀 Unibots 模拟器 + 网页可视化启动器")
    print("=" * 60)
    
    # 1. 启动网页服务器
    print("\n[1/2] 启动网页服务器...")
    web_server_path = os.path.join(script_dir, "web_server.py")
    
    try:
        web_process = subprocess.Popen(
            [sys.executable, web_server_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print("✅ 网页服务器已启动 (PID: {})".format(web_process.pid))
        print("📺 打开浏览器访问: http://localhost:5000")
    except Exception as e:
        print(f"❌ 启动网页服务器失败: {e}")
        sys.exit(1)
    
    # 等待服务器启动
    time.sleep(2)
    
    # 2. 打开浏览器
    print("\n[2/2] 打开浏览器...")
    try:
        if platform.system() == 'Darwin':  # macOS
            subprocess.run(['open', 'http://localhost:5000'], check=False)
        elif platform.system() == 'Windows':
            os.startfile('http://localhost:5000')
        elif platform.system() == 'Linux':
            subprocess.run(['xdg-open', 'http://localhost:5000'], check=False)
        print("✅ 浏览器已打开")
    except Exception as e:
        print(f"⚠️  无法自动打开浏览器: {e}")
        print("   请手动访问: http://localhost:5000")
    
    print("\n" + "=" * 60)
    print("✨ 准备就绪！现在启动 Webots 模拟器...")
    print("=" * 60)
    print("\n提示:")
    print("  • 网页服务器运行在后台")
    print("  • 打开浏览器到 http://localhost:5000 查看实时数据")
    print("  • 按 Ctrl+C 停止所有服务")
    print()
    
    # 3. 启动 Webots（阻塞）
    # 注意：这里假设你已经在 Webots 中配置了 supervisor_controller 为默认控制器
    # 如果需要从命令行启动特定的世界文件，可以修改下面的命令
    
    try:
        # 尝试启动 Webots
        # 如果你想启动特定的世界文件，取消下面的注释并修改路径
        
        # 方案1: 如果已在 Webots GUI 中配置好，直接运行 supervisor_controller
        # webots_cmd = [sys.executable, os.path.join(script_dir, "supervisor_controller.py")]
        
        # 方案2: 启动特定的世界文件（推荐）
        worlds_dir = os.path.join(os.path.dirname(script_dir), os.pardir, "worlds")
        world_file = os.path.join(worlds_dir, "Greedy.wbt")
        
        if os.path.exists(world_file):
            print(f"📦 启动世界文件: {world_file}")
            # 在 macOS 上使用 open 命令启动 Webots
            if platform.system() == 'Darwin':
                subprocess.call(['open', '-a', 'Webots', world_file])
            else:
                # 其他系统尝试直接运行 webots 命令
                subprocess.call(['webots', world_file])
        else:
            print(f"⚠️  世界文件不存在: {world_file}")
            print("   请手动在 Webots 中打开世界文件并启动模拟")
            
            # 保持网页服务器运行
            print("\n🌐 网页服务器继续运行中...")
            try:
                web_process.wait()
            except KeyboardInterrupt:
                print("\n⏹️  停止中...")
                web_process.terminate()
                web_process.wait()
    
    except KeyboardInterrupt:
        print("\n\n⏹️  用户中断，清理中...")
        web_process.terminate()
        try:
            web_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            web_process.kill()
        print("✅ 已关闭网页服务器")
        sys.exit(0)
    except Exception as e:
        print(f"❌ 启动失败: {e}")
        web_process.terminate()
        sys.exit(1)

if __name__ == '__main__':
    main()
