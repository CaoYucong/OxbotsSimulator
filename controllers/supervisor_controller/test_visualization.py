#!/usr/bin/env python3
"""
测试脚本：验证网页服务器是否能正确读取数据文件
"""

import os
import sys

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(__file__))

from web_server import (
    read_ball_positions, 
    read_robot_position,
    read_webots_time,
    read_waypoint_status
)

print("=" * 60)
print("🧪 Unibots 可视化数据读取测试")
print("=" * 60)
print()

# 测试各个数据读取函数
print("[1/4] 测试球位置读取...")
balls = read_ball_positions()
print(f"    ✅ 读到 {len(balls)} 个球")
if balls:
    print(f"    示例: {balls[0]}")
print()

print("[2/4] 测试机器人位置读取...")
robot = read_robot_position()
print(f"    ✅ 机器人位置: ({robot['x']:.3f}, {robot['y']:.3f})")
print()

print("[3/4] 测试模拟时间读取...")
time = read_webots_time()
print(f"    ✅ 模拟时间: {time:.2f} 秒")
print()

print("[4/4] 测试机器人状态读取...")
status = read_waypoint_status()
print(f"    ✅ 机器人状态: {status}")
print()

print("=" * 60)
print("✅ 所有测试完成！")
print("=" * 60)
print()
print("📝 提示:")
print("   • 如果上面显示 0 个球或默认位置，说明模拟器还未生成数据")
print("   • 启动 Webots 模拟器后，这些值会自动更新")
print("   • 现在可以启动网页服务器了！")
print()
print("🚀 启动网页服务器:")
print("   python3 web_server.py")
print()
print("   然后访问: http://localhost:5000")
print()
