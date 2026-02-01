#!/bin/bash

# 一键启动脚本：启动网页可视化 + Webots 模拟器

echo "================================"
echo "🚀 Unibots 模拟器 + 网页可视化"
echo "================================"
echo ""

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "📂 工作目录: $SCRIPT_DIR"
echo ""

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误：未找到 Python 3"
    echo "   请先安装 Python 3 和 Flask"
    exit 1
fi

echo "✅ Python 3 已安装"

# 检查 Flask
if ! python3 -c "import flask" 2>/dev/null; then
    echo "⚠️  Flask 未安装，正在安装..."
    pip install flask numpy
    if [ $? -ne 0 ]; then
        echo "❌ 安装 Flask 失败"
        exit 1
    fi
fi

echo "✅ Flask 已安装"
echo ""

# 启动网页服务器
echo "[1/2] 启动网页服务器..."
python3 web_server.py &
WEB_PID=$!
echo "✅ 网页服务器已启动 (PID: $WEB_PID)"
echo ""

# 等待服务器启动
sleep 2

# 打开浏览器
echo "[2/2] 打开浏览器..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    open http://localhost:5000
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    xdg-open http://localhost:5000 &
else
    echo "⚠️  请手动打开浏览器访问: http://localhost:5000"
fi

echo "✅ 浏览器已打开"
echo ""
echo "================================"
echo "✨ 准备就绪"
echo "================================"
echo ""
echo "📺 网页地址: http://localhost:5000"
echo "🔑 按 Ctrl+C 停止所有服务"
echo ""

# 保持运行直到用户中断
trap "kill $WEB_PID; exit 0" INT
wait

echo ""
echo "⏹️  已停止网页服务器"
exit 0
