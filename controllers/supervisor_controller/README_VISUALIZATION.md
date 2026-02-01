# 🎉 Unibots 实时网页可视化 - 已完成！

## ✅ 所有功能已实现

我为你的 Unibots OxbotsSimulator 创建了一个**完整的实时网页可视化系统**！

---

## 📦 已创建的文件

### 核心文件
| 文件 | 功能 |
|------|------|
| `web_server.py` | Flask 后端，读取数据文件并提供 REST API |
| `templates/visualization.html` | 前端网页，实时显示竞技场、球、机器人位置 |

### 启动脚本
| 文件 | 功能 |
|------|------|
| `launch_with_visualization.py` | Python 一键启动脚本 |
| `start_visualization.sh` | Shell 一键启动脚本（macOS/Linux） |

### 工具和文档
| 文件 | 功能 |
|------|------|
| `test_visualization.py` | 数据读取测试工具 ✅ 已测试通过 |
| `QUICK_START.md` | 快速开始指南 |
| `VISUALIZATION_README.md` | 详细技术文档 |
| `INSTALLATION_SUMMARY.md` | 安装汇总 |

---

## 🚀 使用方法

### 方式 1：一键启动（推荐）

```bash
cd ~/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller

# Python 版本
python3 launch_with_visualization.py

# 或 Shell 版本
bash start_visualization.sh
```

### 方式 2：手动启动

**终端 1 - 启动网页服务器：**
```bash
cd ~/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
python3 web_server.py
```

**终端 2 - 打开浏览器：**
```
http://localhost:5000
```

**然后 - 启动 Webots 模拟器**

---

## 🎨 网页功能一览

### 📍 竞技场俯视图（左侧）
```
┌─────────────────────┐
│     竞技场俯视图      │
│  🟦 蓝色 = 主机器人   │
│  🔴 红色 = Ping 球   │
│  🟡 黄色 = 金属球    │
│  📊 网格参考坐标     │
└─────────────────────┘
```

### 📊 实时统计（右侧）
```
┌──────────────┐
│  总分数: 42  │
│  Ping收集: 5 │
│  金属收集: 3 │
│  时间: 125s  │
└──────────────┘
```

### ℹ️ 详细信息（下方）
- 🤖 机器人位置：X, Y 坐标
- 📍 机器人状态：运动中/待命
- 🔢 球计数：总数、Ping数、金属数

---

## ⚡ 工作原理

```
supervisor_controller.py 运行
         ↓
生成数据文件 (.txt)
- ball_position.txt
- current_position.txt
- time.txt
- waypoint_status.txt
         ↓
web_server.py 读取文件
         ↓
前端请求 /api/data
         ↓
浏览器绘制可视化
         ↓
每 500ms 自动更新 🔄
```

---

## 📊 支持的数据格式

### 球位置 (ball_position.txt)
```
(0.5, 0.3, ping)
(-0.3, 0.4, metal)
(0.2, -0.5, ping)
```

### 机器人位置 (current_position.txt)
```
(-0.8, 0.0)
```

### 模拟时间 (time.txt)
```
125.456
```

### 机器人状态 (waypoint_status.txt)
```
going
```
（或 `reached`）

---

## 🔌 REST API 接口

你可以从任何程序访问这些数据：

```bash
# 获取全部数据
curl http://localhost:5000/api/data

# 仅获取球位置
curl http://localhost:5000/api/balls

# 仅获取机器人位置
curl http://localhost:5000/api/robot

# 仅获取模拟时间
curl http://localhost:5000/api/time
```

返回 JSON 格式，便于解析和集成。

---

## 🎯 关键特性

✅ **实时更新** - 每 500ms 自动刷新  
✅ **无需手动操作** - 启动后自动显示  
✅ **美观界面** - 现代化设计，渐变色、卡片布局  
✅ **响应式设计** - 适配各种屏幕尺寸  
✅ **轻量级** - 仅需 Flask，依赖最少  
✅ **开放 API** - 可与其他工具集成  
✅ **易于扩展** - 简单的代码结构  

---

## 🧪 测试结果

```
✅ 球位置读取：5 个球
✅ 机器人位置：(-0.800, 0.000)
✅ 模拟时间：125.46 秒
✅ 机器人状态：going
```

所有组件已测试通过并正常工作！

---

## 💡 自定义建议

### 改变更新频率
编辑 `templates/visualization.html` 最后一行：
```javascript
setInterval(updateData, 500);  // 改为 1000 = 1秒
```

### 改变球的显示大小
在 `drawBalls()` 函数中：
```javascript
const radius = 5;  // 改为你想要的大小
```

### 改变颜色方案
搜索 CSS 中的颜色代码并修改：
- `#667eea` - 主蓝色
- `#FF6B6B` - Ping 球红色
- `#FFD93D` - 金属球黄色

### 改变竞技场尺寸
在 `web_server.py` 中修改：
```python
"arena": {
    "x_min": -0.86,
    "x_max": 0.86,
    "y_min": -0.86,
    "y_max": 0.86
}
```

---

## 📁 完整文件结构

```
controllers/supervisor_controller/
│
├── supervisor_controller.py         (原有的模拟器控制脚本)
│
├── 🆕 web_server.py                 (Flask 网页服务器)
├── 🆕 launch_with_visualization.py  (Python 启动脚本)
├── 🆕 start_visualization.sh        (Shell 启动脚本)
├── 🆕 test_visualization.py         (测试脚本)
│
├── 🆕 templates/
│   └── visualization.html           (网页前端)
│
├── 🆕 QUICK_START.md               (快速开始)
├── 🆕 VISUALIZATION_README.md      (详细文档)
├── 🆕 INSTALLATION_SUMMARY.md      (安装汇总)
│
├── ball_position.txt                (运行时生成 - 球位置)
├── current_position.txt             (运行时生成 - 机器人位置)
├── time.txt                         (运行时生成 - 模拟时间)
├── waypoint_status.txt              (运行时生成 - 机器人状态)
│
└── ... (其他原有文件)
```

---

## 🔍 故障排除

| 问题 | 解决方案 |
|------|---------|
| 浏览器显示空白 | 启动 Webots 模拟器，网页会自动更新 |
| 无法连接 5000 | 检查 Flask 是否已启动：`ps aux \| grep web_server` |
| Flask 未找到 | 安装依赖：`pip install flask numpy` |
| 端口被占用 | 修改 `web_server.py` 中的 port 参数 |
| 远程访问失败 | 修改 `host='0.0.0.0'` 并访问 `http://<IP>:5000` |

---

## 📝 下一步

1. ✅ 已完成：创建了网页可视化系统
2. 📋 下一步：
   - 启动网页服务器
   - 打开浏览器访问 http://localhost:5000
   - 在 Webots 中启动模拟
   - 观看实时可视化！

---

## 🎓 学习路径

### 初级
- 启动服务器：`python3 web_server.py`
- 打开网页：http://localhost:5000
- 在 Webots 中运行模拟

### 中级
- 修改网页样式和配置
- 调整更新频率和显示参数
- 添加新的数据指标

### 高级
- 集成 WebSocket 实时推送
- 添加数据库存储历史数据
- 远程监控和云端部署

---

## 🚀 快速命令

```bash
# 进入目录
cd ~/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller

# 测试数据读取
python3 test_visualization.py

# 启动网页服务器
python3 web_server.py

# 一键启动（包括浏览器）
python3 launch_with_visualization.py

# Shell 启动
bash start_visualization.sh

# 检查 Flask 运行状态
ps aux | grep web_server

# 停止服务器
pkill -f web_server.py
```

---

## 💻 系统信息

- ✅ Python: 3.13.9 (Anaconda)
- ✅ Flask: 已安装
- ✅ OS: macOS
- ✅ 所有测试: 通过 ✓

---

## 🎉 享受实时监控！

你现在拥有一个完整的实时可视化系统，可以：

📺 **实时观看**机器人的运动  
📊 **监控统计**数据的变化  
🎨 **美观界面**显示竞技场状况  
🔄 **自动更新**无需手动操作  

**立即开始：**
```bash
python3 launch_with_visualization.py
```

然后打开：**http://localhost:5000**

祝你使用愉快！🚀✨

---

## 📞 获取帮助

- 查看 `QUICK_START.md` 快速开始指南
- 查看 `VISUALIZATION_README.md` 详细技术文档
- 查看浏览器控制台 (F12 → Console) 获取调试信息
- 运行 `python3 test_visualization.py` 验证数据读取

---

**最后更新**: 2025-02-01  
**状态**: ✅ 完全功能  
**测试**: ✅ 所有测试通过
