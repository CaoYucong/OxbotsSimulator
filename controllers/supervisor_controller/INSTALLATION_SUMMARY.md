# ✨ Unibots 网页可视化系统 - 安装完成

## 🎉 已为你创建的内容

我已成功为你的 Unibots 模拟器创建了一个完整的**实时网页可视化系统**！

## 📦 新增文件列表

| 文件 | 说明 |
|------|------|
| `web_server.py` | Flask 后端服务器，读取数据文件并提供 API |
| `templates/visualization.html` | 前端网页，实时显示竞技场、球、机器人 |
| `launch_with_visualization.py` | 一键启动脚本（Python 版本）|
| `start_visualization.sh` | 一键启动脚本（Shell 版本，macOS/Linux）|
| `test_visualization.py` | 测试脚本，验证数据读取 |
| `VISUALIZATION_README.md` | 详细技术文档 |
| `QUICK_START.md` | 快速开始指南 |
| `INSTALLATION_SUMMARY.md` | 本文件 |

## 🚀 立即开始使用

### 步骤 1️⃣ - 安装 Flask
```bash
pip install flask numpy
```

### 步骤 2️⃣ - 启动网页服务器

选择以下任一方式：

**最简单（推荐）：**
```bash
cd /Users/caoyucong/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
python3 launch_with_visualization.py
```

**或者：**
```bash
bash start_visualization.sh
```

**或者：**
```bash
python3 web_server.py
```

### 步骤 3️⃣ - 打开浏览器
```
http://localhost:5000
```

### 步骤 4️⃣ - 启动 Webots 模拟器
在 Webots 中打开世界文件并运行，网页会自动显示实时数据！

## 🎨 网页功能

### 左侧：竞技场俯视图
- 🟦 **蓝色圆点** = 主机器人
- 🔴 **红色圆点** = Ping 球
- 🟡 **黄色圆点** = 金属球
- 实时显示所有对象的位置

### 右侧：实时统计
- 📊 总分数
- 🎯 已收集的球数
- ⏱️ 模拟时间

### 下方：详细信息
- 🤖 机器人坐标
- 📍 机器人状态
- 🔢 球数统计

## 📊 数据流

```
supervisor_controller.py (Webots 模拟)
            ↓
    生成数据文件 (.txt)
            ↓
    web_server.py (Flask)
            ↓
    visualization.html (网页)
            ↓
    浏览器实时显示 ✨
```

## 🔧 工作原理

1. **Webots 模拟器**运行 `supervisor_controller.py`
2. 模拟器每帧生成数据文件：
   - `ball_position.txt` - 所有球的坐标
   - `current_position.txt` - 主机器人的坐标
   - `time.txt` - 模拟时间
   - `waypoint_status.txt` - 机器人状态
3. **Flask 服务器** (`web_server.py`) 读取这些文件
4. **前端网页** 每 500ms 请求一次 API
5. **Canvas 图形**实时绘制所有对象

## 🧪 测试

验证所有配置是否正确：

```bash
python3 test_visualization.py
```

这会检查：
- 能否读取球位置
- 能否读取机器人位置
- 能否读取模拟时间
- 能否读取机器人状态

## 💡 关键特性

✅ **实时更新** - 每 500ms 刷新一次  
✅ **无需手动刷新** - 自动更新，无感知  
✅ **响应式设计** - 适配各种屏幕大小  
✅ **美观界面** - 现代化设计风格  
✅ **轻量级** - 只需 Flask，无复杂依赖  
✅ **易于扩展** - API 接口开放，可自定义  

## 📝 修改数据文件位置

如果需要改变数据文件的读取路径，编辑 `web_server.py`：

```python
DATA_DIR = os.path.dirname(__file__)  # 改为你的路径
```

## 🌐 远程访问

如果需要从其他电脑访问，编辑 `web_server.py` 最后一行：

```python
app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)
```

然后通过 `http://<你的IP>:5000` 访问

## 📋 API 文档

### 获取全部数据
```
GET /api/data
```
返回：所有实时数据（球、机器人、时间、状态等）

### 仅获取球的位置
```
GET /api/balls
```
返回：球的坐标和类型数组

### 仅获取机器人位置
```
GET /api/robot
```
返回：机器人的 X、Y 坐标

### 仅获取模拟时间
```
GET /api/time
```
返回：当前模拟时间（秒）

## ❓ 常见问题

**Q: 浏览器显示空白或无数据？**  
A: 这是正常的！需要启动 Webots 模拟器后才会有数据。

**Q: 能否同时监测多个机器人？**  
A: 可以！修改 `web_server.py` 添加障碍物机器人的读取即可。

**Q: 更新太快或太慢？**  
A: 修改 `visualization.html` 最后的 `setInterval(updateData, 500)` 中的数字。

**Q: 端口 5000 被占用了？**  
A: 修改 `web_server.py` 中的 `port=5000` 为其他端口号。

## 🎯 下一步建议

1. ✅ 启动网页服务器
2. ✅ 打开浏览器访问 http://localhost:5000
3. ✅ 在 Webots 中运行模拟
4. ✅ 观看实时可视化！
5. 💡 可选：自定义颜色、尺寸、更新频率

## 📞 需要帮助？

检查以下文档：
- `QUICK_START.md` - 快速入门
- `VISUALIZATION_README.md` - 详细说明
- 浏览器控制台 (F12) - 调试信息

## 🎨 自定义建议

- **改变颜色**：编辑 `visualization.html` 中的 CSS
- **改变更新频率**：修改 JavaScript 的 `setInterval` 参数
- **添加新指标**：在 `web_server.py` 中添加新的读取函数
- **改变球的大小**：修改 `drawBalls()` 函数中的 `radius`

---

## 📦 完整的文件树

```
controllers/supervisor_controller/
├── supervisor_controller.py              (原有)
├── web_server.py                        ✨ 新增 - Flask 服务器
├── launch_with_visualization.py         ✨ 新增 - 启动脚本
├── start_visualization.sh               ✨ 新增 - Shell 启动
├── test_visualization.py                ✨ 新增 - 测试脚本
├── QUICK_START.md                       ✨ 新增 - 快速开始
├── VISUALIZATION_README.md              ✨ 新增 - 详细文档
├── INSTALLATION_SUMMARY.md              ✨ 新增 - 本文件
├── templates/
│   └── visualization.html               ✨ 新增 - 网页前端
├── ball_position.txt                    (运行时生成)
├── current_position.txt                 (运行时生成)
├── time.txt                             (运行时生成)
├── waypoint_status.txt                  (运行时生成)
└── ... (其他原有文件)
```

---

## ✨ 享受你的新可视化系统！

一切已准备就绪！现在你可以：

🎮 **实时监控**机器人和球的位置  
📊 **观看统计**数据的实时变化  
🎨 **美观的界面**显示竞技场俯视图  
🔄 **自动更新**无需手动刷新  

**立即开始：**
```bash
cd /Users/caoyucong/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
python3 launch_with_visualization.py
```

祝你使用愉快！🚀
