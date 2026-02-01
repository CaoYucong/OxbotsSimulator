# 🎮 Unibots 模拟器实时可视化 - 快速开始指南

## 📋 系统要求

- Python 3.7+
- Flask
- 现代浏览器（Chrome、Safari、Firefox 等）
- Webots 模拟器（已安装）

## 🚀 快速开始（3 步）

### 步骤 1: 安装依赖

```bash
pip install flask numpy
```

### 步骤 2: 启动网页服务器

选择以下其中一种方式启动：

**方式 A：使用 Python 启动脚本（推荐）**
```bash
cd /Users/caoyucong/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
python3 launch_with_visualization.py
```

**方式 B：使用 Shell 脚本**
```bash
cd /Users/caoyucong/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
bash start_visualization.sh
```

**方式 C：手动启动**
```bash
cd /Users/caoyucong/Desktop/Unibots/OxbotsSimulator/controllers/supervisor_controller
python3 web_server.py
```

### 步骤 3: 打开浏览器

访问：**http://localhost:5000**

然后在 Webots 中启动模拟！

---

## 🎯 工作流程

```
Webots 模拟器运行
       ↓
生成数据文件 (ball_position.txt 等)
       ↓
网页服务器读取数据
       ↓
浏览器实时显示可视化
       ↓
每 500ms 刷新一次
```

---

## 📁 文件结构

```
controllers/supervisor_controller/
├── supervisor_controller.py          # 主模拟器控制脚本
├── web_server.py                     # 🆕 Flask 网页服务器
├── launch_with_visualization.py      # 🆕 一键启动脚本 (Python)
├── start_visualization.sh            # 🆕 一键启动脚本 (Shell)
├── test_visualization.py             # 🆕 测试脚本
├── templates/
│   └── visualization.html            # 🆕 网页前端
├── VISUALIZATION_README.md           # 🆕 详细文档
│
├── ball_position.txt                 # 数据文件：球位置
├── current_position.txt              # 数据文件：机器人位置
├── time.txt                          # 数据文件：模拟时间
├── waypoint_status.txt               # 数据文件：机器人状态
└── 其他文件...
```

---

## 🖼️ 网页界面介绍

### 左侧：竞技场俯视图
- **背景**：竞技场边界
- **蓝色圆点**：主机器人（带方向指示）
- **红色圆点**：Ping 球
- **黄色圆点**：金属球
- **灰色网格**：参考坐标

### 右侧：实时统计
- **总分数**：PING×4 + 金属×2 + 金属存储×1
- **Ping 收集**：已存储的 Ping 球数
- **金属收集**：已存储的金属球数
- **模拟时间**：Webots 内部时间（秒）

### 下方：详细信息
- 机器人精确坐标
- 机器人运动状态
- 竞技场内球的数量统计

---

## 🧪 测试

验证所有数据读取是否正常工作：

```bash
python3 test_visualization.py
```

输出示例：
```
[1/4] 测试球位置读取...
    ✅ 读到 0 个球
    
[2/4] 测试机器人位置读取...
    ✅ 机器人位置: (0.000, 0.000)
    
[3/4] 测试模拟时间读取...
    ✅ 模拟时间: 0.00 秒
    
[4/4] 测试机器人状态读取...
    ✅ 机器人状态: idle
```

> **注意**：如果显示 0 个球或默认值，这是正常的。启动 Webots 模拟器后，数据会自动更新。

---

## 🐛 故障排除

### Q: 浏览器显示 "无法连接"

**A: 检查以下事项：**
1. 确认网页服务器已启动
2. 检查是否已运行：`ps aux | grep web_server.py`
3. 确认 Flask 已安装：`pip list | grep Flask`
4. 检查端口 5000 是否被占用：`lsof -i :5000`

### Q: 网页打开但显示为空（无球无机器人）

**A: 这是正常的，需要：**
1. 启动 Webots 模拟器
2. 运行模拟至少 1 个时间步
3. 网页会在 500ms 内自动更新

### Q: 数据不实时更新

**A: 检查：**
1. Webots 模拟器是否正在运行
2. 检查数据文件权限：`ls -la *.txt`
3. 查看浏览器控制台错误：F12 → Console

### Q: Flask 启动失败 "ModuleNotFoundError: No module named 'flask'"

**A: 安装依赖：**
```bash
pip install flask numpy
```

### Q: 启动脚本权限不足

**A: 修复权限：**
```bash
chmod +x launch_with_visualization.py
chmod +x start_visualization.sh
```

---

## 💡 高级用法

### 自定义更新频率

编辑 `templates/visualization.html` 最后一行：
```javascript
setInterval(updateData, 500);  // 改为 1000 = 1秒更新一次
```

### 自定义竞技场显示大小

编辑 `web_server.py` 中的 `read_ball_positions()` 返回的竞技场参数：
```python
"arena": {
    "x_min": -0.86,
    "x_max": 0.86,
    "y_min": -0.86,
    "y_max": 0.86
}
```

### 添加更多数据字段

1. 在 `web_server.py` 中添加读取函数
2. 在 `/api/data` 路由中添加新字段
3. 在 `templates/visualization.html` 中更新显示逻辑

---

## 📊 API 接口

如果需要从其他程序访问数据：

```bash
# 获取所有数据
curl http://localhost:5000/api/data

# 仅获取球的位置
curl http://localhost:5000/api/balls

# 仅获取机器人位置
curl http://localhost:5000/api/robot

# 仅获取模拟时间
curl http://localhost:5000/api/time
```

返回格式（JSON）：
```json
{
  "balls": [
    {"x": 0.5, "y": 0.3, "type": "ping"},
    {"x": -0.2, "y": 0.1, "type": "metal"}
  ],
  "robot": {"x": -0.8, "y": 0.0},
  "time": 125.34,
  "status": "going"
}
```

---

## 🎨 自定义样式

所有样式都在 `templates/visualization.html` 的 `<style>` 中定义，可以修改：
- 颜色：搜索 `#667eea` 等颜色代码
- 字体：搜索 `font-family`
- 布局：修改 CSS Grid 相关代码

---

## 📞 常见问题

**Q: 能否同时监测多个机器人？**
A: 可以！修改 `web_server.py` 添加读取多个机器人位置的函数

**Q: 能否保存历史数据？**
A: 可以！修改 `web_server.py` 添加数据日志功能

**Q: 能否在远程访问？**
A: 可以！修改 `web_server.py` 的 `app.run()` 参数为 `host='0.0.0.0'`

---

## 🚀 优化建议

1. **提高更新频率**：改为 200ms 或 100ms（可能需要优化网页）
2. **添加数据缓存**：避免频繁读取文件
3. **WebSocket 实时推送**：比轮询更高效
4. **数据库存储**：长期运行时保存历史数据

---

## 📝 许可证

此工具为 Unibots OxbotsSimulator 的一部分。

## ✨ 享受实时可视化！

现在你可以实时看到机器人和球的动态了！🎉

有任何问题或建议，欢迎反馈！
