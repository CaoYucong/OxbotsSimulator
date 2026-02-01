# Unibots 模拟器实时可视化

这是一个实时网页可视化工具，用于监测 Webots 模拟器中的机器人和球的位置。

## 功能特性

✨ **实时数据可视化**
- 俯视图显示竞技场、机器人和球的位置
- 不同颜色区分不同类型的球（Ping 球/金属球）
- 实时更新机器人位置和状态

📊 **实时统计**
- 总分数
- 收集的 Ping 球和金属球数量
- 模拟器运行时间
- 竞技场内的球数统计

🔄 **自动更新**
- 每 500ms 更新一次数据
- 无需手动刷新浏览器
- 实时同步模拟器状态

## 安装依赖

```bash
pip install flask numpy
```

## 使用方法

### 方法 1：使用启动脚本（推荐）

```bash
python3 launch_with_visualization.py
```

这个脚本会：
1. 启动网页服务器（后台）
2. 自动打开浏览器到 http://localhost:5000
3. 启动 Webots 模拟器

### 方法 2：手动启动

**终端 1：启动网页服务器**
```bash
python3 web_server.py
```

**终端 2：启动 Webots 模拟器**
```bash
# 使用 Webots GUI 或命令行启动
webots worlds/Arena_Development.wbt
```

然后打开浏览器访问：http://localhost:5000

## 必需的数据文件

模拟器必须生成以下数据文件（由 `supervisor_controller.py` 自动生成）：

- `ball_position.txt` - 球的位置
- `current_position.txt` - 机器人当前位置
- `time.txt` - 模拟时间
- `waypoint_status.txt` - 机器人状态（going/reached）

这些文件会放在 `controllers/supervisor_controller/` 目录中。

## 网页界面

访问 http://localhost:5000 后，你会看到：

### 左侧：竞技场俯视图
- 白色背景 = 竞技场
- 蓝色圆点 = 主机器人
- 红色圆点 = Ping 球
- 黄色圆点 = 金属球
- 灰色圆点 = 障碍物

### 右侧：实时统计
- 总分数
- Ping 球收集数
- 金属球收集数
- 模拟时间

### 下方：详细信息
- 机器人位置 (X, Y 坐标)
- 机器人状态（运动中/待命）
- 竞技场内球数统计

## API 端点

如果需要自定义集成，网页服务器提供以下 API：

```
GET /api/data          - 获取所有数据
GET /api/balls         - 仅获取球的位置
GET /api/robot         - 仅获取机器人位置
GET /api/time          - 仅获取模拟时间
```

示例：
```bash
curl http://localhost:5000/api/data
```

## 故障排除

### 无法连接到网页服务器
- 确认 `web_server.py` 已启动
- 检查 Flask 是否已安装：`pip install flask`
- 检查端口 5000 是否被其他程序占用

### 数据未实时更新
- 确认模拟器正在运行并生成数据文件
- 检查数据文件权限
- 查看浏览器控制台是否有错误信息

### 竞技场显示为空
- 确认模拟器已启动并至少运行了一个时间步
- 检查 `ball_position.txt` 和 `current_position.txt` 是否存在

## 技术栈

- **后端**：Python Flask
- **前端**：HTML5 Canvas + JavaScript
- **通信**：REST API (JSON)

## 自定义选项

### 修改更新频率

在 `visualization.html` 中，修改最后一行的数字（单位：毫秒）：
```javascript
setInterval(updateData, 500);  // 改为想要的值，如 1000 (1秒)
```

### 修改球的显示大小

在 `visualization.html` 中找到 `drawBalls()` 函数，修改 `radius` 值：
```javascript
const radius = 5;  // 改为想要的像素大小
```

### 修改竞技场尺寸

在 `web_server.py` 中修改 `read_ball_positions()` 后的竞技场参数：
```python
"arena": {
    "x_min": -0.86,
    "x_max": 0.86,
    "y_min": -0.86,
    "y_max": 0.86
}
```

## 许可证

此项目作为 Unibots OxbotsSimulator 的一部分。

## 支持

如有问题，请检查：
1. supervisor_controller.py 是否正确生成数据文件
2. 网页服务器的 Flask 调试输出
3. 浏览器开发者工具的网络和控制台选项卡
