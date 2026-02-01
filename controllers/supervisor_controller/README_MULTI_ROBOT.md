# 多机器人控制系统说明

## 概述

重构后的 `supervisor_controller.py` 支持多个机器人的独立控制：

1. **主机器人 (MY_ROBOT)**: 按照 `dynamic_waypoints.txt` 中的单个目标点行动
2. **障碍物机器人 (OBSTACLE_ROBOT)**: 按照 `obstacle_plan.txt` 中的循环路径行动（可选）

## 文件配置

### dynamic_waypoints.txt
主机器人的目标点（单个）

格式示例：
```
(0.5, 0.5, East)
```

或简化格式：
```
(0.5, 0.5)
```

### obstacle_plan.txt
障碍物机器人循环遍历的路径（多个点）

格式示例：
```
(0.0, 0.0, North)
(0.5, 0.0, East)
(0.5, 0.5, South)
(0.0, 0.5, West)
```

机器人会按顺序访问这些点，到达最后一个点后会回到第一个点重新开始。

## 类：MotionController

### 初始化

```python
motion = MotionController(trans_field, rot_field, dt, cycle_mode=False)
```

- `cycle_mode=False`: 单点导航模式（主机器人）
- `cycle_mode=True`: 循环路径模式（障碍物机器人）

### 主要方法

#### start(x, y, velocity=None, angle=None)
启动单个目标点的导航

#### load_waypoint_list(waypoint_list)
加载循环路径列表（仅在 cycle_mode=True 时有效）

#### update()
每帧调用，推进机器人运动
- 返回 True：任务完成或空闲
- 返回 False：任务进行中

## 系统运作流程

```
初始化
  ├─ 主机器人：加载 dynamic_waypoints.txt (单点)
  └─ 障碍物机器人：加载 obstacle_plan.txt (循环路径)

主循环
  ├─ 更新主机器人运动
  ├─ 更新障碍物机器人运动（如果存在）
  ├─ 监测球体吸收
  ├─ 更新文件输出
  └─ 检查主机器人是否到达目标（到达时等待新目标）

循环路径自动行为
  └─ 障碍物机器人到达每个路径点后自动移动到下一个点
     └─ 到达最后一点后回到第一个点
```

## 添加更多机器人

如需添加更多机器人，遵循以下模式：

1. 在 World 文件中定义新机器人的 DEF 名称
2. 在初始化部分获取该机器人的引用
3. 创建对应的 MotionController 实例
4. 根据需求选择 cycle_mode
5. 在主循环中调用 `.update()` 方法

示例：
```python
# 获取机器人引用
patrol_robot = supervisor.getFromDef("PATROL_ROBOT")
if patrol_robot is not None:
    patrol_trans = patrol_robot.getField("translation")
    patrol_rot = patrol_robot.getField("rotation")
    patrol_motion = MotionController(patrol_trans, patrol_rot, dt, cycle_mode=True)
    
    # 加载路径
    patrol_waypoints = _load_waypoint_list_from_file("patrol_plan.txt")
    if patrol_waypoints:
        patrol_motion.load_waypoint_list(patrol_waypoints)

# 在主循环中
if patrol_motion is not None:
    patrol_motion.update()
```

## 关键特性

- ✅ **非阻塞式运动控制**: 所有机器人运动不会阻塞主循环
- ✅ **循环路径支持**: 障碍物机器人自动循环遍历路径
- ✅ **灵活扩展**: 易于添加更多机器人
- ✅ **独立状态管理**: 每个机器人独立管理其运动状态
- ✅ **角度插值**: 支持运动过程中的平滑旋转插值
