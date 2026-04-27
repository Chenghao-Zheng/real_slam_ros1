# 终点朝向约束功能说明

## 概述

该功能扩展了 PID 局部规划器，使其支持**终点朝向控制**。小车不仅能到达目标位置，还能在到达后继续旋转，直到满足指定的目标朝向。

---

## 工作原理

### 传统行为（修改前）
1. 小车沿路径跟踪，向目标点前进
2. **位置到达后，立即停止**，不考虑朝向
3. 即使 RViz 中用户指定了朝向，小车也不会执行该朝向约束

### 新行为（修改后）
1. 小车沿路径跟踪，向目标点前进
2. **位置到达后，继续旋转**直到朝向满足要求
3. 只有位置和朝向都满足时，小车才停止
4. 支持通过参数启用/禁用该功能

---

## 使用方法

### 1. 在 RViz 中设置目标

- 点击工具栏的 **"2D Nav Goal"** 按钮
- **点击**要去的位置
- **拖动鼠标**设置小车的目标朝向（箭头指向的方向）

### 2. 参数配置

编辑 [omnidirectional_pid_param.yaml](config/omnidirectional_pid_param.yaml)：

```yaml
# ===== 【新增】终点朝向约束 =====
# 是否启用终点朝向控制（true：到达终点后继续旋转直到满足目标朝向）
ENABLE_FINAL_YAW: true

# 朝向容差（弧度）：小车朝向误差在此范围内认为已满足
# 0.05 rad ≈ 2.9 度；0.1 rad ≈ 5.7 度；建议 0.05~0.1
YAW_TOL: 0.05
```

#### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_FINAL_YAW` | `true` | 启用终点朝向约束 |
| `YAW_TOL` | `0.05` | 朝向容差（弧度）<br>0.05 ≈ 2.9°<br>0.10 ≈ 5.7°<br>0.15 ≈ 8.6° |

### 3. 启用和禁用

**禁用终点朝向约束（恢复原来的行为）**：
```yaml
ENABLE_FINAL_YAW: false
```

此时小车到达位置后立即停止，不再进行朝向调整。

---

## 控制流程图

```
1. 小车收到导航目标
   ├─ 目标位置：(x_goal, y_goal)
   └─ 目标朝向：theta_goal（来自RViz的2D Nav Goal）

2. 小车运动阶段
   └─ 沿全局路径跟踪，控制 (vx, vy, wz)

3. 位置到达判定
   └─ 如果 distance(小车, 目标) < GOAL_TOL：进入步骤4
   └─ 否则：继续步骤2

4. 朝向调整阶段（NEW）
   ├─ 如果 ENABLE_FINAL_YAW == false：停止运动，任务完成
   └─ 如果 ENABLE_FINAL_YAW == true：
      ├─ 计算朝向误差：e_yaw = wrap_to_pi(theta_goal - theta_current)
      ├─ 如果 |e_yaw| < YAW_TOL：停止运动，任务完成
      └─ 否则：输出 wz（旋转命令），继续步骤4

5. 任务完成
   └─ 小车处于 IDLE 状态，位置和朝向都满足目标
```

---

## 代码修改详情

### 新增方法

```cpp
// 检查朝向是否到达（误差在指定范围内）
bool isYawReached(double yaw_target) const
{
  const double e_yaw = wrap_to_pi(yaw_target - yaw_);
  return std::fabs(e_yaw) < yaw_tol_;
}

// 从路径末尾pose提取目标朝向
double getPathEndYaw() const
{
  if (!has_path_ || path_.poses.empty()) return yaw_;
  const auto& last_pose = path_.poses.back().pose;
  return tf::getYaw(last_pose.orientation);
}
```

### 修改 `onTimer()` 函数

在位置到达后，新增朝向检查逻辑：
- 如果启用了 `ENABLE_FINAL_YAW`，则继续执行朝向控制
- 只输出 `wz` 命令（角速度），`vx` 和 `vy` 设为 0
- 直到朝向误差小于 `YAW_TOL`

---

## 调试和验证

### 1. 检查话题和 TF

```bash
# 查看导航目标
rostopic echo /move_base_simple/goal

# 查看小车当前朝向
rosrun tf tf_echo map base_link

# 查看路径中末点的朝向
rostopic echo /opt_path | grep -A 5 "poses:" | tail -20
```

### 2. 查看日志

```bash
# 启用终点朝向时的日志
roslaunch robot_pid_local_planner Omnidirectional_PID.launch
# 输出：
# [robot_pid_local_planner] reached goal (position & yaw) -> IDLE (latched).

# 禁用终点朝向时的日志
# [robot_pid_local_planner] REACHED & LATCHED IDLE. dist=0.003
```

### 3. 参数调整建议

- **朝向容差太严格**（例如 0.01）：可能导致小车长时间旋转
- **朝向容差太宽松**（例如 0.3）：可能无法满足用户期望
- **建议范围**：0.05 ~ 0.15 弧度（3° ~ 9°）

---

## 常见问题

### Q1: 小车到达目标后不旋转，怎么办？

**检查清单**：
1. 确认 `ENABLE_FINAL_YAW: true` 在参数文件中
2. 确认重新编译：`catkin_make --pkg robot_pid_local_planner`
3. 确认 RViz 中的 2D Nav Goal 有朝向箭头（拖动鼠标设置）
4. 检查路径末点的 orientation 是否正确（`rostopic echo /opt_path`）

### Q2: 小车旋转速度太快/太慢？

调整以下参数：
```yaml
KP_YAW: 1.2      # 增大使旋转更快
MAX_WZ: 0.80     # 限制最大角速度
YAW_TOL: 0.05    # 减小使容差更严格（旋转时间更长）
```

### Q3: 与现有的 `IDLE_LATCH_ON_REACHED` 有什么关系？

- `IDLE_LATCH_ON_REACHED: true`：到达目标后进入锁定状态
- `ENABLE_FINAL_YAW: true`：在进入锁定前，确保朝向也满足
- 两者配合使用，确保位置和朝向都到位后才真正停止

---

## 相关文件修改

1. **cpp 文件**：[src/omnidirectional_pid_local_planner_node.cpp](src/omnidirectional_pid_local_planner_node.cpp)
   - 新增 `isYawReached()` 和 `getPathEndYaw()` 方法
   - 修改 `onTimer()` 核心逻辑
   - 添加新成员变量 `yaw_tol_` 和 `enable_final_yaw_`

2. **参数文件**：[config/omnidirectional_pid_param.yaml](config/omnidirectional_pid_param.yaml)
   - 新增 `ENABLE_FINAL_YAW` 参数
   - 新增 `YAW_TOL` 参数

---

## 总结

这个功能使小车导航系统更加完整：
- ✅ 到达目标位置
- ✅ 到达目标朝向（NEW）
- ✅ 支持灵活启用/禁用
- ✅ 参数可调，便于调试
