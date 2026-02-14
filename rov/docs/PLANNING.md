# 路径规划与控制流程说明

## 1. 总体架构

采用两层结构：

1. **全局规划层**：3D A* 在体素地图中生成离散路径
2. **局部控制层**：跟踪路径点并输出广义力，映射到 6 推进器命令

该结构计算代价可控，便于调试和工程迭代。

---

## 2. 环境建模

环境由 `OceanEnv` 管理：
- 网格大小：默认 `50 x 50 x 20`
- 障碍：盒体障碍写入 occupancy
- 海流：时变三维函数 `current(x,y,z,t)`

代码位置：`env/ocean_env.py`

---

## 3. 全局规划（3D A*）

输入：
- 起点栅格 `start`
- 终点栅格 `goal`
- 可通行 occupancy

特性：
- 26 邻接扩展
- 欧氏距离启发
- 输出离散路径（waypoints）

代码位置：`planner/grid_planner.py`

---

## 4. 路径跟踪控制

控制器逻辑：
- 选择 lookahead 目标点
- 计算世界系位置误差
- 转换到体坐标误差
- `PD` 生成 `Fx,Fy,Fz`
- 按目标航向生成 `Mz`
- 通过分配矩阵得到 6 推进器指令

代码位置：`controller/path_tracker.py`

---

## 5. 运行入口

主流程入口：`run_rov_planning.py`

流程：
1. 创建环境并加载障碍
2. 调用 3D A* 求路径
3. 执行仿真循环（控制 + 动力学 + 海流）
4. 输出结果文本与多张图

---

## 6. 输出图解释

- `01_topdown_path.png`：俯视障碍 + 规划路径 + 实际轨迹
- `02_3d_tracking.png`：三维路径与跟踪对比
- `03_state_timeseries.png`：位置/姿态/速度/误差随时间
- `04_thruster_commands.png`：6 推进器命令变化
- `05_ocean_current_field.png`：流场采样矢量图

---

## 7. 参数调节建议

若跟踪误差大：
- 增大 `kp_pos`
- 适度增大 `kd_vel`

若航向振荡：
- 降低 `kp_yaw`
- 增大 `kd_r`

若推进器频繁饱和：
- 降低 `kp_pos/kp_yaw`
- 提高 lookahead
- 调整路径平滑性
