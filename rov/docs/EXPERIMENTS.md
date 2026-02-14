# 实验复现与结果说明

## 1. 实验目标

验证以下能力：
- 复杂障碍与时变海流下的 3D 全局路径规划
- 6 推进器 ROV 在 4DOF 模型下的闭环路径跟踪
- 多图输出，清晰展示规划与控制全过程

---

## 2. 复现步骤

### 安装依赖

```bash
python3 -m pip install -r requirements.txt
```

### 运行 ROV 仿真

```bash
python3 run_rov_planning.py
```

### 运行测试

```bash
python3 -m pytest -q
```

---

## 3. 结果文件

目录：`output/rov/`

- `result.txt`：关键数值结果
- `01_topdown_path.png`
- `02_3d_tracking.png`
- `03_state_timeseries.png`
- `04_thruster_commands.png`
- `05_ocean_current_field.png`

打包文件：`output/rov_outputs.zip`

---

## 4. 关键指标建议

建议关注：
- `Reached goal`
- `Final distance error`
- `Control effort integral`
- 跟踪误差曲线是否单调收敛或振荡

---

## 5. 参数扫描建议

可以做三组对比实验：

1. **流场强度扫描**：增大/减小 `current()` 振幅
2. **控制器增益扫描**：`kp_pos`, `kd_vel`, `kp_yaw`, `kd_r`
3. **规划复杂度扫描**：障碍密度、起终点距离

输出建议：
- 每组实验单独保存到 `output/exp_xxx/`
- 统一记录 CSV：误差、能耗、到达时间

---

## 6. 已知限制

- 当前模型为简化 4DOF，不含完整 6DOF 水动力项
- 当前规划为静态 A* + 跟踪控制，未做动态障碍实时重规划
- 当前未引入状态估计器与传感器噪声闭环
