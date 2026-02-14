# ROV 路径规划项目

该仓库仅包含 ROV 路径规划与控制仿真代码。

## 功能
- 3D 体素环境建图（障碍 + 时变海流）
- 3D A* 全局路径规划
- 6 推进器 ROV（4DOF: x, y, z, yaw）简化动力学
- 路径跟踪控制与推进器分配
- 多图结果输出（轨迹、状态、推进器、流场）

## 结构
```text
rov/
├── src/
│   ├── env/
│   ├── planner/
│   ├── dynamics/
│   ├── controller/
│   └── viz/
├── scripts/
│   └── run_rov_planning.py
├── tests/
└── docs/
```

## 安装
```bash
python3 -m pip install -r requirements.txt
```

## 运行
```bash
python3 rov/scripts/run_rov_planning.py
```

输出目录：`output/rov/`

## 测试
```bash
python3 -m pytest -q rov/tests/test_rov_planner.py
```

## 文档
- `rov/docs/ROV_MODEL.md`
- `rov/docs/PLANNING.md`
- `rov/docs/EXPERIMENTS.md`
