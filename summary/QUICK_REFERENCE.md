# FUEL 项目一页纸快速参考

## 项目概览
**FUEL**: Fast UAV Exploration & Localization  
**总包数**: 20个ROS包  
**语言**: C++/ROS  
**核心**: 无人机自主规划、控制、仿真系统  

## 20个包分类

### fuel_planner (规划层) - 9个包
```
  plan_manage         ←─ 核心：FSM规划管理器 (KAstar/Topo切换)
      ↓
  ├─ plan_env        ← SDF/EDT环境、碰撞检测
  ├─ path_searching  ← KAstar、TopologyPRM路径搜索
  ├─ bspline_opt     ← B样条轨迹优化
  ├─ bspline         ← B样条表示和评估
  ├─ poly_traj       ← 多项式轨迹
  ├─ traj_utils      ← 可视化工具
  ├─ active_perception ← 前沿点检测、视觉感知
  └─ exploration_manager ← 探索任务 + TSP求解
```

### uav_simulator (控制仿真层) - 11个包
```
  traj_server        ← 订阅bspline → 实时计算pos_cmd
      ↓
  so3_control        ← PID + SO(3)控制 → so3_cmd
      ↓
  so3_quadrotor_simulator ← 四旋翼动力学 → Odometry
      ↓
  local_sensing      ← 光线投射 → 深度/点云反馈
  map_generator      ← 环境生成
  
  [基础库] quadrotor_msgs, uav_utils, cmake_utils
```

## 核心数据流 (闭环)

```
1. odometry          3. /planning/bspline      5. /so3_cmd            7. /visual_slam/odom
   (起点)     →      B样条轨迹        →      姿态控制     →      新位置反馈
        ↑                                                              ↓
        └──── 8. /map_ros/cloud (感知反馈) ────────────────────────┘
```

## ROS话题一览

| 话题 | 类型 | 方向 | 频率 |
|------|------|------|------|
| /state_ukf/odom | Odometry | → 规划 | 20Hz |
| /map_ros/cloud | PointCloud2 | → 规划 | 20Hz |
| /goal_point | PointStamped | → 规划 | 手动 |
| **/planning/bspline** | Bspline | 规划 → 执行 | 1-2Hz |
| **/planning/pos_cmd** | PositionCommand | 执行 → 控制 | 50Hz |
| **/so3_cmd** | SO3Command | 控制 → 仿真 | 100Hz |
| /visual_slam/odom | Odometry | 仿真 → 所有 | 200Hz |

**加粗** = 关键话题

## 规划管道 (1-2 Hz)

```
①输入: 当前状态 + 目标
  ↓
②环境查询: SDF查询 (plan_env)
  ↓
③路径搜索: KAstar (0.2-0.5s) 或 TopologyPRM
  ↓
④轨迹优化: BsplineOptimizer (梯度下降, 20-100次迭代)
  ↓
⑤发布: /planning/bspline
  ↓
⑥执行: traj_server 逐帧评估 → pos_cmd (50Hz)
```

## 关键算法

| 算法 | 位置 | 输入 | 输出 |
|------|------|------|------|
| KinodynamicAstar | path_searching | start, goal, vel | 路径点序列 |
| TopologyPRM | path_searching | start, goal | 多条拓扑路径 |
| BsplineOptimizer | bspline_opt | 路径 + 约束 | 优化轨迹 |
| SO(3)Control | so3_control | pos_cmd + odom | so3_cmd |
| Quadrotor Dynamics | so3_simulator | so3_cmd | 新odom |

## 参数关键值 (调试用)

```yaml
规划参数:
  max_vel: 2.5 m/s
  max_acc: 2.0 m/s²
  collision_margin: 0.15-0.2m
  plan_horizon: 20s max

控制参数:
  Kp_xyz: [50, 50, 80]
  Kd_xyz: [10, 10, 15]
  Kp_yaw: 30

感知参数:
  resolution: 0.1m (网格大小)
  local_radius: 30m
  occupancy_threshold: 0.80

仿真参数:
  gravity: 9.8 m/s²
  quadrotor_mass: 1.0 kg (可调)
```

## 常见问题快速查找

| 问题 | 查看 |
|------|------|
| 规划为什么那么慢 | path_searching 搜索参数, bspline_opt迭代数 |
| 轨迹老是和障碍物碰撞 | plan_env的margin/distance_threshold |
| 控制不稳定 | so3_control的Kp/Kd增益 |
| 点云重影 | local_sensing的光线投射参数 |
| 重规划频繁 | plan_manage的碰撞检测阈值 |
| 无人机不听指令 | 检查/planning/pos_cmd是否发送 |

## 调试建议

1. **可视化查看**
   - RViz 订阅: /planning/trajectory, /planning/path, /planning/goal
   - 查看是否有规划结果

2. **查看话题频率和内容**
   ```bash
   rostopic hz /planning/bspline
   rosbag record -O debug.bag <topics>
   ```

3. **修改规划参数**
   - 位置: `src/FUEL/fuel_planner/plan_manage/include/plan_manage.h`
   - 重新编译: `catkin_make -j4`

4. **单步调试规划**
   - 在 plan_manage 的 kinodynamicReplan() 处设断点
   - 查看路径搜索和轨迹优化的中间结果

5. **检查碰撞**
   ```cpp
   plan_env_->checkTrajCollision(traj, dist);
   // 如果dist < margin，则碰撞
   ```

## 文件结构速览

```
src/FUEL/
├── fuel_planner/
│   ├── plan_manage/          ← main loop here
│   ├── plan_env/             ← SDF + collision detection
│   ├── path_searching/       ← A* + PRM
│   ├── bspline_opt/          ← trajectory optimization
│   ├── active_perception/    ← frontier + visual planning
│   └── ...
├── uav_simulator/
│   ├── so3_quadrotor_simulator/  ← physics engine
│   ├── so3_control/              ← attitude controller
│   ├── local_sensing/            ← depth renderer
│   ├── map_generator/            ← scenario creator
│   └── Utils/                    ← msgs, utils, etc.
└── README.md (项目说明)
```

## 模块间最常见的调用

```cpp
plan_manage
  ├─ plan_env.evaluateEDTWithGrad()        // 查询距离梯度
  ├─ path_searching.kinodynamicAstar()    // 搜索路径
  └─ bspline_opt.optimize()               // 优化轨迹

bspline_opt
  ├─ plan_env.evaluateEDTWithGrad()       // 约束检测
  └─ bspline.evaluateDeBoor()             // 轨迹评估

traj_server
  ├─ bspline.evaluateDeBoor()             // 位置
  ├─ bspline.evaluateDerivative()         // 速度
  └─ bspline.evaluateSecond()             // 加速度

so3_control
  ├─ [订阅] /planning/pos_cmd
  └─ [发布] /so3_cmd

so3_simulator
  ├─ [订阅] /so3_cmd
  └─ [发布] /visual_slam/odom
```

## 性能指标 (参考)

| 指标 | 值 |
|------|-----|
| 规划时间 | 0.2-0.5s (KAstar) |
| 轨迹优化 | 0.1-0.3s |
| 轨迹跟踪频率 | 50 Hz |
| 控制频率 | 100 Hz |
| 仿真频率 | 200 Hz |
| 感知处理 | 20 Hz |
| **环路频率** | **1-2 Hz** (规划主循环) |

## 完整系统启动

```bash
# 终端1: 启动仿真
roslaunch FUEL <launch_file>.launch

# 终端2: 查看RViz
rviz -d <config>.rviz

# 终端3: 发送目标点
rostopic pub /goal_point geometry_msgs/PointStamped \
  "header: {frame_id: 'map'} point: {x: 10, y: 0, z: 2}"
```

---

**快速参考版本**: v1.0  
**最后更新**: 2026-03-23  
**配套文档**: 
- ARCHITECTURE_INDEX.md (导航)
- PROJECT_ARCHITECTURE.md (详细架构)
- ARCHITECTURE_DIAGRAMS.md (可视化图表)
- MODULE_INTERACTIONS.md (交互细节)
