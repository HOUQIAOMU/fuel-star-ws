# FUEL 项目架构可视化图表

## 1. 整体系统架构

```
┌────────────────────────────────────────────────────────────────────┐
│                        FUEL ROS 项目                               │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│   ╔══════════════════════════════════════════════════════════╗   │
│   ║         FUEL PLANNER (规划与感知层)                      ║   │
│   ╠══════════════════════════════════════════════════════════╣   │
│   ║                                                          ║   │
│   ║  ┌─────────────────┐  ┌─────────────────┐              ║   │
│   ║  │ exploration_    │  │ active_         │              ║   │
│   ║  │ manager         │  │ perception      │              ║   │
│   ║  │                 │  │                 │              ║   │
│   ║  │ 探索任务        │  │ 前沿探测        │              ║   │
│   ║  │ 管理            │  │ 视觉规划        │              ║   │
│   ║  └────────┬────────┘  └────────┬────────┘              ║   │
│   ║           │                    │                       ║   │
│   ║           └────────┬───────────┘                       ║   │
│   ║                    ↓                                   ║   │
│   ║           ┌────────────────────┐                      ║   │
│   ║           │  plan_manage       │                      ║   │
│   ║           │                    │                      ║   │
│   ║           │ FSM 规划管理器     │                      ║   │
│   ║           │ 运动学/拓扑规划    │                      ║   │
│   ║           └────────┬───────────┘                      ║   │
│   ║                    │                                  ║   │
│   ║      ┌─────────────┼─────────────┐                   ║   │
│   ║      ↓             ↓             ↓                   ║   │
│   ║  ┌────────┐ ┌──────────┐ ┌──────────┐               ║   │
│   ║  │path_   │ │bspline   │ │bspline_  │               ║   │
│   ║  │search  │ │          │ │ opt      │               ║   │
│   ║  │        │ │ 轨迹表示 │ │ 轨迹优化 │               ║   │
│   ║  │Astar   │ │          │ │          │               ║   │
│   ║  │PRM     │ └──────────┘ └──────────┘               ║   │
│   ║  └────────┘      ↑                                   ║   │
│   ║                  │                                   ║   │
│   ║           ┌──────┴───────┐                          ║   │
│   ║           ↓              ↓                          ║   │
│   ║        ┌────────┐   ┌────────┐                     ║   │
│   ║        │plan_   │   │poly_   │                     ║   │
│   ║        │env     │   │traj    │                     ║   │
│   ║        │        │   │        │                     ║   │
│   ║        │碰撞检测│   │多项式  │                     ║   │
│   ║        │SDF/EDT│   │轨迹    │                     ║   │
│   ║        └────────┘   └────────┘                     ║   │
│   ║                                                     ║   │
│   ║           traj_utils (可视化工具)                  ║   │
│   ║                                                     ║   │
│   ╚══════════════════════════════════════════════════════╝   │
│                          ↓ ROS Topics                       │
│   ╔══════════════════════════════════════════════════════╗   │
│   ║         UAV SIMULATOR (仿真与控制层)                ║   │
│   ╠══════════════════════════════════════════════════════╣   │
│   ║                                                      ║   │
│   ║  ┌──────────────┐  ┌──────────────────────┐        ║   │
│   ║  │so3_control   │  │poscmd_2_odom         │        ║   │
│   ║  │              │  │                      │        ║   │
│   ║  │SO(3)姿态     │  │位置指令->里程计      │        ║   │
│   ║  │控制器        │  │命令追踪              │        ║   │
│   ║  └──────┬───────┘  └──────────┬───────────┘        ║   │
│   ║         │                      │                    ║   │
│   ║         └──────────┬───────────┘                    ║   │
│   ║                    ↓                                ║   │
│   ║    ┌────────────────────────────────┐              ║   │
│   ║    │so3_quadrotor_simulator         │              ║   │
│   ║    │                                │              ║   │
│   ║    │四旋翼动力学仿真                │              ║   │
│   ║    │状态集成与约束                  │              ║   │
│   ║    └────────┬───────────────────────┘              ║   │
│   ║             │                                      ║   │
│   ║      ┌──────┴──────┐                              ║   │
│   ║      ↓             ↓                              ║   │
│   ║  ┌─────────┐  ┌──────────────┐                   ║   │
│   ║  │map_     │  │local_        │                   ║   │
│   ║  │generate │  │ sensing      │                   ║   │
│   ║  │         │  │              │                   ║   │
│   ║  │随机生成 │  │深度/点云     │                   ║   │
│   ║  │障碍物   │  │渲染          │                   ║   │
│   ║  └─────────┘  └──────────────┘                   ║   │
│   ║                                                   ║   │
│   ║  so3_disturbance_generator (噪声生成)            ║   │
│   ║                                                   ║   │
│   ╚══════════════════════════════════════════════════╝   │
│                                                            │
│   共享库 (所有模块依赖)                                   │
│   ├─ quadrotor_msgs (ROS消息定义)                        │
│   ├─ uav_utils (通用工具)                                │
│   ├─ cmake_utils (构建工具)                              │
│   ├─ odom_visualization (可视化)                         │
│   └─ waypoint_generator (路径点生成)                     │
│                                                            │
└────────────────────────────────────────────────────────────────────┘
```

## 2. 数据流通路

### 2.1 完整规划-控制环路

```
时刻 t=0
│
├─→ 输入源
│   ├─ 里程计: /state_ukf/odom (nav_msgs::Odometry)
│   │  └─ [位置, 速度, 加速度, 方向]
│   │
│   ├─ 感知: /map_ros/cloud (sensor_msgs::PointCloud2)
│   │  └─ [障碍物点集]
│   │
│   ├─ 目标: /goal_point (geometry_msgs::PointStamped)
│   │  └─ [目标位置]
│   │
│   └─ 参考: /waypoints (nav_msgs::Path)
│      └─ [路径点序列]
│
├─→ 感知处理 (plan_env)
│   ├─ RaycastDepth::rayCasting()
│   │  └─ 深度图 → 点云
│   │
│   ├─ SDFMap::updateOccupancy()
│   │  └─ 更新占有栅格 (概率更新)
│   │
│   └─ EDTEnvironment::buildLocalEDT()
│      └─ 计算欧氏距离变换
│
├─→ 规划 (plan_manage + path_searching)
│   │
│   ├─ IF 首次规划
│   │   └─ FastPlannerManager::kinodynamicReplan()
│   │       ├─ KinodynamicAstar::search(start, goal)
│   │       │  └─ 返回路径: [p0, p1, ..., pn]
│   │       │
│   │       ├─ BsplineOptimizer::optimize(path)
│   │       │  ├─ 将路径转换为B样条控制点
│   │       │  ├─ 梯度下降优化
│   │       │  └─ 返回 NonUniformBspline 轨迹
│   │       │
│   │       └─ 或选择 topoReplan()
│   │           ├─ TopologyPRM::findPath()
│   │           │  └─ 返回多条拓扑不同的路径
│   │           │
│   │           └─ 各路径优化后选择最优
│   │
│   └─ ELSE 需要重规划
│       ├─ 检测：trajectory.checkCollision(map)
│       │
│       └─ 如碰撞：重新执行规划流程
│
├─→ 轨迹执行 (traj_server)
│   ├─ 订阅: /planning/bspline
│   │
│   ├─ 轨迹跟踪
│   │   ├─ t = current_time
│   │   ├─ q(t) = bspline.eval(t)  # 位置
│   │   ├─ v(t) = bspline.evalDerivative(t)  # 速度
│   │   └─ a(t) = bspline.evalSecond(t)  # 加速度
│   │
│   ├─ 位置控制指令生成
│   │   ├─ cmd_pos = q(t)
│   │   ├─ cmd_vel = v(t)
│   │   └─ cmd_acc = a(t)
│   │
│   └─ 发布: /planning/pos_cmd (quadrotor_msgs::PositionCommand)
│
├─→ 姿态控制 (so3_control)
│   ├─ 订阅: /planning/pos_cmd
│   │
│   ├─ PID位置控制
│   │   ├─ error_pos = odom.pos - cmd_pos
│   │   ├─ error_vel = odom.vel - cmd_vel
│   │   └─ thrust = m * (a_d + g + Kp*err_p + Kd*err_v)
│   │
│   ├─ SO(3)姿态计算
│   │   ├─ 从推力方向 → 所需四元数
│   │   └─ 四元数规范化
│   │
│   └─ 发布: /so3_cmd (quadrotor_msgs::SO3Command)
│
├─→ 动力学仿真 (so3_quadrotor_simulator)
│   ├─ 订阅: /so3_cmd
│   │
│   ├─ 状态集成
│   │   ├─ 四元数解析 → 旋转矩阵 R(t)
│   │   ├─ 扭矩计算: τ = I * α (基于四元数误差)
│   │   ├─ 加速度: a = [0,0,-g] + R*thrust/m
│   │   ├─ 数值积分: x(t+dt) = x(t) + v(t)*dt + 0.5*a*dt²
│   │   └─ 速度和位置更新
│   │
│   ├─ 约束处理
│   │   ├─ 最大倾斜角: |pitch|, |roll| < 45°
│   │   ├─ 最大速度: |v| < 5 m/s
│   │   └─ 最大加速度: |a| < 10 m/s²
│   │
│   └─ 发布: /visual_slam/odom (nav_msgs::Odometry)
│
├─→ 感知反馈
│   ├─ local_sensing 渲染深度/点云
│   │   ├─ 从仿真环境和机器人位置
│   │   ├─ 使用GPU/CPU光线投射
│   │   └─ 发布: /map_ros/depth, /map_ros/cloud
│   │
│   └─ 循环到感知处理...
│
└─→ 时刻 t = t + dt （返回）
```

### 2.2 ROS话题通信流图

```
【输入层】
  /state_ukf/odom (Odometry)
  /map_ros/cloud (PointCloud2)
  /goal_point (PointStamped)
        ↓
        ↓
【规划层】
  ┌──────────────────────────────────────────┐
  │  fast_planner_node                       │
  │  ├─ 订阅 odometry, cloud, goal           │
  │  ├─ 更新 plan_env (SDFMap/EDT)           │
  │  ├─ 调用规划算法 (KAstar/Topo)           │
  │  ├─ 生成 B-spline 轨迹                   │
  │  └─ 发布多个话题 ↓                       │
  └──────────────────────────────────────────┘
        ↓
    ┌───┴──────────────────────┬──────────────┬──────┐
    ↓                          ↓              ↓      ↓
/planning/bspline      /planning/replan  /planning/traj /planning/...
(Bspline)              (Empty)           (Marker)
    │
    ↓
【轨迹执行】
┌──────────────────────────────────────────┐
│  traj_server                             │
│  ├─ 订阅 /planning/bspline                │
│  ├─ 实时评估轨迹 (pos, vel, acc)          │
│  └─ 发布 pos_cmd ↓                        │
└──────────────────────────────────────────┘
    │
    ↓
/planning/pos_cmd (PositionCommand)
    │
    ↓
【控制层】
┌──────────────────────────────────────────┐
│  so3_control                             │
│  ├─ 订阅 pos_cmd 和 odom                  │
│  ├─ 计算PID + SO(3) 控制                  │
│  └─ 发布 so3_cmd ↓                        │
└──────────────────────────────────────────┘
    │
    ↓
/so3_cmd (SO3Command)
    │
    ↓
【仿真层】
┌──────────────────────────────────────────┐
│  so3_quadrotor_simulator                 │
│  ├─ 订阅 so3_cmd                          │
│  ├─ 动力学积分                            │
│  └─ 发布 /visual_slam/odom ↓              │
└──────────────────────────────────────────┘
    │
    ↓
/visual_slam/odom (Odometry) → 反馈到规划层
    │
    └─→ local_sensing 渲染点云 → /map_ros/cloud
```

## 3. 包间依赖关系树

```
exploration_manager (应用顶层)
    ├─ plan_manage
    │   ├─ plan_env
    │   │   ├─ quadrotor_msgs (消息定义)
    │   │   └─ uav_utils
    │   │
    │   ├─ path_searching
    │   │   ├─ plan_env
    │   │   └─ traj_utils
    │   │
    │   ├─ bspline
    │   │   └─ traj_utils
    │   │
    │   ├─ bspline_opt
    │   │   ├─ bspline
    │   │   ├─ plan_env
    │   │   └─ traj_utils
    │   │
    │   ├─ poly_traj
    │   │   └─ traj_utils
    │   │
    │   └─ traj_utils
    │       ├─ quadrotor_msgs
    │       └─ uav_utils
    │
    ├─ active_perception
    │   ├─ plan_env
    │   ├─ plan_manage
    │   └─ traj_utils
    │
    └─ lkh_tsp_solver (TSP求解)
        └─ uav_utils

so3_control
    ├─ quadrotor_msgs
    ├─ uav_utils
    └─ so3_quadrotor_simulator (数据结构)

so3_quadrotor_simulator
    ├─ quadrotor_msgs
    ├─ uav_utils
    ├─ so3_disturbance_generator (扰动)
    └─ local_sensing (传感器)

local_sensing
    ├─ quadrotor_msgs
    ├─ uav_utils
    ├─ map_generator (环境)
    ├─ cmake_utils
    └─ pcl (外部库)

map_generator
    ├─ uav_utils
    ├─ cmake_utils
    └─ octomap (外部库)

poscmd_2_odom
    ├─ quadrotor_msgs
    └─ uav_utils

waypoint_generator
    ├─ quadrotor_msgs
    ├─ plan_manage
    └─ traj_utils

odom_visualization
    ├─ quadrotor_msgs
    └─ uav_utils

【基础库】
    quadrotor_msgs       (所有模块)
    uav_utils            (所有模块)
    cmake_utils          (部分模块)
    pose_utils           (某些模块)
    rviz_plugins         (可视化)
```

## 4. 规划算法流程图

### 4.1 运动学规划 (Kinodynamic Planning)

```
KinodynamicReplan()
    ↓
KinodynamicAstar::search()
┌─────────────────────────────────────┐
│ 启发式搜索 (运动基元扩展)            │
│ State = [x, y, z, vx, vy, vz]       │
│ Action = 4 个加速度方向              │
│ Cost = 时间 (time optimal)           │
│ Heuristic = dist / max_vel          │
└──────────┬──────────────────────────┘
           ↓
      返回路径点序列
      [start, ..., end]
           ↓
BsplineOptimizer::optimize()
┌─────────────────────────────────────┐
│ 1. 路径 → B样条控制点                │
│ 2. 约束：                            │
│    - 安全性 (距离 > margin)          │
│    - 速度 (|v| < v_max)              │
│    - 加速度 (|a| < a_max)            │
│ 3. 梯度下降优化                      │
│ 4. 返回优化后的轨迹                  │
└──────────┬──────────────────────────┘
           ↓
      NonUniformBspline
      (可用于执行和重规划检测)
```

### 4.2 拓扑规划 (Topological Planning)

```
TopoReplan()
    ↓
TopologyPRM::findPath()
┌──────────────────────────────────────┐
│ 1. 概率路图构建                      │
│    - 随机采样点                      │
│    - 局部连接性检测                  │
│                                      │
│ 2. 拓扑约束                          │
│    - 路径卷绕数 (winding number)    │
│    - 同伦类 (homotopy class)        │
│                                      │
│ 3. 返回多条路径 (不同拓扑)            │
│    [path1, path2, path3, ...]       │
└──────────┬───────────────────────────┘
           ↓
   对每条路径执行优化
   BsplineOptimizer::optimize()
           ↓
      评估所有轨迹 (成本/安全)
           ↓
      选择最优轨迹
```

## 5. FSM 状态转移图

```
                ┌─────────────────────┐
                │   INIT (初始)        │
                │                     │
                │ - 等待规划器初始化  │
                │ - 等待目标点        │
                └──────────┬──────────┘
                           │
                           ↓
        ┌──────────────────────────────────┐
        │  WAIT_TARGET (等待目标)          │
        │                                  │
        │ - 订阅 /goal_point               │
        │ - 监听切换命令                   │
        └──────────┬───────────────────────┘
                   │ [收到目标点]
                   ↓
        ┌──────────────────────────────────┐
        │  GEN_NEW_TRAJ (规划新轨迹)      │
        │                                  │
        │ - 调用规划算法                   │
        │ - Kinodynamic 或 Topological    │
        │ - 发布 /planning/bspline         │
        └──────────┬───────────────────────┘
                   │ [规划成功]
                   ↓
        ┌──────────────────────────────────┐
        │  EXEC_TRAJ (执行轨迹)            │
        │                                  │
        │ - 轨迹服务器跟踪                 │
        │ - 发送位置指令                   │
        │ - 监听碰撞                       │
        └──────────┬────────┬──────────────┘
                   │        │
           [碰撞]  │        │  [轨迹完成]
                   ↓        ↓
        ┌──────────────────────────────────┐
        │  REPLAN_TRAJ (重规划)            │
        │                                  │
        │ - 从当前状态重新规划             │
        │ - 发布重规划信号                 │
        └──────────┬───────────────────────┘
                   │
                   └─→ GEN_NEW_TRAJ 或 WAIT_TARGET
```

## 6. 参数流向图

```
【系统参数配置】
launch 文件
  ├─ 规划参数
  │  ├─ v_max, a_max, w_max (速度/加速度限制)
  │  ├─ max_tau (最大规划时间)
  │  ├─ max_vel, max_acc (轨迹限制)
  │  └─ margin (安全边界)
  │
  ├─ 控制参数
  │  ├─ Kp, Kd (PID增益)
  │  ├─ mass (质量)
  │  └─ gravity (重力加速度)
  │
  ├─ 感知参数
  │  ├─ resolution (栅格分辨率)
  │  ├─ local_radius (本地地图半径)
  │  └─ p_hit, p_miss (占有概率)
  │
  └─ 仿真参数
     ├─ simulate_params (模拟噪声)
     ├─ simulator_time_ratio (时间缩放)
     └─ disturbance_level (扰动等级)
        ↓
   ┌─────────────────────────────────┐
   │  PlanningParameters             │
   │  ControlParameters              │
   │  SensingParameters              │
   │  SimulationParameters           │
   └─────────────────────────────────┘
        ↓
   加载到各模块：
   ├─ plan_manage (规划参数)
   ├─ so3_control (控制参数)
   ├─ plan_env (感知参数)
   └─ so3_quadrotor_simulator (仿真参数)
```

## 7. 关键接口总结

| 模块 | 关键类/函数 | 输入 | 输出 |
|------|-----------|------|------|
| **plan_manage** | `kinodynamicReplan()` | start, goal, vel, acc | bool, Bspline |
| | `topoReplan()` | start, goal | bool, vector<Bspline> |
| | `planYaw()` | start_yaw, yaw_path | bool |
| **plan_env** | `evaluateEDTWithGrad()` | pos, time | dist, gradient |
| | `checkTrajCollision()` | trajectory | bool |
| **path_searching** | `KinodynamicAstar::search()` | start, goal, vel | vector<Vec3d> |
| | `TopologyPRM::findPath()` | start, goal | vector<Path> |
| **bspline** | `evaluateDeBoor()` | t | Vec3d |
| | `evaluateDerivative()` | t | Vec3d |
| **bspline_opt** | `optimize()` | control_pts, guide_path | Bspline |
| **traj_server** | 订阅 /planning/bspline | Bspline | /planning/pos_cmd |
| **so3_control** | 订阅 /planning/pos_cmd | PositionCommand | /so3_cmd |
| **so3_simulator** | 订阅 /so3_cmd | SO3Command | /visual_slam/odom |

---

**更新时间**: 2026-03-23
**项目**: FUEL (Fast UAV Exploration & Localization)
