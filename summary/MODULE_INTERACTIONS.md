# FUEL 项目模块交互详细说明

## 1. 核心规划模块间的交互

### 1.1 plan_manage ↔ plan_env（环境查询）

**调用关系**：
```
plan_manage::FastPlannerManager
    ↓
    checkTrajCollision() / checkPathCollision()
    ↓
plan_env::EDTEnvironment
    ├─ evaluateEDTWithGrad(pos, time, &dist, &grad)
    │  └─ 返回：距离值 + 距离梯度
    │
    └─ evaluateCoarseCollision()
       └─ 粗粒度碰撞检测
```

**数据流**：
- **输入**：轨迹上的采样点 (时间参数化的位置)
- **处理**：查询SDF/EDT栅格，获取到障碍物的距离
- **输出**：距离、梯度、是否碰撞标志
- **用途**：
  - 规划过程中的约束检验
  - 动态重规划的触发判断
  - 轨迹优化的成本计算

**关键参数**：
```yaml
distance_threshold: 0.2  # 碰撞判定距离
check_interval: 0.01      # 采样间隔
```

---

### 1.2 plan_manage ↔ path_searching（路径规划）

**调用关系**：
```
plan_manage::FastPlannerManager::kinodynamicReplan()
    ↓
KinodynamicAstar::search()
    ├─ 输入：start, end, start_vel, start_acc
    ├─ 内部循环：
    │   ├─ getNeighbor() → 生成运动基元
    │   ├─ 碰撞检测 → plan_env::evaluateEDTWithGrad()
    │   ├─ 成本评估 → h = dist_to_goal / max_vel
    │   └─ 优先队列 (open_set) 扩展
    └─ 返回：路径点序列 [p0, p1, ..., pn]
```

**数据交换**：
- **输入参数**：
  - start: 起点位置与速度/加速度状态
  - end: 终点位置与期望速度
  - max_vel, max_acc: 运动学约束
  
- **内部调用**：
  - `plan_env::evaluateEDTWithGrad()` - 每次扩展时查询障碍物距离
  - `plan_env::checkTrajCollision()` - 折线段碰撞检测

- **输出数据**：
  - 路径点序列 (向量序列)
  - 每个点的速度信息

**算法细节**：
```
运动基元扩展方向 (4个):
  [+x, -x, +y, -y] 或 [+v, -v] 方向
  
每个扩展的成本计算：
  cost(child) = cost(parent) + dist_to_child + h(child)
  其中 h(child) = euclidean_dist(child, goal) / max_vel
```

---

### 1.3 plan_manage ↔ bspline_opt（轨迹优化）

**调用关系**：
```
plan_manage::FastPlannerManager
    ↓
kinodynamicReplan() 或 topoReplan()
    ↓
得到路径点 → BsplineOptimizer::optimize()
    │
    ├─ 初始化：
    │  ├─ 路径 → B样条控制点 (均匀参数化)
    │  ├─ 计算时间分配 (速度/加速度约束)
    │  └─ 设置目标函数权重
    │
    ├─ 优化循环 (n_iteration):
    │   ├─ 计算约束函数：
    │   │   ├─ Distance constraint (plan_env查询)
    │   │   ├─ Velocity constraint
    │   │   ├─ Acceleration constraint
    │   │   └─ Dynamic feasibility
    │   │
    │   ├─ 计算梯度：
    │   │   ├─ J_smoothness (平滑性)
    │   │   ├─ J_distance (距离最大化)
    │   │   ├─ J_feasibility (可行性)
    │   │   └─ J_time (时间最小化)
    │   │
    │   └─ 梯度下降更新：
    │       control_pts -= learning_rate * gradient
    │
    └─ 返回：NonUniformBspline 对象
```

**关键约束**：
```yaml
安全约束:
  min_distance: 0.15  # 到障碍物最小距离
  margin: 0.2         # 安全边界
  
速度约束:
  max_vel: 2.5  # m/s
  
加速度约束:
  max_acc: 2.0  # m/s²
  
时间约束:
  max_tau: 20   # 最大规划时间
```

---

### 1.4 bspline ↔ bspline_opt（轨迹表示与优化）

**调用关系**：
```
bspline_opt::BsplineOptimizer
    │
    ├─ 创建轨迹对象：
    │  bspline::NonUniformBspline traj(control_pts, order, time_allocations)
    │
    ├─ 轨迹评估：
    │  ├─ traj.evaluateDeBoor(t) → 位置
    │  ├─ traj.evaluateDerivative(t) → 速度
    │  └─ traj.evaluateSecondDerivative(t) → 加速度
    │
    └─ 优化后生成输出轨迹
```

**数据结构**：
```cpp
class NonUniformBspline {
  vector<Eigen::Vector3d> control_points;  // 控制点
  vector<double> knots;                     // 节点向量
  int p_;                                   // 阶数 (通常为3)
  vector<double> u_;                        // 参数时间
};
```

---

### 1.5 path_searching ↔ plan_env（路径搜索中的碰撞检测）

**关键交互**：
```
KinodynamicAstar::search()
    │
    └─ 每次扩展状态时：
       ├─ neighbor_state = moveByMotionPrimitive(current_state)
       │
       ├─ 检测 neighbor 的可行性：
       │  ├─ plan_env::evaluateEDTWithGrad(neighbor.pos, time)
       │  ├─ IF distance < collision_threshold
       │  │  └─ 标记为不可行 (skip)
       │  └─ ELSE
       │     └─ 添加到 open_set
       │
       └─ 继续扩展 (广度优先/启发式优先)
```

**成本评估**：
```
f(node) = g(node) + h(node)

其中：
  g(node) = 到起点的代价 (通常是时间或距离)
  h(node) = 到目标的启发式估计
         = euclidean_dist(node.pos, goal) / max_vel
```

---

## 2. 轨迹执行层的交互

### 2.1 traj_server ↔ so3_control（轨迹到控制指令）

**ROS话题流**：
```
/planning/bspline (bspline::Bspline message)
    ↓
traj_server (接收并解析)
    ├─ 初始化：
    │  └─ bspline::Bspline b_spline = msg
    │
    ├─ 实时执行环（控制频率 50-100 Hz）:
    │  ├─ t_now = ros::Time::now() - t_start
    │  │
    │  ├─ IF t_now < b_spline.duration
    │  │  ├─ p_cmd = b_spline.evaluateDeBoor(t_now)
    │  │  ├─ v_cmd = b_spline.evaluateDerivative(t_now)
    │  │  ├─ a_cmd = b_spline.evaluateSecond(t_now)
    │  │  │
    │  │  └─ 构建 PositionCommand 消息：
    │  │     {
    │  │       position: p_cmd,
    │  │       velocity: v_cmd,
    │  │       acceleration: a_cmd,
    │  │       yaw: yaw_traj(t_now),
    │  │       yaw_rate: dyaw_traj(t_now)
    │  │     }
    │  │
    │  └─ 发布 /planning/pos_cmd
    │
    └─ 轨迹完成或被中断 → 发布重规划信号
        /planning/replan (std_msgs::Empty)
```

**消息转换**：
```
Input:  /planning/bspline (bspline::Bspline)
          ├─ knots (节点向量)
          ├─ control_points (控制点)
          └─ time_allocations (时间分配)

Processing: 轨迹评估 (导数计算)

Output: /planning/pos_cmd (quadrotor_msgs::PositionCommand)
          ├─ position
          ├─ velocity
          ├─ acceleration
          ├─ yaw
          └─ yaw_rate
```

---

### 2.2 so3_control ↔ so3_quadrotor_simulator（控制指令到动力学）

**ROS话题流**：
```
/planning/pos_cmd (PositionCommand)
    ↓
so3_control (位置控制器)
    ├─ 订阅 /state_ukf/odom (当前状态)
    │
    ├─ 位置误差计算：
    │  ├─ e_pos = odom.pose - cmd.position
    │  ├─ e_vel = odom.twist - cmd.velocity
    │  └─ e_acc_desired = cmd.acceleration - gravity_vector
    │
    ├─ PID控制律：
    │  ├─ a_ff = e_acc_desired
    │  ├─ a_p = -Kp * e_pos
    │  ├─ a_d = -Kd * e_vel
    │  └─ a_total = a_ff + a_p + a_d
    │
    ├─ 所需推力方向：
    │  ├─ 归一化 a_total 方向
    │  ├─ 计算所需四元数 q_des
    │  └─ 满足偏航角约束
    │
    ├─ 四元数错误计算：
    │  ├─ e_q = q_current * q_des⁻¹
    │  └─ ω_des = 从 e_q 提取 (proportional)
    │
    ├─ 扭矩控制：
    │  ├─ τ = I * α
    │  └─ α 基于四元数错误反馈
    │
    └─ 发布 /so3_cmd (SO3Command)
        ├─ force (推力向量 或 归一化方向)
        ├─ q (四元数)
        ├─ w (角速度)
        └─ a_des (期望加速度)
```

**PID参数示例**：
```yaml
so3_control:
  Kp_xyz: [50, 50, 80]      # 位置P增益
  Kd_xyz: [10, 10, 15]      # 位置D增益
  Kp_yaw: 30                # 偏航P增益
  Kd_yaw: 10                # 偏航D增益
```

---

### 2.3 so3_quadrotor_simulator ↔ local_sensing（仿真反馈感知）

**数据交互**：
```
so3_quadrotor_simulator
    │
    ├─ 订阅 /so3_cmd (SO3Command)
    │
    ├─ 状态积分：
    │  ├─ 从四元数 q 计算旋转矩阵 R
    │  ├─ 加速度计算：
    │  │  a = R * (thrust / mass) + gravity
    │  ├─ 状态更新（Runge-Kutta 或欧拉法）：
    │  │  x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
    │  │  v(t+dt) = v(t) + a(t)*dt
    │  │
    │  └─ 发布 /visual_slam/odom (Odometry)
    │      ├─ position: x, y, z
    │      ├─ velocity: vx, vy, vz
    │      ├─ orientation: q (四元数)
    │      └─ angular_velocity: ω
    │
    └─ 仿真器状态 → local_sensing 读取
        ├─ 机器人位置 (tf 变换)
        ├─ 机器人方向 (旋转矩阵)
        │
        └─ local_sensing::RaycastDepth
            ├─ 从仿真环境 (Gazebo voxel grid 或 mesh)
            ├─ 按光线投射获取深度
            ├─ 转换为点云
            └─ 发布 /map_ros/depth 和 /map_ros/cloud
                ├─ sensor_msgs::Image (深度图)
                └─ sensor_msgs::PointCloud2 (点云)
```

---

### 2.4 local_sensing ↔ plan_env（感知到环境更新）

**数据流**：
```
/map_ros/cloud (PointCloud2)
或
/map_ros/depth (Image)
    ↓
plan_env::RaycastDepth
    ├─ 深度图转点云 (如果输入是深度图)
    │  ├─ 相机内参 (fx, fy, cx, cy)
    │  ├─ 逐像素深度值 → 3D点
    │  └─ 坐标变换到世界坐标系
    │
    └─ plan_env::SDFMap::updateOccupancy()
        ├─ 遍历点云中的每个点
        ├─ 体素化：确定点所在网格位置
        ├─ 占有概率更新 (贝叶斯更新)：
        │  ├─ 点本身 → p_hit = 0.65 (命中)
        │  ├─ 点到原点的光线 → p_miss = 0.43 (未命中)
        │  └─ Bayesian: p_occ = p_old * p_hit / (p_old*p_hit + (1-p_old)*(1-p_miss))
        │
        ├─ 构建距离场：
        │  ├─ SDF (Signed Distance Field): 有符号距离
        │  └─ EDT (Euclidean Distance Transform): 欧氏距离
        │
        └─ 梯度计算 (用于优化):
           ├─ 数值梯度 (有限差分)
           └─ 或 Sobel 算子
```

**参数配置**：
```yaml
plan_env_params:
  local_radius: 30       # 本地地图范围 (m)
  resolution: 0.1        # 网格分辨率 (m)
  p_hit: 0.65           # 打中概率
  p_miss: 0.43          # 打不中概率
  p_occ_threshold: 0.80 # 占有判定阈值
  unknown_value: 0.5    # 未知网格值
```

---

## 3. 感知与规划的反馈环路

### 3.1 完整的规划-控制-感知环路

```
时刻 0:
  │
  ├─→ 【规划决策】
  │   ├─ 读取当前状态 (里程计)
  │   ├─ 读取环境地图 (plan_env 的 SDF)
  │   ├─ 规划新轨迹 (KAstar + BsplineOpt)
  │   └─ 发布 /planning/bspline
  │
  ├─→ 【轨迹执行】 (频率 ~50Hz)
  │   ├─ traj_server 评估轨迹
  │   ├─ 计算位置/速度/加速度指令
  │   └─ 发布 /planning/pos_cmd
  │
  ├─→ 【姿态控制】 (频率 ~100Hz)
  │   ├─ so3_control 计算 PID 控制律
  │   ├─ 从笛卡尔控制 → 姿态控制
  │   └─ 发布 /so3_cmd
  │
  ├─→ 【动力学仿真】 (频率 ~200Hz)
  │   ├─ so3_simulator 集成状态方程
  │   ├─ 计算新位置/速度/方向
  │   └─ 发布 /visual_slam/odom
  │
  ├─→ 【感知渲染】
  │   ├─ local_sensing 基于机器人位置
  │   ├─ 射线投射获取深度/点云
  │   └─ 发布 /map_ros/cloud 和 /map_ros/depth
  │
  └─→ 【环境更新】
      ├─ plan_env 接收新点云
      ├─ 更新占有栅格 (概率积分)
      ├─ 重建 SDF 和梯度
      └─ 准备下一步规划
```

**时间尺度**：
```
规划频率:      1-2 Hz   (决策等级)
轨迹执行:     50 Hz   (中间层)
控制:        100 Hz   (低层)
仿真:        200 Hz   (物理层)
感知:         20 Hz   (传感器)
```

---

## 4. 主要模块的关键函数签名

### plan_manage::FastPlannerManager

```cpp
class FastPlannerManager {
public:
  // 规划接口
  bool kinodynamicReplan(
    Eigen::Vector3d start, Eigen::Vector3d start_vel,
    Eigen::Vector3d start_acc, Eigen::Vector3d end, Eigen::Vector3d end_vel
  );
  
  bool topoReplan(bool collide);
  
  bool checkPathCollision(
    const std::vector<Eigen::Vector3d>& path,
    double& max_distance
  );
  
  // 轨迹查询
  NonUniformBspline getRefTraj(const Eigen::Vector3d& pos, double& t_now);
  
  // 参数设置
  void setPlanningParams(const PlanParameters& params);
};
```

### plan_env::EDTEnvironment

```cpp
class EDTEnvironment {
public:
  void evaluateEDTWithGrad(
    const Eigen::Vector3d& pos,
    double time,
    double& dist,
    Eigen::Vector3d& grad
  );
  
  void checkTrajCollision(
    const std::vector<Eigen::Vector3d>& path,
    std::vector<bool>& collision_flags
  );
  
  void buildLocalEDT();  // 重建距离场
  void updateOccupancy(
    const sensor_msgs::PointCloud2& cloud
  );
};
```

### path_searching::KinodynamicAstar

```cpp
class KinodynamicAstar {
public:
  bool search(
    Eigen::Vector3d start, Eigen::Vector3d goal,
    Eigen::Vector3d start_v, Eigen::Vector3d start_a,
    Eigen::Vector3d goal_v,
    std::vector<Eigen::Vector3d>& path
  );
  
  std::vector<StatePtr> getPath() const;
};
```

### bspline_opt::BsplineOptimizer

```cpp
class BsplineOptimizer {
public:
  void optimize(
    Eigen::MatrixXd& control_points,
    double time_budget,
    const std::vector<Eigen::Vector3d>& guide_path
  );
  
  NonUniformBspline getOptimizedTrajectory() const;
};
```

---

## 5. 错误处理与异常流程

### 5.1 规划失败处理

```
plan_manage::kinodynamicReplan()
    ├─ 如果 KAstar::search() 返回 false
    │  ├─ 日志记录规划失败原因
    │  ├─ FSM 转移到 REPLAN_TRAJ
    │  ├─ 尝试拓扑规划 (topoReplan)
    │  └─ 如果仍失败：发布告警，进入 WAIT_TARGET
    │
    └─ 如果 BsplineOptimizer::optimize() 失败
       ├─ 使用未优化的轨迹
       └─ 或触发重规划
```

### 5.2 碰撞恢复

```
轨迹执行中检测到碰撞 (plan_env::checkTrajCollision)
    │
    ├─ 设置碰撞标志
    ├─ 停止当前轨迹执行
    ├─ 更新环境地图 (plan_env)
    └─ FSM 转移到 REPLAN_TRAJ
        └─ 从当前位置重新规划新轨迹
```

---

## 6. 数据流总结表

| 数据 | 来源 | 流向 | 用途 | 频率 |
|------|------|------|------|------|
| 位置/速度 (Odometry) | so3_simulator | plan_manage | 规划起点 | 20 Hz |
| 点云 (PointCloud2) | local_sensing | plan_env | 环境更新 | 20 Hz |
| SDF/梯度 | plan_env | bspline_opt | 碰撞约束 | 1-2 Hz |
| 路径点 | path_searching | bspline_opt | 轨迹初始化 | 1-2 Hz |
| B样条轨迹 | bspline_opt | traj_server | 轨迹执行 | 1-2 Hz |
| 位置指令 | traj_server | so3_control | 控制输入 | 50 Hz |
| 控制指令 | so3_control | so3_simulator | 动力学 | 100 Hz |
| 新里程计 | so3_simulator | 所有模块 | 状态更新 | 200 Hz (内部) |

---

**更新时间**: 2026-03-23
