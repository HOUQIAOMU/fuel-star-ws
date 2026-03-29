# FUEL 项目架构文档索引

本文档集合提供了FUEL项目(Fast UAV Exploration & Localization)的完整架构分析，包括项目结构、数据流、模块交互等多个维度。

## 📚 文档列表

### 1. **PROJECT_ARCHITECTURE.md** (16KB) - 核心架构文档
主要内容：
- ✅ **第1章: 项目结构梳理** 
  - 20个ROS包的分类 (fuel_planner 9个 + uav_simulator 11个)
  - 各包功能说明
  
- ✅ **第2章: 数据流和话题分析**
  - 核心ROS话题通信路径
  - 完整话题清单 (Subscribers/Publishers)
  - 函数调用链 (规划→控制→仿真环路)
  - 关键依赖关系
  
- ✅ **第3章: 核心模块功能详解**
  - plan_manage (规划管理)
  - plan_env (环境表示与碰撞检测)
  - path_searching (路径搜索)
  - traj_utils (轨迹工具)
  - bspline/bspline_opt (B样条)
  - active_perception (视觉感知)
  
**适用场景**: 快速了解项目整体架构、核心模块功能

---

### 2. **ARCHITECTURE_DIAGRAMS.md** (25KB) - 架构可视化图表
主要内容：
- ✅ **第1章: 整体系统架构** (ASCII图)
  - 分层架构图 (规划层、仿真控制层、工具库)
  
- ✅ **第2章: 数据流通路**
  - 完整规划-控制环路图
  - ROS话题通信流图
  - 规划算法流程图 (运动学规划/拓扑规划)
  
- ✅ **第3章: 包间依赖关系树**
  - 所有模块的依赖链条
  - 基础库标注
  
- ✅ **第4章: 规划算法流程图**
  - 运动学规划 (Kinodynamic Planning)
  - 拓扑规划 (Topological Planning)
  
- ✅ **第5章: FSM状态转移图**
  - 规划状态机的完整流程
  
- ✅ **第6章: 参数流向图**
  - 配置参数如何流入各模块
  
- ✅ **第7章: 关键接口总结**
  - 各模块关键类/函数一览表

**适用场景**: 需要可视化理解数据流、依赖关系、算法流程

---

### 3. **MODULE_INTERACTIONS.md** (11KB) - 模块交互详细说明
主要内容：
- ✅ **第1章: 核心规划模块间的交互**
  - plan_manage ↔ plan_env (环境查询)
  - plan_manage ↔ path_searching (路径规划)
  - plan_manage ↔ bspline_opt (轨迹优化)
  - bspline ↔ bspline_opt (轨迹表示)
  - path_searching ↔ plan_env (碰撞检测)
  
- ✅ **第2章: 轨迹执行层的交互**
  - traj_server ↔ so3_control (轨迹到控制)
  - so3_control ↔ so3_simulator (控制到动力学)
  - so3_simulator ↔ local_sensing (仿真反馈感知)
  - local_sensing ↔ plan_env (感知到环境)
  
- ✅ **第3章: 感知与规划的反馈环路**
  - 完整的规划-控制-感知环路
  - 时间尺度说明
  
- ✅ **第4章: 主要模块的关键函数签名**
  - 核心类和方法定义
  
- ✅ **第5章: 错误处理与异常流程**
  - 规划失败处理
  - 碰撞恢复
  
- ✅ **第6章: 数据流总结表**
  - 各数据源、流向、用途、频率

**适用场景**: 需要理解模块间如何通信、数据如何流动、调用关系

---

## 🎯 快速导航

### 按用途查找

| 我想了解... | 查看文档 | 章节 |
|-----------|--------|------|
| 项目有多少个包，各自做什么 | PROJECT_ARCHITECTURE | 第1章 |
| ROS话题有哪些，怎么流动 | PROJECT_ARCHITECTURE | 第2章 |
| 规划算法的详细工作流程 | MODULE_INTERACTIONS | 第1章 |
| 控制信号怎么从规划传到仿真 | ARCHITECTURE_DIAGRAMS | 第2章 |
| plan_manage 调用了哪些其他包 | MODULE_INTERACTIONS | 第1章 |
| 完整的闭环流程 (规划→控制→感知) | MODULE_INTERACTIONS | 第3章 |
| 各包之间的依赖关系 | ARCHITECTURE_DIAGRAMS | 第3章 |
| FSM状态转移过程 | ARCHITECTURE_DIAGRAMS | 第5章 |
| 怎么添加新的规划算法 | PROJECT_ARCHITECTURE | 第4.3节 + MODULE_INTERACTIONS |
| 感知数据怎么被规划利用 | MODULE_INTERACTIONS | 第2.4章 |

---

## 📊 架构快速概览

### 两大模块

```
FUEL 项目 (20个包)
├── fuel_planner (9个包)
│   ├─ 核心: plan_manage, plan_env, path_searching
│   ├─ 轨迹: bspline, bspline_opt, poly_traj
│   ├─ 工具: traj_utils
│   └─ 应用: active_perception, exploration_manager
│
└── uav_simulator (11个包)
    ├─ 核心仿真: so3_control, so3_quadrotor_simulator
    ├─ 感知: local_sensing, map_generator
    ├─ 消息: quadrotor_msgs, uav_utils
    └─ 辅助: poscmd_2_odom, odom_visualization 等
```

### 数据流动方向

```
输入 (感知)
  ↓
规划 (决策)
  ↓
控制 (执行)
  ↓
仿真 (模拟)
  ↓
感知 (反馈) → 循环回规划
```

### 主要ROS话题

```
输入: /state_ukf/odom, /map_ros/cloud
规划: /planning/bspline, /planning/pos_cmd
控制: /so3_cmd
仿真: /visual_slam/odom (反馈)
```

---

## 🔧 核心功能流程

### 规划流程 (1-2 Hz)

```
1. 获取当前状态 (位置、速度)
2. 查询环境地图 (SDF/EDT)
3. 搜索路径 (KinodynamicAstar 或 TopologyPRM)
4. 优化轨迹 (BsplineOptimizer)
5. 发布B样条 (/planning/bspline)
6. 进入轨迹执行
```

### 轨迹执行流程 (50-200 Hz)

```
1. 解析B样条轨迹
2. 实时评估 (位置、速度、加速度)
3. 发送位置指令 (/planning/pos_cmd)
4. SO(3)控制器计算姿态
5. 发送控制指令 (/so3_cmd)
6. 四旋翼仿真器集成动力学
7. 返回新里程计 (反馈)
```

---

## 📝 重要参数配置位置

- **规划参数** (speed, acceleration limits): plan_manage
- **控制参数** (PID gains, mass): so3_control
- **感知参数** (resolution, occupancy): plan_env
- **仿真参数** (time step, disturbance): so3_quadrotor_simulator

参数通常在 `*.launch` 文件中定义，在 `*.h` 文件中有默认值。

---

## 🚀 典型使用场景

### 场景1: 调试规划算法
1. 查看 PROJECT_ARCHITECTURE 第4.3节了解path_searching
2. 查看 MODULE_INTERACTIONS 第1.2章了解plan_manage的调用
3. 修改规划参数或算法，重新编译

### 场景2: 添加新的控制器
1. 参考 ARCHITECTURE_DIAGRAMS 第2.2章的控制流
2. 查看 MODULE_INTERACTIONS 第2.2章的so3_control实现
3. 实现新控制器，订阅pos_cmd，发布so3_cmd

### 场景3: 集成新传感器
1. 查看 MODULE_INTERACTIONS 第2.4章的感知处理
2. 修改 local_sensing 或 plan_env 的数据源
3. 确保发布正确的ROS话题格式

### 场景4: 理解重规划机制
1. 查看 ARCHITECTURE_DIAGRAMS 第5章的FSM
2. 查看 PROJECT_ARCHITECTURE 第2章的重规划信号
3. 查看 MODULE_INTERACTIONS 第5.2章的碰撞恢复

---

## 📖 阅读建议

**第一次阅读** (30分钟)
1. 本索引文档 (快速概览)
2. ARCHITECTURE_DIAGRAMS 第1-2章 (系统架构和数据流)

**深入学习** (1-2小时)
1. PROJECT_ARCHITECTURE 第1-2章 (结构和话题)
2. MODULE_INTERACTIONS 第1-2章 (模块交互)

**成为专家** (完整阅读)
1. 逐个阅读全部3个文档
2. 结合源代码理解实现细节
3. 参考 PROJECT_ARCHITECTURE 第4章的各模块详解

---

## 📞 相关源代码位置

| 模块 | 源代码位置 |
|------|----------|
| plan_manage | src/FUEL/fuel_planner/plan_manage/src/ |
| plan_env | src/FUEL/fuel_planner/plan_env/src/ |
| path_searching | src/FUEL/fuel_planner/path_searching/src/ |
| bspline_opt | src/FUEL/fuel_planner/bspline_opt/src/ |
| so3_control | src/FUEL/uav_simulator/so3_control/src/ |
| so3_quadrotor_simulator | src/FUEL/uav_simulator/so3_quadrotor_simulator/src/ |
| local_sensing | src/FUEL/uav_simulator/local_sensing/src/ |
| traj_server | plan_manage/src/trajectory_server.cpp |

---

**文档生成时间**: 2026-03-23
**项目**: FUEL (Fast UAV Exploration & Localization)
**覆盖包数**: 20个ROS包

---

## 💡 Tips

- 所有3个文档都有丰富的ASCII流程图，方便离线参考
- MODULE_INTERACTIONS 有函数签名，便于查看接口
- ARCHITECTURE_DIAGRAMS 有关键参数示例，方便调试
- 建议在IDE中打开这些文档作为参考，同时阅读源代码
