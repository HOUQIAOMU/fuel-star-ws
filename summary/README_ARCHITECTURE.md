# FUEL 项目架构文档说明

## 📋 概述

本目录包含FUEL (Fast UAV Exploration & Localization) 项目的完整架构分析文档。这是一套全面、系统的文档，覆盖项目的20个ROS包、数据流、模块交互等所有关键方面。

## 📂 文档构成 (5个文档, 1977行, 71KB)

### 1️⃣ **QUICK_REFERENCE.md** ⭐ 推荐首先阅读
- **大小**: 6.6KB
- **行数**: ~200行
- **时间**: 5-10分钟
- **内容**: 
  - 项目一页纸快速参考
  - 20个包的快速分类
  - 核心数据流图
  - ROS话题一览表
  - 常见问题查找
  - 参数关键值
  - 调试建议

**适用**: 想快速入门、快速查找信息

---

### 2️⃣ **ARCHITECTURE_INDEX.md** ⭐ 导航和索引
- **大小**: 7.7KB
- **行数**: ~300行
- **时间**: 10-15分钟
- **内容**:
  - 文档导航和使用指南
  - 按用途的快速查找表
  - 架构快速概览
  - 主要功能流程 (规划/执行)
  - 阅读建议 (30分钟/1-2小时/完整)
  - 相关源代码位置

**适用**: 不知道看哪个文档、需要导航

---

### 3️⃣ **PROJECT_ARCHITECTURE.md** 核心详细文档
- **大小**: 16KB
- **行数**: ~500行
- **时间**: 30-45分钟
- **内容**:
  - **第1章**: 20个包的详细分类 (表格形式)
  - **第2章**: ROS话题、数据流、函数调用链、依赖关系
  - **第3章**: 6大核心模块的详细功能
    - plan_manage (规划管理)
    - plan_env (环境表示)
    - path_searching (路径搜索)
    - traj_utils (轨迹工具)
    - bspline (B样条)
    - active_perception (视觉感知)

**适用**: 需要深入了解项目架构和核心模块

---

### 4️⃣ **ARCHITECTURE_DIAGRAMS.md** 可视化图表库
- **大小**: 25KB
- **行数**: ~800行
- **时间**: 30-45分钟
- **内容**:
  - **第1章**: 整体系统架构 (ASCII图)
  - **第2章**: 数据流通路 (流程图)
  - **第3章**: 包间依赖关系树
  - **第4章**: 规划算法流程图
  - **第5章**: FSM状态转移图
  - **第6章**: 参数流向图
  - **第7章**: 关键接口总结表

**特点**: 丰富的ASCII流程图，离线阅读友好

**适用**: 需要可视化理解、喜欢图表、理解依赖关系

---

### 5️⃣ **MODULE_INTERACTIONS.md** 交互细节
- **大小**: 16KB
- **行数**: ~400行
- **时间**: 30-45分钟
- **内容**:
  - **第1章**: 规划模块间的交互
    - plan_manage ↔ plan_env
    - plan_manage ↔ path_searching
    - plan_manage ↔ bspline_opt
    - 等6个关键交互对
  - **第2章**: 轨迹执行层交互
    - traj_server ↔ so3_control
    - so3_control ↔ so3_simulator
    - so3_simulator ↔ local_sensing
    - local_sensing ↔ plan_env
  - **第3章**: 完整反馈环路 + 时间尺度
  - **第4章**: 关键函数签名
  - **第5章**: 错误处理流程
  - **第6章**: 数据流总结表

**特点**: 详细的数据流向、函数调用、参数交换

**适用**: 需要理解模块间通信、添加新模块、修改现有模块

---

## 🎯 使用指南

### 根据角色选择阅读

#### 👨‍💼 项目经理 / 系统设计者
1. QUICK_REFERENCE.md (5分钟了解全貌)
2. ARCHITECTURE_DIAGRAMS.md 第1-2章 (系统架构)

#### 👨‍💻 新开发者 / 实习生
1. QUICK_REFERENCE.md (快速入门)
2. ARCHITECTURE_INDEX.md (选择性阅读)
3. PROJECT_ARCHITECTURE.md (了解各模块)
4. ARCHITECTURE_DIAGRAMS.md (查看数据流)

#### 🔧 模块维护者 / 算法工程师
1. QUICK_REFERENCE.md (快速参考参数)
2. PROJECT_ARCHITECTURE.md (相关模块详解)
3. MODULE_INTERACTIONS.md (交互关系)
4. ARCHITECTURE_DIAGRAMS.md (查看约束和限制)

#### 🚀 全栈工程师 / 系统整合者
**完整阅读**: 所有5个文档 (按顺序)
- 第一遍: QUICK_REFERENCE → ARCHITECTURE_INDEX → PROJECT_ARCHITECTURE
- 第二遍: MODULE_INTERACTIONS → ARCHITECTURE_DIAGRAMS
- 结合源代码理解实现

---

### 按任务快速查找

| 我要... | 查看文档 | 位置 |
|--------|--------|------|
| 快速入门 | QUICK_REFERENCE | 整个文档 |
| 了解有哪些包 | PROJECT_ARCHITECTURE | 第1章 |
| 看ROS话题流向 | MODULE_INTERACTIONS | 第2.1-2.4章 + ARCHITECTURE_DIAGRAMS 第2章 |
| 理解规划算法 | ARCHITECTURE_DIAGRAMS | 第4章 |
| 修改控制器 | MODULE_INTERACTIONS | 第2.2章 |
| 添加新传感器 | MODULE_INTERACTIONS | 第2.4章 |
| 理解FSM状态 | ARCHITECTURE_DIAGRAMS | 第5章 |
| 调试性能问题 | QUICK_REFERENCE | "调试建议" + PROJECT_ARCHITECTURE 第3章 |
| 找到源代码 | ARCHITECTURE_INDEX | "相关源代码位置" |
| 了解模块依赖 | ARCHITECTURE_DIAGRAMS | 第3章 |

---

## 📊 文档特点

### ✅ 优势
- ✓ **全面性**: 覆盖20个包、所有关键模块、完整数据流
- ✓ **层次性**: 从快速参考到深度分析，满足不同需求
- ✓ **可读性**: 丰富的表格、ASCII图、流程图
- ✓ **实用性**: 包含参数值、函数签名、调试建议
- ✓ **可维护性**: 模块化的5个文档，易于更新某一部分
- ✓ **离线友好**: 所有图表都是文本形式，可以离线查看

### 📐 信息架构
```
QUICK_REFERENCE (快速参考)
        ↓ (深度 ↓)
ARCHITECTURE_INDEX (导航)
        ↓
PROJECT_ARCHITECTURE (详细)
        ↓
MODULE_INTERACTIONS (细节)
ARCHITECTURE_DIAGRAMS (图表) ← 可并行阅读
        ↓
完整理解整个系统
```

---

## 🔍 核心概念速览

### 20个包分两大部分
```
FUEL (Fast UAV Exploration)
├── fuel_planner (9包) ← 规划、路径搜索、轨迹优化
└── uav_simulator (11包) ← 控制、仿真、感知
```

### 5个关键ROS话题
```
/state_ukf/odom          ← 当前位置 (20Hz)
    ↓
/planning/bspline        ← 规划轨迹 (1-2Hz)
    ↓
/planning/pos_cmd        ← 位置指令 (50Hz)
    ↓
/so3_cmd                 ← 控制指令 (100Hz)
    ↓
/visual_slam/odom        ← 新位置反馈 (200Hz) → 循环
```

### 4个关键算法
1. **KinodynamicAstar** - 运动学路径搜索
2. **TopologyPRM** - 拓扑路径搜索
3. **BsplineOptimizer** - 轨迹优化
4. **SO(3)Control** - 姿态控制

---

## 💡 使用建议

1. **首次接触项目**
   - 先读QUICK_REFERENCE (5分钟快速了解)
   - 再看ARCHITECTURE_DIAGRAMS 第1-2章 (数据流)
   - 最后查ARCHITECTURE_INDEX找具体需要的内容

2. **需要修改代码**
   - 在QUICK_REFERENCE找参数位置
   - 在PROJECT_ARCHITECTURE找模块详解
   - 在MODULE_INTERACTIONS找调用关系

3. **性能调优**
   - 看QUICK_REFERENCE的"调试建议"
   - 查PROJECT_ARCHITECTURE的关键参数
   - 分析ARCHITECTURE_DIAGRAMS的算法流程

4. **整合新模块**
   - 查MODULE_INTERACTIONS的交互模式
   - 参考ARCHITECTURE_DIAGRAMS的接口表
   - 按照数据流接入新模块

---

## 📚 配套资源

- **源代码**: `src/FUEL/` 目录
- **Launch文件**: 各package下的 `launch/` 目录
- **参数文件**: 各package下的 `config/` 目录
- **本项目README**: `src/FUEL/README.md`

---

## 📝 文档版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-03-23 | 初始版本，覆盖20个包的完整架构分析 |

---

## ❓ 常见问题

**Q: 我应该先读哪个文档?**
A: 从QUICK_REFERENCE开始 (5分钟)，然后根据需要选择其他文档。

**Q: 我只有15分钟，应该看什么?**
A: QUICK_REFERENCE + ARCHITECTURE_DIAGRAMS 第1-2章

**Q: 图表能否复制到其他工具?**
A: 可以！所有图表都是纯文本ASCII，可以复制到任何编辑器。

**Q: 文档包含完整的源代码分析吗?**
A: 不完全包含。文档提供架构和调用关系，具体实现细节请查看源代码。

**Q: 文档会定期更新吗?**
A: 这是初版。建议在项目有重大变更时更新文档。

---

## 📞 反馈

如有建议或发现错误，请检查相关源代码并提交更新。

---

**总结**: 这套5个文档共1977行、71KB的架构文档集合，提供了FUEL项目从快速参考到深度分析的完整覆盖，满足不同角色、不同深度的需求。建议收藏此README作为入口点。

**文档生成时间**: 2026-03-23  
**覆盖范围**: 20个ROS包、完整系统架构、数据流、模块交互
