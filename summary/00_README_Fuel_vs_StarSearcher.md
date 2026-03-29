# 📊 Fuel vs Star-Searcher - fast_exploration_manager.cpp 完整对比

## 📂 本目录文档说明

本目录包含了针对 **Fuel** 和 **Star-Searcher** 两个项目中 `fast_exploration_manager.cpp` 文件的详细对比分析。

### 核心对比文档

| 文档名 | 内容 | 适合场景 |
|--------|------|---------|
| **快速探索管理器代码差异对比报告.md** | 📋 完整对比分析，包括头文件、参数、函数等全方位差异 | 全面了解两个版本的区别 |
| **新增函数详细说明.md** | 🔍 Fuel版本新增的9个函数详细说明 | 理解Fuel的新功能 |
| **函数速查表.md** | ⚡ 函数快速查询表，包含签名、参数、返回值 | 快速查阅函数信息 |

---

## 📌 核心差异一览

### 新增函数统计

**Fuel 相比 Star-Searcher 新增了9个关键函数**:

```
1. ✅ shortenPath                  - 路径简化
2. ✅ findNextCluster              - 聚类选择 ⭐⭐⭐⭐⭐
3. ✅ findLocalTour                - 局部TSP求解
4. ✅ findGlobalTour               - 全局TSP求解
5. ✅ updateFrontierStruct         - Frontier更新
6. ✅ updateInertialTour           - 惯性路线规划 ⭐⭐⭐⭐
7. ✅ solveTSP                     - 通用TSP求解接口 ⭐⭐⭐⭐⭐
8. ✅ clearExplorationData         - 数据清空
9. ✅ hausdorffDistance            - 距离计算
```

### 修改的函数

**1个主函数重命名**:
- `planExploreMotion` → `planExploreMotionCluster` 
- 从全局规划改为聚类化分层规划

**1个初始化函数扩展**:
- `initialize()` 新增7个参数 + 可视化模块 + TSP预处理

---

## 🎯 关键创新 - Fuel的三大核心函数

### 1. findNextCluster - 多聚类选择策略 ⭐⭐⭐⭐⭐

```cpp
void findNextCluster(
    const Vector3d &cur_pos,     // 当前位置
    const Vector3d &cur_vel,     // 当前速度
    const Vector3d &cur_yaw,     // 当前偏航角
    vector<checkPoint> &check_tour,       // [输出] 检查点
    Eigen::Vector3d &next_cluster_pos,    // [输出] 下一聚类
    const bool neighbor)         // 是否考虑邻近
```

**创新点**:
- 支持2种聚类选择策略：
  - ✨ **惯性规划** (considerInertial) - 基于运动连续性
  - �� **TSP优化** (标准模式) - 基于成本最优
- 自动选择成本更低的策略

---

### 2. updateInertialTour - 惯性路线规划 ⭐⭐⭐⭐

```cpp
void updateInertialTour(
    const vector<int> &indices,                        // 聚类索引
    const vector<Eigen::Vector3d> &cluster_centers,    // 聚类中心
    double &inertia_cost)                              // [输出] 惯性成本
```

**创新点**:
- Fuel独有的惯性路线规划
- 考虑机器人运动连续性
- 支持特征点距离阈值
- 可调节成本偏移参数

---

### 3. solveTSP - 通用TSP求解接口 ⭐⭐⭐⭐⭐

```cpp
void solveTSP(
    const Eigen::MatrixXd &cost_matrix,     // N×N成本矩阵
    const TSPConfig &config,                // 配置结构体
    vector<int> &result_indices,            // [输出] 最优路线
    double &total_cost)                     // [输出] 总成本
```

**创新点**:
- 统一的TSP求解框架
- 灵活的配置系统 (跳过首尾点、ID偏移等)
- 自动文件管理 (.tsp → LKH求解 → .txt)
- 支持多种问题类型 (hgrid, frontier, cluster)

---

## 📊 数据结构对比

| 方面 | Star-Searcher | Fuel |
|------|---------------|------|
| **路线表示** | `global_tour` | `global_tour` + `local_tour` + `grid_tour` + `inertia_tour` |
| **Frontier处理** | 获取边界框 | 获取分割聚类 + 计算中心 |
| **路径优化** | 无 | `shortenPath()` |
| **聚类支持** | ❌ | ✅ (完整的聚类选择策略) |
| **距离度量** | 无 | `hausdorffDistance()` |
| **可视化** | ❌ | ✅ (PlanningVisualization) |

---

## ⚙️ 参数扩展 (initialize函数新增)

Fuel版本在initialize()中新增了7个参数:

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `init_plan_num` | int | 2 | 初始规划数量 |
| `alc_cp_search_range` | double | 10 | 检查点搜索范围 |
| `enable_fixed_hgrid` | bool | false | 固定网格启用 |
| `perception_aware_local` | bool | false | 感知感知规划 |
| `feature_max_dist` | double | -1.0 | 特征最大距离 |
| `inertial_cost_offset` | double | -1.0 | 惯性成本偏移 |
| `verbose_active_loop` | bool | false | 详细日志 |

---

## 🔄 核心规划流程对比

### Star-Searcher 流程
```
planExploreMotion
  ↓
getTopViewpointsInfo (所有viewpoint)
  ↓
findGlobalTour (全局TSP)
  ↓
refineLocalTour (可选的细化)
  ↓
返回单个 next_yaw
```

### Fuel 流程 (新)
```
planExploreMotionCluster
  ↓
getFrontierDivision (获取多个聚类)
  ↓
findNextCluster ⭐ (聚类选择)
  ├─ updateInertialTour (惯性成本)
  ├─ findLocalTour (局部TSP)
  └─ solveTSP
  ↓
返回多步规划的 next_yaw[]
```

---

## 📈 功能完整性评分

| 维度 | Star-Searcher | Fuel | 提升 |
|------|---------------|------|------|
| **算法复杂度** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | +2★ |
| **聚类支持** | ❌ | ⭐⭐⭐⭐⭐ | +5★ |
| **可视化** | ❌ | ⭐⭐⭐⭐⭐ | +5★ |
| **参数灵活性** | ⭐⭐ | ⭐⭐⭐⭐ | +2★ |
| **性能优化** | ⭐⭐ | ⭐⭐⭐⭐ | +2★ |
| **代码规范** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | +2★ |

---

## 🚀 使用建议

### 当您需要...

📌 **快速浏览核心差异**
→ 查看《快速探索管理器代码差异对比报告.md》的「概览」和「关键差异总结」

📌 **理解新增函数的输入输出**
→ 查看《函数速查表.md》的「完整函数签名列表」

📌 **深入研究某个函数的实现**
→ 查看《新增函数详细说明.md》中该函数的详细说明

📌 **查询函数的调用关系**
→ 查看《新增函数详细说明.md》中的「函数关系图」

📌 **了解完整的规划流程**
→ 查看《新增函数详细说明.md》中的「函数调用流程」

---

## 📚 文档导航

```
00_README_Fuel_vs_StarSearcher.md (本文档)
├── 快速探索管理器代码差异对比报告.md
│   ├── 1. 头文件Include变化
│   ├── 2. 构造函数改动
│   ├── 3. initialize()函数改动
│   ├── 4. 核心算法函数改动 ⭐ 最重要
│   ├── 5. 数据结构差异
│   ├── 6. Frontier处理差异
│   ├── 7. 新增关键函数 ⭐ 最重要
│   ├── 8. 代码风格改动
│   ├── 9. 性能优化相关改动
│   └── 10. 总体评估
│
├── 新增函数详细说明.md
│   ├── 9个完全新增的函数详解
│   ├── 2个修改的函数
│   ├── 函数关系图 📊
│   ├── 函数调用流程 🔄
│   └── 核心创新函数组合 🚀
│
└── 函数速查表.md
    ├── 快速概览表
    ├── 完整函数签名列表
    ├── 数据类型速查
    ├── 函数使用场景
    └── 核心创新函数组合

```

---

## ✨ 最关键的3个函数

如果您只有时间看3个函数，这是必看的：

### 1️⃣ **findNextCluster** - 多聚类选择
- **为什么重要**: Fuel的核心创新，实现聚类选择策略
- **复杂度**: ⭐⭐⭐⭐⭐ 高
- **位置**: 《新增函数详细说明.md》第2节

### 2️⃣ **solveTSP** - 通用求解器
- **为什么重要**: 所有TSP求解的统一接口
- **复杂度**: ⭐⭐⭐⭐ 中-高
- **位置**: 《新增函数详细说明.md》第7节

### 3️⃣ **updateInertialTour** - 惯性规划
- **为什么重要**: Fuel独特的运动连续性优化
- **复杂度**: ⭐⭐⭐⭐ 中-高
- **位置**: 《新增函数详细说明.md》第6节

---

## 📋 快速对比表

### 函数变化统计

| 类型 | 数量 | 说明 |
|------|------|------|
| 新增函数 | 9 | shortenPath, findNextCluster, 等 |
| 重命名函数 | 1 | planExploreMotion → planExploreMotionCluster |
| 修改函数 | 1 | initialize 函数扩展 |
| **总计变化** | **11** | - |

### Include变化统计

| 类型 | 数量 | 说明 |
|------|------|------|
| 新增Include | 3 | visualization, expl_data, 等 |
| 移除Include | 0 | - |
| 重排序 | 6 | 代码规范化 |

### 参数变化统计

| 类型 | 数量 | 说明 |
|------|------|------|
| 新增ROS参数 | 7 | init_plan_num, alc_cp_search_range, 等 |
| 数据结构增强 | 4 | local_tour, grid_tour, inertia_tour, clusters |

---

## 🎓 总结

**Fuel版本是Star-Searcher的升级版，主要改进包括**:

1. ✨ **聚类化架构** - 从单一全局规划改为分层聚类规划
2. 🎯 **多步规划** - 支持多个viewpoint的并行规划
3. 🚀 **惯性优化** - 考虑机器人运动连续性的新策略
4. 📊 **完整框架** - 统一的TSP求解接口和配置系统
5. 🎨 **可视化增强** - 完整的轨迹规划可视化
6. ⚙️ **灵活参数** - 7个新参数支持更细粒度的控制

---

**生成时间**: 2026-03-24  
**对比文件**: `fast_exploration_manager_diff.txt` (945行)  
**分析范围**: Star-Searcher vs Fuel 的 fast_exploration_manager.cpp

