# A* 调用分析：fast_exploration_manager.cpp

## 两个 A* 实例

| 实例 | 所在位置 | 初始化位置 |
|------|---------|-----------|
| `ViewNode::astar_`（静态成员） | `active_perception/src/graph_node.cpp` | `FastExplorationManager::initialize()` L100-101 |
| `planner_manager_->path_finder_` | `plan_manage/planner_manager` 内部 | `initPlanModules()` 中 |

---

## 调用链 1：规划轨迹路径（`path_finder_`）

**触发函数：** `planExploreMotionCluster()` → `search_path_to()` lambda（L373~408）

```
planExploreMotionCluster()
  └─ evaluate_goal(next_pos, ...) [L417]
       └─ search_path_to(goal, path_out) [L373]
            ├─ project_to_searchable() — 将起/终点投影到自由空间（若在障碍物内则偏移）
            ├─ path_finder_->reset()
            └─ for time_budget in {0.001s, 0.008s, 0.02s}:  // 逐级放宽时间限制重试
                 path_finder_->search(start, goal)
                 如果 == REACH_END → getPath() 返回成功
                 否则继续尝试下一个时间预算
```

**作用：** 为无人机从当前位置到下一个 viewpoint 搜索一条**避障几何路径**，路径结果存入 `ed_->path_next_goal_`，后续传入 `planExploreTraj()` 进行轨迹优化。

- 若目标距离 `< 5.0m`（`radius_far`）：直接用完整路径做轨迹优化
- 若目标距离 `>= 5.0m`：截取路径前 5m 作为中间目标，再做轨迹优化

---

## 调用链 2：计算节点间代价（`ViewNode::astar_`）

**触发函数：** `updateLocalTour()`（L1133）、`updateInertialTour()`（L1176）

```
updateLocalTour() / updateInertialTour()
  └─ ViewNode::searchPath(p1, p2, path)        [graph_node.cpp:34]
  │    ├─ Raycasting 检查 p1→p2 直线是否安全
  │    │    ├─ 安全（无障碍）→ 直接返回直线路径，不调用 A*
  │    │    └─ 不安全（有障碍）→ 调用 A*：
  │    │         astar_->reset()
  │    │         astar_->search(p1, p2)
  │    │         如果 REACH_END  → 返回真实 A* 路径长度
  │    │         如果搜索失败    → 返回 1000 + 欧氏距离（作为惩罚代价）
  │    └─ 返回路径长度，用于构建 TSP/Dijkstra 代价矩阵
  │
  └─ ViewNode::computeCostPos(p1, p2, ...) [graph_node.cpp:188]
       └─ 内部同样调用 ViewNode::searchPath()
```

**作用：** 为 tour 规划（TSP/Dijkstra）构建**节点间代价矩阵**。优先用直线（射线检测），仅当直线被障碍物阻挡时才启动 A* 搜索真实路径长度。

> `updateInertialTour()` 中额外做了距离过滤（L1174）：若两点欧氏距离 `>= 8.0m`，直接用 `500 + 欧氏距离` 作为代价，**跳过 A* 搜索**，避免远距离耗时搜索。

---

## 调用链 3：可视化 Refined Local Tour（`ViewNode::astar_`）

**触发函数：** `refineLocalTour()`（L1133）

```
refineLocalTour()
  └─ for each pt in refined_pts:
       ViewNode::searchPath(refined_tour_.back(), pt, path)
         → 同调用链 2，先直线检测，失败再 A*
       将路径点追加到 ed_->refined_tour_ 用于可视化
```

**作用：** 将优化后的 local tour waypoints 之间连上真实路径，供可视化展示。

---

## 总结

| 场景 | A* 实例 | 触发函数 | 触发时机 |
|------|---------|---------|---------|
| 规划到下一 viewpoint 的轨迹路径 | `path_finder_` | `planExploreMotionCluster()` | 每次探索规划循环 |
| TSP tour 节点间代价估计 | `ViewNode::astar_` | `updateLocalTour()` / `updateInertialTour()` | 构建代价矩阵，直线不可通时 |
| 可视化 refined local tour | `ViewNode::astar_` | `refineLocalTour()` | tour 更新后生成可视化路径 |

**共同模式：** 两个实例都遵循"先直线检测，直线受阻再 A*"的策略，以减少不必要的搜索开销。
