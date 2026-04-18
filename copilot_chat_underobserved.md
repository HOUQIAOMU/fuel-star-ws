 答案非常明确：

  ----------------------------------------------------------------------------------------------------------------------------------------

  文件与函数

  文件：src/FUEL/fuel_planner/plan_env/include/plan_env/sdf_map.h

  函数：SDFMap::getOccupancy(const Eigen::Vector3i &id)（第241行，inline实现）

  ----------------------------------------------------------------------------------------------------------------------------------------

  分类逻辑（第241-257行）

   inline int SDFMap::getOccupancy(const Eigen::Vector3i &id) {
     double occ           = md_->occupancy_buffer_[toAddress(id)];   // 占用概率（log-odds）
     double observed_dist = md_->min_observed_dist_[toAddress(id)];  // 传感器最近观测距离
   
     if (occ < mp_->clamp_min_log_ - 1e-3)
       return UNKNOWN;          // 从未被观测过（log-odds极小）
   
     if (occ > mp_->min_occupancy_log_) {
       if (observed_dist > mp_->belief_dist_)
         return UNDEROBSERVED;  // 被标记为占用，但观测距离太远，置信度不足
       else
         return OCCUPIED;       // 被标记为占用，且近距离确认
     }
   
     return FREE;               // 被观测过，且未占用
   }

  ----------------------------------------------------------------------------------------------------------------------------------------

  三种类型的判断依据

  ┌─────────────────┬─────────────────────────────────────────────────────────────────────────────────────────────────────┐
  │ 类型            │ 条件                                                                                                │
  ├─────────────────┼─────────────────────────────────────────────────────────────────────────────────────────────────────┤
  │ UNKNOWN         │ occupancy_buffer < clamp_min_log，即该voxel从未被任何传感器光线hit过                                │
  ├─────────────────┼─────────────────────────────────────────────────────────────────────────────────────────────────────┤
  │ UNDEROBSERVED   │ log-odds > 阈值（认为有障碍），但传感器观测该点的最近距离 > belief_dist，即观测太远、置信度低       │
  ├─────────────────┼─────────────────────────────────────────────────────────────────────────────────────────────────────┤
  │ OCCUPIED        │ log-odds > 阈值，且近距离观测确认（observed_dist ≤ belief_dist）                                    │
  ├─────────────────┼─────────────────────────────────────────────────────────────────────────────────────────────────────┤
  │ FREE            │ log-odds ≤ 阈值（被观测过但判定为空旷）                                                             │
  └─────────────────┴─────────────────────────────────────────────────────────────────────────────────────────────────────┘

  UNDEROBSERVED 是FUEL相比标准地图的关键扩展——用于识别"看到了但没看清楚"的voxel，驱动主动感知去做近距离确认。

❯ 我的active_perception里面暂时把有关处理underobserved函数的部分去掉了，因为pcl_render_node下的rviz仿真似乎无法很好的处理underobserved，那
  我要不要在underobserved的定义处暂时注释掉underobserved，这样会不会对飞机因为残留frontier而总是绕圈的问题有一定缓解                      

◐ The user is asking whether commenting out the UNDEROBSERVED definition would help with the issue of the UAV circling due to residual 
  frontiers. Let me think about this carefully.

  The user has already removed underobserved handling in active_perception. Now they're asking if they should also comment out the 
  UNDEROBSERVED enum/classification in sdf_map.h.

  Let me think about the implications:

  1. If UNDEROBSERVED is removed from the enum, then `getOccupancy()` would never return UNDEROBSERVED - those voxels would fall through 
  to... what? Looking at the logic:
     - If occ > min_occupancy_log_ and observed_dist > belief_dist_ → currently returns UNDEROBSERVED
     - If we remove UNDEROBSERVED, these voxels would need to return something else

  2. The question is: what are these UNDEROBSERVED voxels being treated as in frontier_finder.cpp now that the underobserved handling is 
  removed?

  Let me check what frontier detection uses - specifically what happens when getOccupancy returns UNDEROBSERVED in the frontier detection 
  code that's still active. Looking at the frontier finder, it's still counting UNDEROBSERVED voxels separately, which means removing the 
  enum would break that code path entirely. So the real issue is that frontier detection is still treating UNDEROBSERVED voxels as frontier 
  candidates even though active_perception isn't prioritizing them anymore. Looking at the occupancy logic more carefully, if I remove the 
  UNDEROBSERVED case entirely, those voxels with high occupancy but distant observations would fall through to return OCCUPIED instead, which
   would incorrectly treat them as obstacles. That's worse than the current behavior. I need to think about whether changing UNDEROBSERVED to
   FREE would be better, or if there's a different approach to breaking the frontier cycle. The issue is that these unresolved frontiers keep
   the UAV circling endlessly. The user's suggestion to comment out UNDEROBSERVED won't work directly since the getOccupancy function still 
  needs to handle those voxels, so the real question is whether to change getOccupancy to return OCCUPIED or FREE instead of UNDEROBSERVED. I
   need to examine how frontier_finder.cpp actually uses UNDEROBSERVED in the frontier detection logic to understand what impact this change 
  would have.

● Read frontier_finder.cpp
  │ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp
  └ L1610:1640 (31 lines read)

● Read frontier_finder.cpp
  │ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp
  └ L1790:1815 (26 lines read)

● Read frontier_finder.cpp
  │ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp
  └ L75:130 (56 lines read)

◐ Now let me look at the frontier detection logic - specifically how frontiers are detected - to understand what kind of voxels form frontier
   boundaries. I need to see the `searchFrontiers` or equivalent function that determines what constitutes a frontier cell.

● Search (grep)
  │ "isNeighborUnknown|isNeighborUnderObserved|isFrontier|UNKNOWN.*frontier|frontier.*UNKNOWN""
  │ (src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp)
  └ 18 lines found

● Search (grep)
  │ "isNeighborUnknown|isUnknownNeighbor|searchFrontier\|updateFrontier\|isFrontierCell""
  │ (src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp)
  └ 4 lines found

● Read frontier_finder.cpp
  │ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp
  └ L199:220 (22 lines read)

● Read frontier_finder.cpp
  │ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp
  └ L300:360 (61 lines read)

◐ Now I have a clear picture. Let me analyze the situation:

  1. `isFrontierCellActive` returns true if a free voxel has an UNDEROBSERVED or UNKNOWN neighbor.
  2. `expandFrontier` creates SURFACEFTR (surface frontier) when the seed cell has an UNDEROBSERVED neighbor, and UNKNOWNFTR when it has only
   UNKNOWN neighbors.
  3. Both types of frontiers propagate through cells that are `knownfree` AND (have an UNDEROBSERVED OR UNKNOWN neighbor).

  The user's question: Should they comment out UNDEROBSERVED from getOccupancy (so those voxels return something else) to stop the circling 
  behavior?


  If I trace through what would happen if UNDEROBSERVED voxels were treated as OCCUPIED instead, they'd become obstacles that block frontier 
  propagation entirely—SURFACEFTR frontiers would vanish since there'd be no UNDEROBSERVED neighbors to trigger them, and only UNKNOWNFTR 
  would remain. But this would actually be the wrong fix for the problem.

  The real issue is that treating UNDEROBSERVED as OCCUPIED creates phantom obstacles that confuse the path planner, so I need to consider 
  what happens if I change UNDEROBSERVED to return FREE or UNKNOWN instead. If it returns FREE, those voxels look like safe space and 
  SURFACEFTR disappears, but now the planner might fly through actually-occupied areas. If it returns UNKNOWN, I get a middle ground where 
  those voxels stay uncertain rather than being treated as either obstacles or free space.

  Looking at what the user actually did—they removed the underobserved handling from active_perception but the UAV still circles because 
  residual SURFACEFTR frontiers are lingering. The cleanest fix might be to just remove the `isNeighborUnderObserved` check from the frontier
   finder itself, so UNDEROBSERVED voxels stop generating surface frontiers altogether while remaining in the map as-is.

  But if they want to comment out UNDEROBSERVED from the occupancy definition instead, I need to trace through what happens—if we remove that
   return statement, the code would fall through to the next condition rather than explicitly marking cells as underobserved. Commenting out 
  that return statement would cause those high-occupancy, far-observation voxels to fall through and return FREE instead of UNDEROBSERVED, 
  which would break the frontier detection logic—no more SURFACEFTR frontiers would be identified, and cells adjacent to underobserved voxels
   would stop being marked as active.

  The tradeoff is that this workaround eliminates the residual frontiers but risks making the path planner treat occupied-but-distant voxels 
  as navigable, which could be problematic. Still, since the sensor model isn't generating UNDEROBSERVED states correctly in the simulation 
  anyway, this seems like a reasonable compromise.

● 现在我有完整的图景了，来直接回答你的问题。

  --------------------------------------------------------------------------------
  --------------------------------------------------------

  影响链路分析

  UNDEROBSERVED 在 frontier 生成中的作用如下：

   getOccupancy() → UNDEROBSERVED
           ↓
   isNeighborUnderObserved()
           ↓
   isFrontierCellActive() ← 这里决定一个 free voxel 是否是 frontier cell
     (line 208) if (isNeighborUnderObserved(idx)) return true;
           ↓
   expandFrontier()
     (line 310) isNeighborUnderObserved(first) → ftr_type = SURFACEFTR

  --------------------------------------------------------------------------------
  --------------------------------------------------------

  直接回答你的问题

  注释掉 return UNDEROBSERVED 这一行（在 getOccupancy 里），那些 voxel 会 fall 
  through 变成 FREE，效果是：

   - ✅ isNeighborUnderObserved 全部返回 false → SURFACEFTR 类型 frontier 完全消失
   - ✅ 残留的 SURFACEFTR frontier 在下次 isFrontierChanged 
  检查时会被删除（isFrontierCellActive 返回 false）
   - ✅ 绕圈问题很可能缓解

  但有一个隐患：
  原来 UNDEROBSERVED 的 voxel（occupancy 高但观测距离远）现在变成 
  FREE，路径规划器会认为那里是空旷的，可能穿越实际上应该是障碍物的区域。

  --------------------------------------------------------------------------------
  --------------------------------------------------------

  更推荐的做法（更精准）

  不改 getOccupancy，只在 isFrontierCellActive 里去掉 UNDEROBSERVED 的触发：

   // frontier_finder.cpp, line 201
   bool FrontierFinder::isFrontierCellActive(const Eigen::Vector3i &idx) {
     if (!knownfree(idx))
       return false;
   
     if (isNeighborUnknown(idx))
       return true;
   
     // if (isNeighborUnderObserved(idx))  // 注释掉这两行
     //   return true;
   
     return false;
   }

  这样：

   - UNDEROBSERVED 在地图语义上仍然是"可疑占用"（isNearObstacle 
  仍然生效，视点不会选在其旁边）
   - 但不再生成/维持 SURFACEFTR frontier
   - 对绕圈问题最精准，对路径规划无副作用



4.2 

  问题1：飞机飞出地图

  office.pcd 实际尺寸 vs 边界框：

  ┌────┬─────────────────┬──────────────┬───────────────────────────────────────────────────────┐
  │ 轴 │ office.pcd 范围 │ 边界框 (box) │ 差距                                                  │
  ├────┼─────────────────┼──────────────┼───────────────────────────────────────────────────────┤
  │ X  │ [-7.97, 7.67]   │ [-10, 10]    │ 边界多出 ~2m → 办公室外是 UNKNOWN → 产生 frontier     │
  ├────┼─────────────────┼──────────────┼───────────────────────────────────────────────────────┤
  │ Y  │ [-15.12, 15.18] │ [-15, 15]    │ 基本匹配 ✓                                            │
  ├────┼─────────────────┼──────────────┼───────────────────────────────────────────────────────┤
  │ Z  │ [-0.50, 2.40]   │ [0, 2]       │ 基本匹配 ✓                                            │
  └────┴─────────────────┴──────────────┴───────────────────────────────────────────────────────┘

  根因： X 方向边界框比办公室大约 2m，办公室墙外的 UNKNOWN 空间会生成 frontier，引诱飞机飞出去。

  还有一个冲突： expandFrontier() 中硬编码 pos[2] > 6.5 限制，与 box_max_z = 2.0 矛盾。

  问题2：轻微往复/绕圈

  排查发现 3 个关键因素：

  ┌─────────────────────────────────┬────────┬───────────────────────────────────────────────────────────┐
  │ 因素                            │ 当前值 │ 问题                                                      │
  ├─────────────────────────────────┼────────┼───────────────────────────────────────────────────────────┤
  │ thresh_replan2                  │ 0.5s   │ 太激进，执行 0.5s 就可能 replan                           │
  ├─────────────────────────────────┼────────┼───────────────────────────────────────────────────────────┤
  │ thresh_replan3                  │ 1.5s   │ 周期 replan 太频繁                                        │
  ├─────────────────────────────────┼────────┼───────────────────────────────────────────────────────────┤
  │ min_view_finish_fraction        │ 0.45   │ 45% 覆盖就算"完成"，部分覆盖的 frontier 反复触发重访      │
  ├─────────────────────────────────┼────────┼───────────────────────────────────────────────────────────┤
  │ kRecentClusterFarThreshold      │ 5.0m   │ 飞机在历史点 5m 内时抑制失效                              │
  ├─────────────────────────────────┼────────┼───────────────────────────────────────────────────────────┤
  │ kCommittedClusterSwitchMargin   │ 2.5    │ 目标切换门槛太低，容易在两个相似代价的 cluster 间跳来跳去 │
  └─────────────────────────────────┴────────┴───────────────────────────────────────────────────────────┘

  -----------------------------------------------------------------------------------------------------------------------------

  要我帮你修改吗？推荐以下改动：

   1. 收紧边界框 exploration.launch: X 从 ±10 改为 ±8
   2. 修复硬编码 Z 限制 frontier_finder.cpp:
    6.5 → 与 box_max_z 一致
   3. 调大 replan 阈值 algorithm.xml: thresh_replan2=2.0, thresh_replan3=3.0
   4. 调大覆盖率门槛 algorithm.xml: min_view_finish_fraction=0.70
   5. 调大反振荡参数 fast_exploration_manager.cpp: kRecentClusterFarThreshold=8.0, kCommittedClusterSwitchMargin=5.0

● Asked user 要我把这些修改都做上去吗？还是你想只改其中几项？
  └ User selected: 全部修改 (推荐)

● Edit exploration.launch +9 -2
  └ src/FUEL/fuel_planner/exploration_manager/launch/exploration.launch

● Edit frontier_finder.cpp +3 -1
  └ src/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp

● Edit algorithm.xml +3 -2
  └ src/FUEL/fuel_planner/exploration_manager/launch/algorithm.xml

● Edit algorithm.xml +2 -1
  └ src/FUEL/fuel_planner/exploration_manager/launch/algorithm.xml

● Edit fast_exploration_manager.cpp +2 -1
  └ src/FUEL/fuel_planner/exploration_manager/src/fast_exploration_manager.cpp

● Edit fast_exploration_manager.cpp +2 -1
  └ src/FUEL/fuel_planner/exploration_manager/src/fast_exploration_manager.cpp

● 修改完成，现在编译验证：

● Build modified packages (shell)
  │ cd /home/jacob/fuel_star_ws && source devel/setup.bash && catkin_make --pkg active_perception exploration_manager 2>&1 | tail
  │ -25
  └ 1 line...

● Read shell output Waiting up to 30 seconds for command output
  └ 1 line...

● Read shell output Waiting up to 30 seconds for command output
  └ 26 lines...

● Shell "Build modified packages" completed 

● 编译成功 ✅，无新错误。

  修改总结

  共 6 处修改，涵盖 4 个文件：

  ┌─────────────────────┬────────────────────────────────┬────────────────────────────────────────┐
  │ 修改                │ 文件                           │ 改动                                   │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ 🗺️ 收紧 X 边界      │ exploration.launch             │ ±10 → ±8，匹配 office.pcd 范围         │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ 📏 修复硬编码 Z     │ frontier_finder.cpp:327        │ 6.5 → 2.0，与 box_max_z 一致           │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ ⏱️ 调大 replan 阈值 │ algorithm.xml                  │ thresh2: 0.5→2.0, thresh3: 1.5→3.0     │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ 📊 调大覆盖率门槛   │ algorithm.xml                  │ min_view_finish_fraction: 0.45→0.70    │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ 🔄 扩大抑制范围     │ fast_exploration_manager.cpp   │ kRecentClusterFarThreshold: 5→8        │
  ├─────────────────────┼────────────────────────────────┼────────────────────────────────────────┤
  │ 🎯 加大切换门槛     │ fast_exploration_manager.cpp   │ kCommittedClusterSwitchMargin: 2.5→5.0 │