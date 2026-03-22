// 诊断补丁 - 在 frontier_finder.cpp 中的 updateFrontierCostMatrix() 方法前添加

// ============ 诊断方法 START ============

void FrontierFinder::diagnoseUpdateFrontierCostMatrix() {
  ROS_WARN("========== DIAGNOSTIC: updateFrontierCostMatrix START ==========");
  ROS_WARN("Total frontiers count: %zu", frontiers_.size());
  
  int mismatches = 0;
  for (int i = 0; i < frontiers_.size(); ++i) {
    size_t costs_size = frontiers_[i].costs_.size();
    size_t paths_size = frontiers_[i].paths_.size();
    size_t viewpoints_size = frontiers_[i].viewpoints_.size();
    
    ROS_WARN("Frontier[%d]: id=%d, costs=%zu, paths=%zu, viewpoints=%zu, cells=%zu",
             i, frontiers_[i].id_, costs_size, paths_size, viewpoints_size, frontiers_[i].cells_.size());
    
    if (costs_size != paths_size) {
      ROS_ERROR("  ❌ SIZE MISMATCH DETECTED! costs != paths");
      mismatches++;
      
      // 自动调整
      size_t minSize = std::min(costs_size, paths_size);
      if (costs_size > minSize) {
        ROS_ERROR("     Resizing costs from %zu to %zu", costs_size, minSize);
        frontiers_[i].costs_.resize(minSize);
      }
      if (paths_size > minSize) {
        ROS_ERROR("     Resizing paths from %zu to %zu", paths_size, minSize);
        frontiers_[i].paths_.resize(minSize);
      }
    }
  }
  
  ROS_WARN("first_new_frt_: %d (should be -1 or 0..%zu)", first_new_frt_, frontiers_.size());
  ROS_WARN("removed_ids_ size: %zu", removed_ids_.size());
  ROS_WARN("Total mismatches found: %d", mismatches);
  ROS_WARN("========== DIAGNOSTIC END ==========");
  
  if (mismatches > 0) {
    ROS_ERROR("❌ CRITICAL: Data structure inconsistency detected!");
  }
}

// ============ 诊断方法 END ============

// 然后修改 updateFrontierCostMatrix() 的开始处：

void FrontierFinder::updateFrontierCostMatrix() {
  // ✅ 添加诊断调用
  diagnoseUpdateFrontierCostMatrix();
  
  // ... 原始代码开始
  
  auto updateCost = [](Frontier &ft1, Frontier &ft2) {
    if (ft1.viewpoints_.empty() || ft2.viewpoints_.empty()) return;
    
    // ... 原始代码
  };
  
  // ... 其余原始代码
}

// ============ 修复1：验证函数 START ============

void FrontierFinder::verifyCostPathsConsistency(Frontier &ft, const char* location) {
  if (ft.costs_.size() != ft.paths_.size()) {
    ROS_ERROR("At %s: Frontier id=%d has cost size=%zu but path size=%zu",
              location, ft.id_, ft.costs_.size(), ft.paths_.size());
    
    // 自动修复
    size_t minSize = std::min(ft.costs_.size(), ft.paths_.size());
    ft.costs_.resize(minSize);
    ft.paths_.resize(minSize);
  }
}

// ============ 修复1：验证函数 END ============

// ============ 修复2：安全的删除 START ============

void FrontierFinder::safeRemoveFrontierCosts(int frontier_idx, int cost_idx) {
  if (frontier_idx < 0 || frontier_idx >= frontiers_.size()) {
    ROS_ERROR("Invalid frontier index: %d (size: %zu)", frontier_idx, frontiers_.size());
    return;
  }
  
  Frontier &ft = frontiers_[frontier_idx];
  
  if (cost_idx < 0 || cost_idx >= ft.costs_.size()) {
    ROS_ERROR("Invalid cost index: %d for frontier %d (size: %zu)",
              cost_idx, frontier_idx, ft.costs_.size());
    return;
  }
  
  if (cost_idx >= ft.paths_.size()) {
    ROS_ERROR("Cost index %d out of paths range for frontier %d (path size: %zu)",
              cost_idx, frontier_idx, ft.paths_.size());
    return;
  }
  
  ft.costs_.erase(ft.costs_.begin() + cost_idx);
  ft.paths_.erase(ft.paths_.begin() + cost_idx);
  
  verifyCostPathsConsistency(ft, "after safeRemoveFrontierCosts");
}

// ============ 修复2：安全的删除 END ============

// ============ 修复3：初始化新 frontier START ============

void FrontierFinder::initializeFrontierCostPaths(Frontier &ft) {
  // 确保新添加的 frontier 的 costs_ 和 paths_ 被正确初始化
  if (!ft.costs_.empty() || !ft.paths_.empty()) {
    ROS_WARN("Frontier id=%d already has costs or paths, clearing them", ft.id_);
  }
  
  ft.costs_.clear();
  ft.paths_.clear();
  
  ROS_DEBUG("Initialized frontier id=%d: costs_.size()=%zu, paths_.size()=%zu",
            ft.id_, ft.costs_.size(), ft.paths_.size());
}

// ============ 修复3：初始化新 frontier END ============

// 使用方式：
// 1. 在 computeFrontiersToVisit() 中，插入新 frontier 前调用：
//    initializeFrontierCostPaths(tmp_ftr);
//
// 2. 在 updateFrontierCostMatrix() 的删除逻辑中调用：
//    safeRemoveFrontierCosts(it, remove_idx);
//
// 3. 随时调用诊断：
//    diagnoseUpdateFrontierCostMatrix();

