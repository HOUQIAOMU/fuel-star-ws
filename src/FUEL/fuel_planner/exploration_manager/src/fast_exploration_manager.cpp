// #include <fstream>
#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <traj_utils/planning_visualization.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
namespace {
double normalizeAngle(double yaw) {
  while (yaw <= -M_PI)
    yaw += 2.0 * M_PI;
  while (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  return yaw;
}

double nearestEquivalentYaw(double reference_yaw, double target_yaw) {
  double adjusted = normalizeAngle(target_yaw);
  double diff = adjusted - reference_yaw;
  while (diff <= -M_PI)
    diff += 2.0 * M_PI;
  while (diff > M_PI)
    diff -= 2.0 * M_PI;
  return reference_yaw + diff;
}

constexpr double kRecentClusterPenaltyRadius = 3.5;
constexpr double kRecentClusterFarThreshold = 5.0;
constexpr double kRecentClusterPenaltyBase = 12.0;
constexpr double kRecentClusterMinSeparation = 1.0;
constexpr size_t kRecentClusterHistoryLimit = 20;
constexpr double kClusterUtilityBonusScale = 0.02;
constexpr double kClusterUtilityBonusCap = 10.0;
constexpr double kCommittedClusterMatchRadius = 2.5;
constexpr double kCommittedClusterSwitchMargin = 2.5;
constexpr double kCommittedClusterReachThresh = 1.2;
} // namespace

// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle &nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  visualization_.reset(new PlanningVisualization(nh));

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);
  nh.param("exploration/alc_cp_search_range", ep_->alc_cp_search_range_, 10);
  nh.param("exploration/enable_fixed_hgrid", ep_->enable_fixed_hgrid_, false);
  nh.param("exploration/perception_aware_local", ep_->perception_aware_local_,
           false);
  nh.param("exploration/feature_max_dist", ep_->feature_max_dist_, -1.0);
  nh.param("exploration/inertial_cost_offset", ep_->inertial_cost_offset_,
           -1.0);
  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);
  nh.param("exploration/verbose_active_loop", ep_->verbose_active_loop_, false);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  // planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // // planner_manager_->path_finder_->max_search_time_ = 0.05;
  // planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  ofstream par_file_hgrid(ep_->tsp_dir_ + "/single_hgrid.par");
  par_file_hgrid << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single_hgrid.tsp\n";
  par_file_hgrid << "GAIN23 = NO\n";
  par_file_hgrid << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_
                 << "/single_hgrid.txt\n";
  par_file_hgrid << "RUNS = 1\n";

  ofstream par_file_frontier(ep_->tsp_dir_ + "/single_frontier.par");
  par_file_frontier << "PROBLEM_FILE = " << ep_->tsp_dir_
                    << "/single_frontier.tsp\n";
  par_file_frontier << "GAIN23 = NO\n";
  par_file_frontier << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_
                    << "/single_frontier.txt\n";
  par_file_frontier << "RUNS = 1\n";

  ofstream par_file_cluster(ep_->tsp_dir_ + "/cluster.par");
  par_file_cluster << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/cluster.tsp\n";
  par_file_cluster << "GAIN23 = NO\n";
  par_file_cluster << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/cluster.txt\n";
  par_file_cluster << "RUNS = 1\n";

  has_committed_cluster_ = false;
  committed_cluster_center_.setZero();
}

int FastExplorationManager::planExploreMotionCluster(const Vector3d &pos,
                                                     const Vector3d &vel,
                                                     const Vector3d &acc,
                                                     const Vector3d &yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Search frontiers and group them into clusters
  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones'
  // info
  frontier_finder_->searchFrontiers(pos);

  frontier_finder_->computeFrontiersToVisit(pos);
  // frontier_finder_->updateFrontierCostMatrix();
  bool neighbor;
  frontier_finder_->clusterFrontiers(pos, neighbor);
  frontier_finder_->printLocalMapUnknownStats(pos, "planExploreMotionCluster");

  frontier_finder_->getFrontiers(ed_->frontiers_);
  // frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  // frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_,
                                         ed_->averages_);
  double frt_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[planExploreMotionCluster] frt_time:%lf", frt_time);

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }
  vector<vector<Eigen::Vector3d>> division_clusters;
  frontier_finder_->getFrontierDivision(division_clusters); //分割后的frontier cluster
  Eigen::Vector3d next_cluster_pos;
  if (division_clusters.size() > 1) {
    findNextCluster(pos, vel, yaw, ed_->local_tour_, next_cluster_pos,
                    neighbor);
  } else {
    vector<int> indices;
    indices.push_back(0);
    frontier_finder_->getClusterTour(indices, ed_->global_tour_);
    next_cluster_pos = ed_->global_tour_[0];
    frontier_finder_->getCheckTour(0, pos, next_cluster_pos, ed_->local_tour_);
    frontier_finder_->printClusterDebugInfo(0, "single_cluster");
    frontier_finder_->printActiveFrontierDebugInfo("single_cluster");
  }

  // for visualize
  ed_->grid_tour_.clear();
  ed_->grid_tour_.push_back(pos);
  ed_->inertia_tour_.insert(ed_->inertia_tour_.begin(), pos);
  for (auto p : ed_->global_tour_)
    ed_->grid_tour_.push_back(p);

  // Do global and local tour planning and retrieve the next viewpoint
  Vector3d next_pos;
  vector<double> next_yaw;
  const double arrived_thresh = 0.15;
  const double cluster_switch_thresh = 0.3;
  const double useful_path_thresh = 1.2;
  const double short_yaw_path_thresh = 1.5;
  vector<int> local_candidates;

  auto assign_local_candidate = [&](int candidate_index) {
    next_pos = ed_->local_tour_[candidate_index].pos_;
    next_yaw = ed_->local_tour_[candidate_index].yaws_;
    ed_->local_tour_vis_.clear();
    ed_->local_tour_vis_.push_back(pos);
  };

  if (ed_->local_tour_.size() > 1) {
    vector<int> indices;
    const int local_tour_size = static_cast<int>(ed_->local_tour_.size());

    findLocalTour(pos, vel, yaw, next_cluster_pos, indices);
    if (indices.empty() || indices[0] >= local_tour_size) {
      ROS_ERROR("Invalid local tour index");
      return FAIL;
    }

    for (const auto &tour_index : indices) {
      if (tour_index < 0 || tour_index >= local_tour_size)
        continue;
      if ((ed_->local_tour_[tour_index].pos_ - pos).norm() >= arrived_thresh)
        local_candidates.push_back(tour_index);
    }
    if (local_candidates.empty()) {
      if ((next_cluster_pos - pos).norm() > cluster_switch_thresh) {
        ROS_WARN("All local viewpoints are too close, switch to next cluster center");
        next_pos = next_cluster_pos;
        next_yaw = {atan2(next_cluster_pos.y() - pos.y(),
                          next_cluster_pos.x() - pos.x())};
        ed_->local_tour_vis_.clear();
        ed_->local_tour_vis_.push_back(pos);
        ed_->local_tour_vis_.push_back(next_cluster_pos);
      } else {
        ROS_WARN("All local viewpoints are within arrived threshold, force frontier update");
        frontier_finder_->searchFrontiers(pos);
        frontier_finder_->computeFrontiersToVisit(pos);
        bool neighbor;
        frontier_finder_->clusterFrontiers(pos, neighbor);
        return FAIL;
      }
    }
    if (next_yaw.empty()) {
      assign_local_candidate(local_candidates.front());
      for (size_t i = 0; i + 1 < indices.size(); ++i) {
        ed_->local_tour_vis_.push_back(ed_->local_tour_[indices[i]].pos_);
      }
      ed_->local_tour_vis_.push_back(next_cluster_pos);
    }
  } else if (ed_->local_tour_.size() == 1) {
    if ((ed_->local_tour_[0].pos_ - pos).norm() < arrived_thresh) {
      if ((next_cluster_pos - pos).norm() > cluster_switch_thresh) {
        ROS_WARN("Single local viewpoint already reached, switch to next cluster center");
        next_pos = next_cluster_pos;
        next_yaw = {atan2(next_cluster_pos.y() - pos.y(),
                          next_cluster_pos.x() - pos.x())};
        ed_->local_tour_vis_.clear();
        ed_->local_tour_vis_.push_back(pos);
        ed_->local_tour_vis_.push_back(next_cluster_pos);
      } else {
        ROS_WARN("Single local viewpoint already reached, force frontier update");
        frontier_finder_->searchFrontiers(pos);
        frontier_finder_->computeFrontiersToVisit(pos);
        bool neighbor;
        frontier_finder_->clusterFrontiers(pos, neighbor);
        return FAIL;
      }
    } else {
      next_pos = ed_->local_tour_[0].pos_;
      next_yaw = ed_->local_tour_[0].yaws_;
      ed_->local_tour_vis_.clear();
      ed_->local_tour_vis_.push_back(pos);
      ed_->local_tour_vis_.push_back(ed_->local_tour_[0].pos_);
    }

  } else
    ROS_ERROR("Empty destination.");

  std::cout << "Next view: " << next_pos.transpose() << std::endl;

  // Plan trajectory (position and yaw) to the next viewpoint
  t1 = ros::Time::now();

  double target_yaw = 0.0;
  double min_yaw = 0.0;
  double max_yaw = 0.0;
  auto update_target_yaw = [&]() {
    if (next_yaw.empty())
      return false;
    target_yaw = nearestEquivalentYaw(yaw[0], next_yaw.front());
    min_yaw = target_yaw;
    max_yaw = target_yaw;
    for (const auto &yaw_candidate : next_yaw) {
      const double adjusted_yaw = nearestEquivalentYaw(yaw[0], yaw_candidate);
      if (fabs(adjusted_yaw - yaw[0]) < fabs(target_yaw - yaw[0]))
        target_yaw = adjusted_yaw;
      min_yaw = min(min_yaw, adjusted_yaw);
      max_yaw = max(max_yaw, adjusted_yaw);
    }
    return true;
  };

  if (next_yaw.empty()) {
    ROS_ERROR("Empty target yaw set");
    return FAIL;
  }
  update_target_yaw();

  // Generate trajectory of x,y,z
  const double map_resolution = sdf_map_->getResolution();
  auto point_is_searchable = [&](const Vector3d &point) {
    return sdf_map_->isInBox(point) && sdf_map_->getInflateOccupancy(point) != 1 &&
           sdf_map_->getDistance(point) > 0.5 * map_resolution;
  };

  auto project_to_searchable = [&](const Vector3d &origin_point,
                                   const Vector3d &bias_dir,
                                   Vector3d &projected_point) {
    if (point_is_searchable(origin_point)) {
      projected_point = origin_point;
      return true;
    }

    Vector3d search_dir = bias_dir;
    if (search_dir.norm() > 1e-3)
      search_dir.normalize();

    const vector<double> radii = {map_resolution, 2.0 * map_resolution,
                                  3.0 * map_resolution, 5.0 * map_resolution,
                                  8.0 * map_resolution};
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const double radius : radii) {
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0)
              continue;
            Vector3d offset(dx * radius, dy * radius, dz * 0.5 * radius);
            Vector3d sample = origin_point + offset;
            if (!point_is_searchable(sample))
              continue;

            double score = offset.norm();
            if (search_dir.norm() > 1e-3) {
              const double alignment = offset.normalized().dot(search_dir);
              score -= 0.3 * alignment;
            }
            if (score < best_score) {
              best_score = score;
              projected_point = sample;
              found = true;
            }
          }
        }
      }
      if (found)
        return true;
    }
    return false;
  };

  auto search_path_to = [&](const Vector3d &goal,
                            vector<Eigen::Vector3d> &path_out) {
    Vector3d search_start = pos;
    Vector3d search_goal = goal;
    const Vector3d goal_dir = goal - pos;

    if (!project_to_searchable(pos, goal_dir, search_start)) {
      ROS_WARN("Current position is not in free space for A*, cannot project to a valid start");
      return false;
    }
    if (!project_to_searchable(goal, pos - goal, search_goal)) {
      ROS_WARN("Goal position is not in free space for A*, cannot project to a valid goal");
      return false;
    }
    if ((search_start - pos).norm() > 1e-3 || (search_goal - goal).norm() > 1e-3) {
      ROS_WARN("Adjust path endpoints for search: start %.2f %.2f %.2f -> %.2f %.2f %.2f, goal %.2f %.2f %.2f -> %.2f %.2f %.2f",
               pos.x(), pos.y(), pos.z(),
               search_start.x(), search_start.y(), search_start.z(),
               goal.x(), goal.y(), goal.z(),
               search_goal.x(), search_goal.y(), search_goal.z());
    }

    planner_manager_->path_finder_->reset();
    for (const double max_search_time : {0.001, 0.008, 0.02}) {
      planner_manager_->path_finder_->setMaxSearchTime(max_search_time);
      if (planner_manager_->path_finder_->search(search_start, search_goal) == Astar::REACH_END) {
        path_out = planner_manager_->path_finder_->getPath();
        if ((path_out.front() - pos).norm() > 1e-3)
          path_out.insert(path_out.begin(), pos);
        if ((path_out.back() - goal).norm() > 1e-3)
          path_out.push_back(goal);
        return true;
      }
    }
    return false;
  };

  vector<Eigen::Vector3d> searched_path;
  bool path_found = false;
  double searched_path_len = -1.0;

  auto evaluate_goal = [&](const Vector3d &goal,
                           vector<Eigen::Vector3d> &path_out,
                           double &path_len) {
    if (!search_path_to(goal, path_out))
      return false;
    path_len = Astar::pathLength(path_out);
    return true;
  };

  if (!local_candidates.empty() && ed_->local_tour_.size() > 1) {
    vector<Eigen::Vector3d> best_short_path;
    double best_short_len = -1.0;
    int best_short_index = -1;
    for (const auto &candidate_index : local_candidates) {
      assign_local_candidate(candidate_index);
      update_target_yaw();

      vector<Eigen::Vector3d> candidate_path;
      double candidate_len = -1.0;
      if (!evaluate_goal(next_pos, candidate_path, candidate_len)) {
        ROS_WARN("Local viewpoint %d unreachable, skip", candidate_index);
        continue;
      }
      if (candidate_len >= useful_path_thresh) {
        searched_path = candidate_path;
        searched_path_len = candidate_len;
        path_found = true;
        break;
      }
      if (candidate_len > best_short_len) {
        best_short_path = candidate_path;
        best_short_len = candidate_len;
        best_short_index = candidate_index;
      }
    }
    if (!path_found && best_short_index >= 0) {
      ROS_WARN("Only short local paths are available, use candidate %d", best_short_index);
      assign_local_candidate(best_short_index);
      update_target_yaw();
      searched_path = best_short_path;
      searched_path_len = best_short_len;
      path_found = true;
    }
  } else {
    path_found = evaluate_goal(next_pos, searched_path, searched_path_len);
  }

  if (( !path_found || searched_path_len < useful_path_thresh) &&
      (next_cluster_pos - pos).norm() > cluster_switch_thresh) {
    next_pos = next_cluster_pos;
    next_yaw = {atan2(next_cluster_pos.y() - pos.y(),
                      next_cluster_pos.x() - pos.x())};
    update_target_yaw();

    vector<Eigen::Vector3d> cluster_path;
    double cluster_path_len = -1.0;
    if (evaluate_goal(next_pos, cluster_path, cluster_path_len) &&
        cluster_path_len > searched_path_len + 0.1) {
      ROS_WARN("Use next cluster center instead of short local path");
      searched_path = cluster_path;
      searched_path_len = cluster_path_len;
      path_found = true;
    }
  }
  if (!path_found) {
    ROS_ERROR("No path to next viewpoint or next cluster center, refresh frontier");
    frontier_finder_->searchFrontiers(pos);
    frontier_finder_->computeFrontiersToVisit(pos);
    bool neighbor;
    frontier_finder_->clusterFrontiers(pos, neighbor);
    return FAIL;
  }

  ed_->path_next_goal_ = searched_path;
  shortenPath(ed_->path_next_goal_);

  if (searched_path_len < short_yaw_path_thresh &&
      (next_cluster_pos - pos).norm() > searched_path_len + 0.3) {
    ROS_WARN("Short local motion detected, align yaw with next cluster center");
    next_yaw = {atan2(next_cluster_pos.y() - pos.y(),
                      next_cluster_pos.x() - pos.x())};
    update_target_yaw();
  }

  // Compute time lower bound of yaw and use in trajectory generation
  double diff = fabs(min_yaw - yaw[0]);
  const double radius_far = 5.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);

  double yaw_time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;
  double pos_time_lb = len / ViewNode::vm_;
  double min_time_lb = 0.2 * M_PI / ViewNode::yd_;
  double time_lb =
      max(yaw_time_lb,
          pos_time_lb); // max(max(yaw_time_lb, pos_time_lb), min_time_lb);

  std::cout << "Size of Path_Next_Goal:" << ed_->path_next_goal_.size() << ","
            << " Path length:" << len << std::endl;

  if (len < radius_far) {
    // Next viewpoint is very close, no need to search kinodynamic path, just
    // use waypoints-based optimization
    if (ed_->path_next_goal_.size() < 2) {
      ROS_WARN("Path too short, frontier reached, force update");
  // 强制重新搜索 frontier，让当前位置附近的 frontier 被消除
      frontier_finder_->searchFrontiers(pos);
      frontier_finder_->computeFrontiersToVisit(pos);
      bool neighbor;
      frontier_finder_->clusterFrontiers(pos, neighbor);
      return FAIL;
    }
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
  } else {
    // Next viewpoint is far away, select intermediate goal on geometric path
    // (this also deal with dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    if (ed_->path_next_goal_.size() < 2) {
      ROS_WARN("Path too short, frontier reached, force update");
      // 强制重新搜索 frontier，让当前位置附近的 frontier 被消除
      frontier_finder_->searchFrontiers(pos);
      frontier_finder_->computeFrontiersToVisit(pos);
      bool neighbor;
      frontier_finder_->clusterFrontiers(pos, neighbor);
      return FAIL;
    }
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
    // if (!planner_manager_->kinodynamicReplan(
    //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
    //   return FAIL;
    // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() <
      time_lb - 0.1) {
    ROS_ERROR("Lower bound not satified!");
    ROS_ERROR("Yaw_tlb:%f, Pos_tlb:%f, Traj_time:%f", yaw_time_lb, pos_time_lb,
              planner_manager_->local_data_.position_traj_.getTimeSum());
  }

  planner_manager_->planYawExplore(yaw, target_yaw, true, ep_->relax_time_);

  double traj_plan_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  double total = (ros::Time::now() - t2).toSec();
  ROS_WARN("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
  if (next_yaw.size() > 1) {
    planner_manager_->local_data_.spiral_max_yaw_ = max_yaw;
    planner_manager_->local_data_.spiral_min_yaw_ = min_yaw;
    planner_manager_->local_data_.need_spiral_ = true;
  } else {
    planner_manager_->local_data_.need_spiral_ = false;
  }
  return SUCCEED;
}

void FastExplorationManager::shortenPath(vector<Vector3d> &path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = {path.front()};
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3)
    short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1,
                      0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findNextCluster(const Vector3d &cur_pos,
                                             const Vector3d &cur_vel,
                                             const Vector3d &cur_yaw,
                                             vector<checkPoint> &check_tour,
                                             Eigen::Vector3d &next_cluster_pos,
                                             const bool neighbor) {

  auto t1 = ros::Time::now();

  Eigen::MatrixXd cost_matrix;
  frontier_finder_->getClusterMatrix(cur_pos, cur_vel, cur_yaw, cost_matrix);
  if (neighbor)
    cost_matrix(0, 1) = 2.0;

  vector<Eigen::Vector3d> centers;
  frontier_finder_->getClusterCenter(centers);
  applyClusterUtilityBonus(centers, cost_matrix);
  applyRecentClusterSuppression(cur_pos, centers, cost_matrix);
  vector<int> inertial_indices, TSP_indices;
  double inertial_cost = std::numeric_limits<double>::max(),
         TSP_cost = std::numeric_limits<double>::max();
  updateInertialTour(cur_pos, ed_->global_tour_, centers, cost_matrix,
                     inertial_indices, inertial_cost);

  TSPConfig cluster_config;
  cluster_config.dimension_ = cost_matrix.rows();
  cluster_config.problem_name_ = "cluster";
  cluster_config.skip_first_ = true;
  cluster_config.skip_last_ = false;
  cluster_config.result_id_offset_ = 2;
  solveTSP(cost_matrix, cluster_config, TSP_indices, TSP_cost);

  vector<int> indices;

  // ROS_WARN("[findNextCluster]Inertial cost : %lf, TSP cost : %lf",
          //  inertial_cost * ViewNode::vm_, TSP_cost * ViewNode::vm_);
  if (inertial_cost * ViewNode::vm_ <
      TSP_cost * ViewNode::vm_ + ep_->inertial_cost_offset_) {
    indices = inertial_indices;
    frontier_finder_->getClusterTour(inertial_indices, ed_->global_tour_);
    // ROS_WARN("[findNextCluster] Using Inertial tour");
  } else {
    indices = TSP_indices;
    frontier_finder_->getClusterTour(TSP_indices, ed_->global_tour_);
    // ROS_WARN("[findNextCluster] Using TSP tour");
  }

  const double cluster_arrived_thresh = 0.3;
  int current_order = 0;
  vector<checkPoint> candidate_tour;
  for (size_t order = 0; order < indices.size(); ++order) {
    const int next_order =
        min(static_cast<int>(order) + 1, static_cast<int>(indices.size()) - 1);
    const Vector3d candidate_next_cluster_pos = centers[indices[next_order]];
    frontier_finder_->getCheckTour(indices[order], cur_pos,
                                   candidate_next_cluster_pos, candidate_tour);
    bool cluster_reached = true;
    for (const auto &checkpoint : candidate_tour) {
      if ((checkpoint.pos_ - cur_pos).norm() > cluster_arrived_thresh) {
        cluster_reached = false;
        break;
      }
    }
    if (!cluster_reached || order == indices.size() - 1) {
      current_order = static_cast<int>(order);
      check_tour = candidate_tour;
      break;
    }
  }

  current_order = applyGlobalTargetCommitment(cur_pos, indices, centers, cost_matrix,
                                              current_order, cluster_arrived_thresh);

  const int next_order = min(current_order + 1, static_cast<int>(indices.size()) - 1);
  next_cluster_pos = centers[indices[next_order]];
  frontier_finder_->getCheckTour(indices[current_order], cur_pos, next_cluster_pos,
                                 check_tour);
  updateRecentClusterHistory(centers[indices[current_order]]);
  frontier_finder_->printClusterDebugInfo(indices[current_order], "selected_cluster");
  frontier_finder_->printActiveFrontierDebugInfo("selected_cluster");
  ROS_WARN("[findNextCluster] cluster num: %zu, indices[%d]: %d, next_cluster: %.2f %.2f %.2f",
    frontier_finder_->frontier_clusters_.size(),
    current_order,
    indices[current_order],
    next_cluster_pos.x(), next_cluster_pos.y(), next_cluster_pos.z());

  double cal_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[findNextCluster] Calculation Time: %f", cal_time);
}

int FastExplorationManager::applyGlobalTargetCommitment(
    const Vector3d &cur_pos, const vector<int> &indices,
    const vector<Vector3d> &cluster_centers,
    const Eigen::MatrixXd &cluster_cost_matrix, int current_order,
    double cluster_arrived_thresh) {
  if (indices.empty() || cluster_centers.empty())
    return current_order;

  /*
   * Global target commitment keeps the current high-level exploration target
   * stable across several replans.
   *
   * Why this is needed:
   * - The frontier map changes at every sensing update.
   * - If global planning always picks the instantaneously best cluster, the
   *   selected target can oscillate between two sides of the map.
   * - That oscillation shows up as large arcs, heading reversals, and
   *   "fly a bit -> replan -> fly back" behavior.
   *
   * The rule implemented here is intentionally conservative:
   * 1. If there is no committed cluster yet, accept the current best one.
   * 2. If the old committed cluster disappeared from the latest cluster list,
   *    release the commitment and accept the new best one.
   * 3. If the committed cluster is already close enough to the UAV, release it
   *    so the planner can naturally switch to the next target.
   * 4. Otherwise, keep following the committed cluster unless the new best
   *    cluster is better by a clear margin.
   *
   * In short: do not switch the global exploration direction for a tiny cost
   * change; only switch when there is a clearly better reason.
   */
  if (!has_committed_cluster_) {
    return current_order;
  }

  int committed_order = -1;
  double committed_match_dist = std::numeric_limits<double>::infinity();
  for (size_t order = 0; order < indices.size(); ++order) {
    const double center_dist =
        (cluster_centers[indices[order]] - committed_cluster_center_).norm();
    if (center_dist < committed_match_dist) {
      committed_match_dist = center_dist;
      committed_order = static_cast<int>(order);
    }
  }

  if (committed_order < 0 || committed_match_dist > kCommittedClusterMatchRadius) {
    ROS_WARN(
        "[findNextCluster] release committed cluster: no matched cluster in current frontier set, match_dist=%.2f",
        committed_match_dist);
    has_committed_cluster_ = false;
    return current_order;
  }

  const Vector3d committed_center = cluster_centers[indices[committed_order]];
  const double dist_to_committed = (cur_pos - committed_center).norm();
  if (dist_to_committed <= max(cluster_arrived_thresh, kCommittedClusterReachThresh)) {
    ROS_WARN(
        "[findNextCluster] release committed cluster: committed target reached, dist=%.2f",
        dist_to_committed);
    has_committed_cluster_ = false;
    return current_order;
  }

  const double current_cost =
      cluster_cost_matrix(0, indices[current_order] + 1);
  const double committed_cost =
      cluster_cost_matrix(0, indices[committed_order] + 1);

  if (current_order != committed_order &&
      current_cost + kCommittedClusterSwitchMargin >= committed_cost) {
    ROS_WARN(
        "[findNextCluster] keep committed cluster order=%d center=(%.2f %.2f %.2f), committed_cost=%.2f, challenger_order=%d, challenger_cost=%.2f",
        committed_order, committed_center.x(), committed_center.y(),
        committed_center.z(), committed_cost, current_order, current_cost);
    return committed_order;
  }

  if (current_order != committed_order) {
    ROS_WARN(
        "[findNextCluster] switch committed cluster: challenger is clearly better, old_cost=%.2f, new_cost=%.2f",
        committed_cost, current_cost);
  }

  return current_order;
}

void FastExplorationManager::applyRecentClusterSuppression(
    const Vector3d &cur_pos, const vector<Vector3d> &cluster_centers,
    Eigen::MatrixXd &cluster_cost_matrix) {
  if (recent_cluster_history_.empty() || cluster_centers.empty())
    return;

  for (size_t cluster_index = 0; cluster_index < cluster_centers.size(); ++cluster_index) {
    double penalty = 0.0;
    double nearest_history_dist = std::numeric_limits<double>::infinity();
    Vector3d nearest_history = Vector3d::Zero();

    for (const auto &history_center : recent_cluster_history_) {
      const double cur_to_history = (cur_pos - history_center).norm();
      if (cur_to_history < kRecentClusterFarThreshold)
        continue;

      const double cluster_to_history =
          (cluster_centers[cluster_index] - history_center).norm();
      if (cluster_to_history >= kRecentClusterPenaltyRadius)
        continue;

      const double candidate_penalty =
          kRecentClusterPenaltyBase *
          pow(1.0 - cluster_to_history / kRecentClusterPenaltyRadius, 2.0);
      if (candidate_penalty > penalty) {
        penalty = candidate_penalty;
        nearest_history_dist = cluster_to_history;
        nearest_history = history_center;
      }
    }

    if (penalty > 0.0) {
      cluster_cost_matrix(0, static_cast<int>(cluster_index) + 1) += penalty;
      ROS_WARN(
          "[findNextCluster] suppress revisit for cluster %zu center=(%.2f %.2f %.2f), penalty=%.2f, nearest_history=(%.2f %.2f %.2f), dist=%.2f",
          cluster_index, cluster_centers[cluster_index].x(),
          cluster_centers[cluster_index].y(), cluster_centers[cluster_index].z(),
          penalty, nearest_history.x(), nearest_history.y(), nearest_history.z(),
          nearest_history_dist);
    }
  }
}

void FastExplorationManager::applyClusterUtilityBonus(
    const vector<Vector3d> &cluster_centers, Eigen::MatrixXd &cluster_cost_matrix) {
  vector<double> utilities;
  frontier_finder_->getClusterUtilities(utilities);
  if (utilities.size() != cluster_centers.size())
    return;

  for (size_t cluster_index = 0; cluster_index < utilities.size(); ++cluster_index) {
    const double bonus =
        min(kClusterUtilityBonusCap, kClusterUtilityBonusScale * utilities[cluster_index]);
    if (bonus <= 1e-6)
      continue;

    cluster_cost_matrix(0, static_cast<int>(cluster_index) + 1) =
        max(0.0, cluster_cost_matrix(0, static_cast<int>(cluster_index) + 1) - bonus);
    ROS_WARN(
        "[findNextCluster] utility bonus for cluster %zu center=(%.2f %.2f %.2f), utility=%.0f, bonus=%.2f",
        cluster_index, cluster_centers[cluster_index].x(),
        cluster_centers[cluster_index].y(), cluster_centers[cluster_index].z(),
        utilities[cluster_index], bonus);
  }
}

void FastExplorationManager::updateRecentClusterHistory(
    const Vector3d &cluster_center) {
  // Update the committed global target after the final cluster is selected.
  // This state is reused in the next planning cycle to avoid frequent
  // left-right switching of the high-level exploration direction.
  has_committed_cluster_ = true;
  committed_cluster_center_ = cluster_center;

  if (!recent_cluster_history_.empty() &&
      (recent_cluster_history_.back() - cluster_center).norm() <
          kRecentClusterMinSeparation) {
    return;
  }

  recent_cluster_history_.push_back(cluster_center);
  if (recent_cluster_history_.size() > kRecentClusterHistoryLimit)
    recent_cluster_history_.erase(recent_cluster_history_.begin());
}

void FastExplorationManager::findLocalTour(const Vector3d &cur_pos,
                                           const Vector3d &cur_vel,
                                           const Vector3d cur_yaw,
                                           const Vector3d &next_cluster_pos,
                                           vector<int> &indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getCheckTourCostMatrix(cur_pos, cur_vel, cur_yaw,
                                           next_cluster_pos, cost_mat);
  const int dimension = cost_mat.rows();
  // double mat_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1) // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();
  // double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Mat_time: %lf, TSP_time: %lf", mat_time, tsp_time);
  double local_tour_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[FindLocalTour] local_tour_time: %lf", local_tour_time);
}

void FastExplorationManager::findGlobalTour(const Vector3d &cur_pos,
                                            const Vector3d &cur_vel,
                                            const Vector3d cur_yaw,
                                            vector<int> &indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " +
  // to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1) // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d &pos) {

  auto t1 = ros::Time::now();
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers(pos);

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit(pos);

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_,
                                         ed_->averages_);

  ROS_WARN("ed->points size: %ld", ed_->points_.size());
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]),
                                                           sin(ed_->yaws_[i]),
                                                           0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("[ActiveExplorationManager] No frontier");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  // ROS_INFO("[ActiveExplorationManager] Frontier search t: %.4lf, viewpoint t: "
  //          "%.4lf, cost mat t: "
  //          "%.4lf, frontier update total t: "
  //          "%.4lf",
  //          frontier_time, view_time, mat_time, total_time);
  return ed_->frontiers_.size();
}

void FastExplorationManager::refineLocalTour(
    const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
    const vector<vector<Vector3d>> &n_points,
    const vector<vector<double>> &n_yaws, vector<Vector3d> &refined_pts,
    vector<double> &refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  // ViewNode::astar_->lambda_heu_ = 1.0;
  // ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(),
                                path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  // ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time,
  // search_time, parse_time);
}

void FastExplorationManager::updateInertialTour(
    const Vector3d cur_pos, const vector<Vector3d> &last_tour,
    const vector<Vector3d> &cluster_centers,
    const Eigen::MatrixXd &cluster_cost_matrix, vector<int> &indices,
    double &inertia_cost) {
  auto t1 = ros::Time::now();
  inertia_cost = 0.0;
  if (last_tour.empty() || cluster_centers.empty()) {
    inertia_cost = 10000.0;
    return;
  }

  indices.clear();
  for (int i = 0; i < cluster_centers.size(); i++) {
    indices.push_back(i);
  }

  // min cost from every cluster centers in present tour to last tour
  vector<double> classified_min_cost;
  // the id of the centers in this tour corresponding to ones in the last
  vector<int> classified_id;
  vector<int> classified_num(last_tour.size(), 0);
  for (int i = 0; i < cluster_centers.size(); i++) {
    double minDistance = std::numeric_limits<double>::max();
    int id = -1;
    double pos_cost;
    for (int j = 0; j < last_tour.size(); j++) {
      // avoid long distance astar
      if ((cluster_centers[i] - last_tour[j]).norm() < 8.0) {
        vector<Vector3d> path;
        pos_cost = ViewNode::computeCostPos(cluster_centers[i], last_tour[j],
                                            Vector3d::Zero(), path);
      } else {
        pos_cost = 500.0 + (cluster_centers[i] - last_tour[j]).norm();
      }

      if (pos_cost < minDistance) {
        minDistance = pos_cost;
        id = j;
      }
    }
    // if (id == -1) {
    //   id = last_tour.size() - 1;
    //   minDistance = 1000.0;
    // }
    classified_min_cost.push_back(minDistance);
    classified_id.push_back(id);
    classified_num[id]++;
  }

  auto compare = [=](int id1, int id2) {
    if (classified_id[id1] != classified_id[id2])
      return classified_id[id1] < classified_id[id2];
    else
      return classified_min_cost[id1] < classified_min_cost[id2];
  };
  // memory indices for cluster_centers according to the order of classified_id
  // and pos_cost
  sort(indices.begin(), indices.end(), compare);

  // calculate TSP from indices[begin_id] to indices[end_id]
  auto calculateLocalTSP = [&](const int begin_id, const int end_id) -> double {
    // consider cur_pos for the first segment
    if (end_id == 0 && begin_id == 0)
      return cluster_cost_matrix(0, indices[0] + 1);
    // case of two neighbor feature points
    if (end_id - begin_id < 2)
      return cluster_cost_matrix(indices[begin_id] + 1, indices[end_id] + 1);

    Eigen::MatrixXd local_cost_matrix;
    vector<int> local_indices;
    vector<int> indices_copy;
    for (int i = begin_id + 1; i < end_id; i++) {
      indices_copy.push_back(indices[i]);
    }

    int dimen;
    double local_cost;
    dimen = end_id - begin_id + 1;
    local_cost_matrix = Eigen::MatrixXd::Zero(dimen, dimen);
    // cost between different cluster_centers
    for (int i = 0; i < dimen; i++) {
      for (int j = i; j < dimen; j++) {
        local_cost_matrix(i, j) = local_cost_matrix(j, i) = cluster_cost_matrix(
            indices[begin_id + i] + 1, indices[begin_id + j] + 1);
      }
    }

    // set cost to fix start point and end point
    local_cost_matrix.leftCols<1>().setZero();
    for (int i = 1; i < dimen - 1; i++) {
      local_cost_matrix(i, 0) = 65536;
      local_cost_matrix(dimen-1, i) = 65536;
    }

    TSPConfig cluster_config;
    cluster_config.dimension_ = local_cost_matrix.rows();
    cluster_config.problem_name_ = "cluster";
    cluster_config.skip_first_ = true;
    cluster_config.skip_last_ = true;
    cluster_config.result_id_offset_ = 2;
    solveTSP(local_cost_matrix, cluster_config, local_indices, local_cost);

    for (int i = 0; i < local_indices.size(); i++) {
      indices[begin_id + i + 1] = indices_copy[local_indices[i]];
    }
    return local_cost;
  };

  // find the feature centers and calculate the local TSP
  int begin_id = 0, end_id = 0;
  while (end_id < cluster_centers.size()) {
    if (classified_min_cost[indices[end_id]] < ep_->feature_max_dist_) {
      inertia_cost += calculateLocalTSP(begin_id, end_id);
      begin_id = end_id;
      end_id += classified_num[classified_id[indices[end_id]]];
    } else {
      end_id += classified_num[classified_id[indices[end_id]]];
    }
  }
  inertia_cost += calculateLocalTSP(begin_id, indices.size() - 1);

  auto cal_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[updateInertialTour] Calculation time:%f, inertia_cost:%f", cal_time,
  //          inertia_cost);
}

void FastExplorationManager::solveTSP(const Eigen::MatrixXd &cost_matrix,
                                      const TSPConfig &config,
                                      vector<int> &result_indices,
                                      double &total_cost) {
  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/" + config.problem_name_ + ".tsp");

  // Problem specification part, follow the format of TSPLIB
  string prob_spec =
      "NAME : single_frontier\nTYPE : ATSP\nDIMENSION : " +
      to_string(config.dimension_) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;

  // Use Asymmetric TSP
  const int scale = 100;
  for (int i = 0; i < config.dimension_; ++i) {
    for (int j = 0; j < config.dimension_; ++j) {
      int int_cost = cost_matrix(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/" + config.problem_name_ + ".par").c_str());

  // Read result indices from the tour section of result file
  ifstream fin(ep_->tsp_dir_ + "/" + config.problem_name_ + ".txt");
  string res;
  // Go to tour section
  while (getline(fin, res)) {
    // Read total cost
    if (res.find("COMMENT : Length") != std::string::npos) {
      int cost_res = stoi(res.substr(19));
      total_cost = (double)cost_res / 100.0;
      // ROS_INFO("[ActiveExplorationManager] TSP problem name: %s, total
      // cost:
      // %.2f",
      //          config.problem_name_.c_str(), cost);
      std::cout << "[ActiveExplorationManager] TSP problem name: "
                << config.problem_name_ << ", total cost: " << total_cost;
    }
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  // Read indices
  while (getline(fin, res)) {
    int id = stoi(res);

    // Ignore the first state (current state)
    if (id == 1 && config.skip_first_) {
      continue;
    }

    // Ignore the last state (next grid or virtual depot)
    if (id == config.dimension_ && config.skip_last_) {
      break;
    }

    // EOF
    if (id == -1)
      break;

    result_indices.push_back(id - config.result_id_offset_);
  }
  fin.close();
}

// int FastExplorationManager::planTrajToViewInfo(
//     const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
//     const Vector3d &yaw, const Vector3d &next_pos, const double &next_yaw)
//     {
//   // Plan perception-aware trajectory (position and yaw) to the next
//   viewpoint TicToc tic_pos;
//   // [Todo] Compute time lower bound of yaw and use in trajectory
//   generation
//   // [Todo] May need to handle ed_->path_next_goal_, ed_->next_goal_
//   bool truncated = false;
//   int local_result =
//       planner_manager_->planLocalMotion(next_pos, pos, vel, acc,
//       truncated);
//   if (local_result == LOCAL_FAIL)
//     return FAIL;

//   ROS_WARN("[Local Planner] Plan path time: %fs", tic_pos.toc());

//   TicToc tic_yaw;
//   // planner_manager_->planYawExplore(yaw, next_yaw, true,
//   ep_->relax_time_); bool specify_end_yaw = (truncated) ? false : true;
//   planner_manager_->planYawInfo(yaw, next_yaw, specify_end_yaw,
//                                 ep_->relax_time_);

//   ROS_WARN("[Local Planner] Plan yaw time: %fs", tic_yaw.toc());

//   return SUCCEED;
// }

void FastExplorationManager::clearExplorationData() {
  ed_->frontier_tour_.clear();
  ed_->n_points_.clear();
  ed_->refined_ids_.clear();
  ed_->unrefined_points_.clear();
  ed_->refined_points_.clear();
  ed_->refined_views_.clear();
  ed_->refined_views1_.clear();
  ed_->refined_views2_.clear();
  ed_->refined_tour_.clear();
  has_committed_cluster_ = false;
  committed_cluster_center_.setZero();
}

double FastExplorationManager::hausdorffDistance(
    const std::vector<Eigen::Vector3d> &set1,
    const std::vector<Eigen::Vector3d> &set2, vector<int> &indices) {
  indices.clear();
  double maxDistance = 0.0;
  int min_idx, idx = 0;

  // Calculate the maximum distance from set1 to set2
  for (int i = 0; i < set1.size(); i++) {
    double minDistance = std::numeric_limits<double>::max();

    for (int j = 0; j < set2.size(); j++) {
      double distance = (set1[i] - set2[j]).norm();
      if (distance < minDistance) {
        minDistance = distance;
        min_idx = j;
      }
    }
    if (minDistance > maxDistance) {
      maxDistance = minDistance;
    }
    if (indices.size() < set2.size()) {
      indices.push_back(min_idx);
    }
  }

  // Calculate the maximum distance from set2 to set1
  for (const auto &p2 : set2) {
    double minDistance = std::numeric_limits<double>::max();
    for (const auto &p1 : set1) {
      double distance = (p2 - p1).norm();
      if (distance < minDistance) {
        minDistance = distance;
      }
    }
    if (minDistance > maxDistance) {
      maxDistance = minDistance;
    }
  }
  return maxDistance;
}

} // namespace fast_planner
