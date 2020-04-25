#include "planner/HybridAStarSearchMap.h"

HybridAStarSearchMap::HybridAStarSearchMap(
    const std::string motion_planning_conf_path) {
  pub_trajectory =
      nh.advertise<visualization_msgs::MarkerArray>("vis_trajectory", 10);
  pub_cost_map =
      nh.advertise<visualization_msgs::MarkerArray>("vis_cost_map", 10);

  nh.getParam("next_node_num", next_node_num_);
  nh.getParam("simulation_step_size", step_size_);
  nh.getParam("max_steer", MAX_STEER);
  nh.getParam("wheel_base", WHEEL_BASE);
  nh.getParam("lidar_to_rear", LIDAR_TO_REAR);
  nh.getParam("vehicle_l", VEHICLE_L);
  nh.getParam("vehicle_w", VEHICLE_W);
  nh.getParam("rear_to_back", WHEEL_BASE_TO_BACK);

  nh.getParam("forward_penalty", FORWARD_PENALTY);
  nh.getParam("backward_penalty", BACKWARD_PENALTY);
  nh.getParam("steer_penalty", STEER_PENALTY);
  nh.getParam("steer_change_penalty", STEER_CHANGE_PENALTY);
  nh.getParam("obstacle_penalty", OBSTACLE_PENALTY);
  nh.getParam("switch_penalty", SWITCH_PENALTY);

  max_kappa_ = std::tan(MAX_STEER) / WHEEL_BASE;
  nh.getParam("rs_visualize_step_size", rs_step_size_);

  grid_map_ = std::make_shared<GridMap>();
  reed_shepp_generator_ =
      std::make_shared<ReedShepp>(max_kappa_, rs_step_size_);
}

std::shared_ptr<Node3d> HybridAStarSearchMap::CreateNodeFromWorldCoord(
    const double x, const double y, const double phi) {
  if (!InsideWorldMap(x, y)) {
    return nullptr;
  }
  return std::make_shared<Node3d>(x, y, phi, xy_grid_resolution_,
                                  phi_grid_resolution_, XYbounds_);
}

std::shared_ptr<Node3d> HybridAStarSearchMap::GetNodeFromWorldCoord(
    const double x, const double y, const double phi) {
  if (!InsideWorldMap(x, y)) {
    return nullptr;
  }

  std::string name = Calc3dIndex(x, y, phi);
  if (map_3d_.find(name) == map_3d_.end()) {
    return nullptr;
  }
  return map_3d_[name];
}

std::shared_ptr<Node3d> HybridAStarSearchMap::GetNodeFromGridCoord(
    int grid_x, int grid_y, int grid_phi) {
  if (!InsideGridMap(grid_x, grid_y)) {
    return nullptr;
  }

  std::string name = Node::ComputeStringIndex(grid_x, grid_y, grid_phi);
  if (map_3d_.find(name) == map_3d_.end()) {
    return nullptr;
  }
  return map_3d_[name];
}

std::string HybridAStarSearchMap::Calc3dIndex(double x, double y, double phi) {
  while (phi >= 2 * M_PI) {
    phi -= 2 * M_PI;
  }
  while (phi < 0) {
    phi += 2 * M_PI;
  }
  int grid_x = static_cast<int>((x - XYbounds_[0]) / xy_grid_resolution_);
  int grid_y = static_cast<int>((y - XYbounds_[2]) / xy_grid_resolution_);
  int grid_phi = static_cast<int>((phi) / phi_grid_resolution_);
  return Node::ComputeStringIndex(grid_x, grid_y, grid_phi);
}

// initialization
void HybridAStarSearchMap::SetXYResolution(double resolution) {
  grid_map_->SetXYResolution(resolution);
  xy_grid_resolution_ = resolution;
  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
}

void HybridAStarSearchMap::SetPhiResolution(double resolution) {
  grid_map_->SetPhiResolution(resolution);
  phi_grid_resolution_ = resolution;
  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
}

void HybridAStarSearchMap::SetBounds(double xmin, double xmax, double ymin,
                                     double ymax) {
  grid_map_->SetBounds(xmin, xmax, ymin, ymax);
  if (!XYbounds_.empty()) {
    XYbounds_.clear();
  }
  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;

  XYbounds_.emplace_back(xmin);
  XYbounds_.emplace_back(xmax);
  XYbounds_.emplace_back(ymin);
  XYbounds_.emplace_back(ymax);
}

bool HybridAStarSearchMap::SetStartPoint(double x, double y, double phi) {
  if (!InsideWorldMap(x, y)) {
    return false;
  }
  grid_map_->SetStartPoint(x, y);
  start_node_ = CreateNodeFromWorldCoord(x, y, phi);
  start_node_->SetPathCost(0);
  return true;
}

bool HybridAStarSearchMap::SetEndPoint(double x, double y, double phi) {
  if (!InsideWorldMap(x, y)) {
    return false;
  }
  grid_map_->SetEndPoint(x, y);
  end_node_ = CreateNodeFromWorldCoord(x, y, phi);
  return true;
}

bool HybridAStarSearchMap::CheckStartEndPoints() {
  if (!InsideWorldMap(start_node_->GetX(), start_node_->GetY())) {
    std::cout << "start point out of range!" << std::endl;
    return false;
  }

  auto start = grid_map_->GetNodeFromWorldCoord(start_node_->GetX(),
                                                start_node_->GetY());
  if (start == nullptr || start->IsUnavailable()) {
    std::cout << "start point unavailable!" << std::endl;
    return false;
  }

  if (!InsideWorldMap(end_node_->GetX(), end_node_->GetY())) {
    std::cout << "end point out of range!" << std::endl;
    return false;
  }

  auto end =
      grid_map_->GetNodeFromWorldCoord(end_node_->GetX(), end_node_->GetY());
  if (end == nullptr || end->IsUnavailable()) {
    std::cout << "end point unavailable!" << std::endl;
    return false;
  }
  return true;
}

void HybridAStarSearchMap::Reset() {
  grid_map_->Reset();
  map_3d_.clear();
  final_node_ = nullptr;
  final_reeds_shepp_ = nullptr;
  count = 0;
}

// search
void HybridAStarSearchMap::AddObstacles(geometry_msgs::Polygon p) {
  grid_map_->AddPolygonObstacles(p, LIDAR_TO_REAR);
}

void HybridAStarSearchMap::AddObstacleArrayPtr(
    costmap_converter::ObstacleArrayMsgConstPtr ptr) {
  for (costmap_converter::ObstacleMsg ob : ptr->obstacles) {
    AddObstacles(ob.polygon);
  }
}

bool HybridAStarSearchMap::GenerateHeuristicMap() {
  if (!CheckStartEndPoints()) {
    ROS_ERROR("start/end points invalid!");
    return false;
  }

  grid_map_->GenerateDestinationDistanceMap();
  grid_map_->GenerateObstacleDistanceMap();

  if (!CollisionDection(start_node_->GetX(), start_node_->GetY(),
                        start_node_->GetPhi()) ||
      !CollisionDection(end_node_->GetX(), end_node_->GetY(),
                        end_node_->GetPhi())) {
    ROS_ERROR("start/end points collision detected!");
    return false;
  }

  start_node_->SetHeuristicCost(
      grid_map_->GetHeuristic(start_node_->Cal2dIndex()));
  end_node_->SetHeuristicCost(grid_map_->GetHeuristic(end_node_->Cal2dIndex()));
  return true;
}

bool HybridAStarSearchMap::Search() {
  struct cmp {
    bool operator()(const std::shared_ptr<Node3d> left,
                    const std::shared_ptr<Node3d> right) const {
      return left->GetCost() >= right->GetCost();
    }
  };
  std::priority_queue<std::shared_ptr<Node3d>,
                      std::vector<std::shared_ptr<Node3d>>, cmp>
      open_pq;
  open_pq.push(start_node_);
  std::set<std::string> visited;

  while (open_pq.size() > 0) {
    std::shared_ptr<Node3d> node = open_pq.top();
    open_pq.pop();

    if (visited.find(node->GetIndex()) != visited.end()) {
      continue;
    }
    visited.emplace(node->GetIndex());
    count++;
    map_3d_[node->GetIndex()] = node;
    max_cost = std::max(max_cost, node->GetCost());

    std::vector<std::shared_ptr<Node3d>> next_nodes;
    NextNodeGenerator(next_nodes, node, -step_size_);
    NextNodeGenerator(next_nodes, node, step_size_);
    double distance = DistanceToTerminateState(node);

    if (node->analysis_step == 0) {
      for (std::shared_ptr<Node3d> next_node : next_nodes) {
        if (AnalyticExpansion(next_node)) {
          final_node_ = next_node;
          std::cout << "Trajectory found! Expanded " << count << " grids!"
                    << std::endl;
          return true;
        }
        next_node->analysis_step = std::min((int)distance / 3, 5);
        open_pq.push(next_node);
      }
    } else {
      for (std::shared_ptr<Node3d> next_node : next_nodes) {
        next_node->analysis_step = node->analysis_step - 1;
        open_pq.push(next_node);
      }
    }
  }

  std::cout << "Path not found! Expanded " << count << " grids!" << std::endl;
  return false;
}

double HybridAStarSearchMap::GetObstacleDistance(std::shared_ptr<Node3d> p) {
  return GetObstacleDistance(p->GetGridX(), p->GetGridY());
}

double HybridAStarSearchMap::GetObstacleDistance(double x, double y) {
  auto point = grid_map_->GetNodeFromWorldCoord(x, y);
  if (point == nullptr) {
    return 0;
  }
  return (double)point->GetObstacleDistance() * xy_grid_resolution_;
}

double HybridAStarSearchMap::ObstacleDistancePenalty(double dis) {
  // need to define some better functions for obstacle penalty
  return dis < 2 * VEHICLE_W ? OBSTACLE_PENALTY : 0;
}

// state check
bool HybridAStarSearchMap::InsideGridMap(const int node_grid_x,
                                         const int node_grid_y) {
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  return true;
}

bool HybridAStarSearchMap::InsideWorldMap(const double x, const double y) {
  if (x <= XYbounds_[0] || x >= XYbounds_[1] || y <= XYbounds_[2] ||
      y >= XYbounds_[3]) {
    return false;
  }
  return true;
}

bool HybridAStarSearchMap::CollisionDection(double x, double y, double phi) {
  double midx = x + (VEHICLE_L / 2 - WHEEL_BASE_TO_BACK) * std::cos(phi);
  double midy = y + (VEHICLE_L / 2 - WHEEL_BASE_TO_BACK) * std::sin(phi);

  if (!InsideWorldMap(midx, midy)) {
    return false;
  }

  double diag = GetObstacleDistance(midx, midy);
  if (4 * diag * diag > VEHICLE_L * VEHICLE_L + VEHICLE_W * VEHICLE_W) {
    return true;
  }

  std::vector<double> local_dx{-VEHICLE_L / 2, -VEHICLE_L / 2, VEHICLE_L / 2,
                               VEHICLE_L / 2};
  std::vector<double> local_dy{-VEHICLE_W / 2, VEHICLE_W / 2, VEHICLE_W / 2,
                               -VEHICLE_W / 2};

  std::vector<double> global_dx, global_dy;

  for (int i = 0; i < 4; i++) {
    global_dx.emplace_back(local_dx[i] * std::sin(phi) +
                           local_dy[i] * std::cos(phi));
    global_dy.emplace_back(local_dx[i] * std::cos(phi) -
                           local_dy[i] * std::sin(phi));
  }

  for (int i = 0; i < 4; i++) {
    double start_x = midx + global_dx[i];
    double start_y = midy + global_dy[i];
    double end_x = midx + global_dx[(i + 1) % 4];
    double end_y = midy + global_dy[(i + 1) % 4];

    if (!InsideWorldMap(start_x, start_y)) {
      return false;
    }

    // DDA
    int length = static_cast<int>(std::max(std::abs(start_x - end_x),
                                           std::abs(start_y - end_y)) /
                                  xy_grid_resolution_) +
                 1;
    double delta_x = (end_x - start_x) / length;
    double delta_y = (end_y - start_y) / length;

    double x = start_x, y = start_y;
    for (int i = 0; i <= length; ++i) {
      if (!InsideWorldMap(x, y)) {
        return false;
      }

      std::shared_ptr<Node3d> grid_p = GetNodeFromWorldCoord(x, y, 0);
      if (grid_p == nullptr || GetObstacleDistance(grid_p) <= 0.3) {
        return false;
      }

      x += delta_x;
      y += delta_y;
    }
  }

  // if (GetObstacleDistance(midx + dx, midy + dy) <= 0.5 * VEHICLE_W) {
  //   return false;
  // }

  // if (GetObstacleDistance(midx, midy) <= 0.8 * VEHICLE_W) {
  //   return false;
  // }

  // if (GetObstacleDistance(midx - dx, midy - dy) <= 0.5 * VEHICLE_W) {
  //   return false;
  // }

  return true;
}

// plot
void HybridAStarSearchMap::PlotDebugMap() {
  grid_map_->PlotBorders(xy_grid_resolution_);
  grid_map_->PlotObstacleMap(xy_grid_resolution_);
  PlotCostMap();
}

void HybridAStarSearchMap::PlotHeuristicMap() {
  grid_map_->PlotHeuristicMap(xy_grid_resolution_);
  grid_map_->PlotBorders(xy_grid_resolution_);
  grid_map_->PlotObstacleMap(xy_grid_resolution_);
}

void HybridAStarSearchMap::PlotCostMap() {
  marker_array.markers.clear();
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(3.33);
  int marker_id = 0;
  double temp_max = 0;

  std::unordered_map<std::string, std::shared_ptr<Node3d>>::iterator iter3d =
      map_3d_.begin();
  while (iter3d != map_3d_.end()) {
    auto node3d = iter3d->second;
    auto node2d =
        grid_map_->GetNodeFromGridCoord(node3d->GetGridX(), node3d->GetGridY());
    node2d->expanded++;
    node2d->debug_cost = std::min(node2d->debug_cost, node3d->GetCost());
    temp_max = std::max(temp_max, node2d->debug_cost);
    ++iter3d;
  }

  std::unordered_map<std::string, std::shared_ptr<Node2d>>::iterator iter =
      grid_map_->map_2d_.begin();
  while (iter != grid_map_->map_2d_.end()) {
    auto node2d = iter->second;
    marker.id = marker_id;
    marker.color.r = node2d->debug_cost / temp_max;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.03 * node2d->expanded;

    marker.pose.position.x =
        XYbounds_[0] + node2d->GetGridX() * xy_grid_resolution_;
    marker.pose.position.y =
        XYbounds_[2] + node2d->GetGridY() * xy_grid_resolution_;
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution_;
    marker.scale.y = xy_grid_resolution_;
    marker.scale.z = xy_grid_resolution_;
    marker_array.markers.push_back(marker);
    ++marker_id;
    ++iter;
  }
  pub_cost_map.publish(marker_array);
  return;
}  // namespace planning

void HybridAStarSearchMap::PlotTrajectory() {
  marker_array.markers.clear();
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";
  int marker_id = 0;

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = xy_grid_resolution_;
  marker.scale.y = xy_grid_resolution_;
  marker.scale.z = xy_grid_resolution_;

  marker.lifetime = ros::Duration(2);
  marker.pose.position.z = 0;

  if (final_reeds_shepp_ != nullptr) {
    for (int i = 0; i < final_reeds_shepp_->x.size(); i++) {
      marker.id = marker_id;
      if (final_reeds_shepp_->gear[i]) {
        marker.color.r = 0.9f;
        marker.color.b = 0.1f;
        marker.color.g = 0.3f;
      } else {
        marker.color.r = 0.1f;
        marker.color.b = 0.9f;
        marker.color.g = 0.3f;
      }
      marker.color.a = 0.5;
      marker.pose.position.x = final_reeds_shepp_->x[i];
      marker.pose.position.y = final_reeds_shepp_->y[i];

      marker_array.markers.push_back(marker);
      ++marker_id;
    }
  }

  std::shared_ptr<Node3d> node = final_node_;
  while (node != nullptr) {
    marker.id = marker_id;
    if (node->GetDirection()) {
      marker.color.r = 1.0f;
      marker.color.b = 0.0f;
      marker.color.g = 0.0f;
    } else {
      marker.color.r = 0.0f;
      marker.color.b = 1.0f;
      marker.color.g = 0.0f;
    }
    marker.color.a = 0.2f;
    marker.pose.position.x = node->GetX();
    marker.pose.position.y = node->GetY();

    marker_array.markers.push_back(marker);
    ++marker_id;
    node = node->GetPreNode();
  }

  pub_trajectory.publish(marker_array);
}

// node expansion
double HybridAStarSearchMap::DistanceToTerminateState(
    std::shared_ptr<Node3d> node) {
  double x_diff = end_node_->GetX() - node->GetX();
  double y_diff = end_node_->GetY() - node->GetY();
  double phi_diff = end_node_->GetPhi() - node->GetPhi();
  return std::sqrt(x_diff * x_diff + y_diff * y_diff +
                   std::abs(std::fmod(phi_diff, 2 * M_PI)));
}

void HybridAStarSearchMap::Update(double& x, double& y, double& phi,
                                  double steer, double dis) {
  x += dis * std::cos(phi);
  y += dis * std::sin(phi);
  phi += dis / WHEEL_BASE * std::tan(steer);
}

void HybridAStarSearchMap::NextNodeGenerator(
    std::vector<std::shared_ptr<Node3d>>& next_nodes,
    std::shared_ptr<Node3d> cur_node, double step_size) {
  for (int index = 0; index <= next_node_num_; index++) {
    double steer = -MAX_STEER + 2 * MAX_STEER / next_node_num_ * index;
    double last_x = cur_node->GetX();
    double last_y = cur_node->GetY();
    double last_phi = cur_node->GetPhi();
    bool forward = step_size > 0;
    bool flag = true;

    for (int i = 0; i <= xy_grid_resolution_ * 1.5 / std::abs(step_size); i++) {
      Update(last_x, last_y, last_phi, steer, step_size);
      if (!InsideWorldMap(last_x, last_y) ||
          !CollisionDection(last_x, last_y, last_phi)) {
        flag = false;
        break;
      }

      auto point = grid_map_->GetNodeFromWorldCoord(last_x, last_y);
      if (point == nullptr || point->IsUnavailable()) {
        flag == false;
        break;
      }
    }

    if (flag) {
      std::shared_ptr<Node3d> p =
          GetNodeFromWorldCoord(last_x, last_y, last_phi);
      if (p == nullptr) {
        p = CreateNodeFromWorldCoord(last_x, last_y, last_phi);
      }

      int obstacle_distance = GetObstacleDistance(p);
      double movement = forward ? FORWARD_PENALTY : BACKWARD_PENALTY;
      double switch_penalty = p->GetDirection() != forward ? SWITCH_PENALTY : 0;

      double path_cost =
          cur_node->GetPathCost() + ObstacleDistancePenalty(obstacle_distance) +
          movement + switch_penalty + std::abs(steer) * STEER_PENALTY +
          std::abs(steer - cur_node->GetSteer()) * STEER_CHANGE_PENALTY;

      if (p->GetPathCost() > path_cost) {
        p->SetDirection(forward);
        p->SetSteer(steer);
        p->SetHeuristicCost(grid_map_->GetHeuristic(p->Cal2dIndex()));
        p->SetPathCost(path_cost);
        p->SetPreNode(cur_node);
        next_nodes.emplace_back(p);
      }
    }
  }
}

bool HybridAStarSearchMap::AnalyticExpansion(
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          *reeds_shepp_to_check)) {
    return false;
  }
  // std::cout << "ShortestRSP found";

  int size = reeds_shepp_to_check->x.size();

  for (int i = size - 1; i > 0; i--) {
    if (!CollisionDection(reeds_shepp_to_check->x[i],
                          reeds_shepp_to_check->y[i],
                          reeds_shepp_to_check->phi[i])) {
      return false;
    }
  }
  final_reeds_shepp_ = reeds_shepp_to_check;

  std::cout << "Reach the end configuration with Reed Sharp";
  return true;
}