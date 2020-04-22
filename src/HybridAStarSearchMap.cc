#include "planner/HybridAStarSearchMap.h"

HybridAStarSearchMap::HybridAStarSearchMap(
    const std::string motion_planning_conf_path) {
  pub = nh.advertise<visualization_msgs::MarkerArray>("vis_trajectory", 10);
  pub_cost_map =
      nh.advertise<visualization_msgs::MarkerArray>("vis_cost_map", 10);
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
  grid_map_.SetXYResolution(resolution);
  xy_grid_resolution_ = resolution;
  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
}

void HybridAStarSearchMap::SetPhiResolution(double resolution) {
  grid_map_.SetPhiResolution(resolution);
  phi_grid_resolution_ = resolution;
  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
}

void HybridAStarSearchMap::SetBounds(double xmin, double xmax, double ymin,
                                     double ymax) {
  grid_map_.SetBounds(xmin, xmax, ymin, ymax);
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
  grid_map_.SetStartPoint(x, y);
  start_node_ = CreateNodeFromWorldCoord(x, y, phi);
  start_node_->SetPathCost(0);
  return true;
}

bool HybridAStarSearchMap::SetEndPoint(double x, double y, double phi) {
  if (!InsideWorldMap(x, y)) {
    return false;
  }
  grid_map_.SetEndPoint(x, y);
  end_node_ = CreateNodeFromWorldCoord(x, y, phi);
  return true;
}

bool HybridAStarSearchMap::CheckStartEndPoints() {
  if (!InsideWorldMap(start_node_->GetX(), start_node_->GetY())) {
    std::cout << "start point out of range!" << std::endl;
    return false;
  }

  auto start =
      grid_map_.GetNodeFromWorldCoord(start_node_->GetX(), start_node_->GetY());
  if (start == nullptr || start->IsUnavailable()) {
    std::cout << "start point unavailable!" << std::endl;
    return false;
  }

  if (!InsideWorldMap(end_node_->GetX(), end_node_->GetY())) {
    std::cout << "end point out of range!" << std::endl;
    return false;
  }

  auto end =
      grid_map_.GetNodeFromWorldCoord(end_node_->GetX(), end_node_->GetY());
  if (end == nullptr || end->IsUnavailable()) {
    std::cout << "end point unavailable!" << std::endl;
    return false;
  }
  return true;
}

void HybridAStarSearchMap::Reset() {
  grid_map_.Reset();
  map_3d_.clear();
  final_node_ = nullptr;
  count = 0;
}

// search
void HybridAStarSearchMap::AddObstacles(geometry_msgs::Polygon p) {
  grid_map_.AddPolygonObstacles(p, LIDAR_TO_REAR);
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

  grid_map_.GenerateDestinationDistanceMap();
  grid_map_.GenerateObstacleDistanceMap();

  if (!CollisionDection(start_node_->GetX(), start_node_->GetY(),
                        start_node_->GetPhi()) ||
      !CollisionDection(end_node_->GetX(), end_node_->GetY(),
                        end_node_->GetPhi())) {
    ROS_ERROR("start/end points invalid!");
    return false;
  }

  start_node_->SetHeuristicCost(
      grid_map_.GetHeuristic(start_node_->Cal2dIndex()));
  end_node_->SetHeuristicCost(grid_map_.GetHeuristic(end_node_->Cal2dIndex()));
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
    // std::cout << "size: " << next_nodes.size() << std::endl;

    for (std::shared_ptr<Node3d> next_node : next_nodes) {
      if (IsTerminateState(next_node)) {
        final_node_ = next_node;
        std::cout << "Trajectory found! Expanded " << count << " nodes!"
                  << std::endl;
        return true;
      }
      open_pq.push(next_node);
    }
  }
  std::cout << "Path not found! Expanded " << count << " nodes!" << std::endl;
  return false;
}

std::vector<common::PathPoint> HybridAStarSearchMap::GetResult(
    const udrive::common::VehicleState& vehicle) {
  double odom_x = vehicle.x();
  double odom_y = vehicle.y();
  double odom_theta = vehicle.heading();

  std::vector<common::PathPoint> rst;
  std::shared_ptr<Node3d> node = final_node_;
  while (node != nullptr) {
    common::PathPoint point;
    point.set_x(odom_x + node->GetX() * std::cos(odom_theta) -
                node->GetY() * std::sin(odom_theta));
    point.set_y(odom_y + node->GetX() * std::sin(odom_theta) +
                node->GetY() * std::cos(odom_theta));
    point.set_theta(odom_theta + node->GetPhi());
    point.set_direction(node->GetDirection() ? 1 : -1);
    rst.emplace_back(point);
    node = node->GetPreNode();
  }
  reverse(rst.begin(), rst.end());
  return rst;
}

double HybridAStarSearchMap::GetObstacleDistance(std::shared_ptr<Node3d> p) {
  return GetObstacleDistance(p->GetGridX(), p->GetGridY());
}

double HybridAStarSearchMap::GetObstacleDistance(double x, double y) {
  auto point = grid_map_.GetNodeFromWorldCoord(x, y);
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
  double midx = x + WHEEL_BASE_TO_BACK * std::cos(phi);
  double midy = y + WHEEL_BASE_TO_BACK * std::sin(phi);

  double dx = VEHICLE_L * std::cos(phi) / 2;
  double dy = VEHICLE_L * std::sin(phi) / 2;

  if (!InsideWorldMap(midx + dx, midy + dy) ||
      !InsideWorldMap(midx - dx, midy - dy)) {
    return false;
  }

  if (GetObstacleDistance(midx + dx, midy + dy) <= 0.5 * VEHICLE_W) {
    // std::cout << " head failure:" << GetObstacleDistance(x + dx, y + dy)
    //           << std::endl;
    return false;
  }

  if (GetObstacleDistance(midx, midy) <= 0.8 * VEHICLE_W) {
    // std::cout << " tail failure:" << GetObstacleDistance(x - dx, y - dy)
    //           << std::endl;
    return false;
  }

  if (GetObstacleDistance(midx - dx, midy - dy) <= 0.5 * VEHICLE_W) {
    // std::cout << " tail failure:" << GetObstacleDistance(x - dx, y - dy)
    //           << std::endl;
    return false;
  }
  return true;
}

// plot
void HybridAStarSearchMap::PlotDebugMap() {
  grid_map_.PlotBorders(xy_grid_resolution_);
  grid_map_.PlotObstacleMap(xy_grid_resolution_);
  PlotCostMap();
}

void HybridAStarSearchMap::PlotHeuristicMap() {
  grid_map_.PlotHeuristicMap(xy_grid_resolution_);
  grid_map_.PlotBorders(xy_grid_resolution_);
  grid_map_.PlotObstacleMap(xy_grid_resolution_);
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
        grid_map_.GetNodeFromGridCoord(node3d->GetGridX(), node3d->GetGridY());
    node2d->expanded++;
    node2d->debug_cost = std::min(node2d->debug_cost, node3d->GetCost());
    temp_max = std::max(temp_max, node2d->debug_cost);
    ++iter3d;
  }

  std::unordered_map<std::string, std::shared_ptr<Node2d>>::iterator iter =
      grid_map_.map_2d_.begin();
  while (iter != grid_map_.map_2d_.end()) {
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

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  int marker_id = 0;

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
    marker.color.a = 0.5f;
    marker.pose.position.x = node->GetX();
    marker.pose.position.y = node->GetY();
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution_;
    marker.scale.y = xy_grid_resolution_;
    marker.scale.z = xy_grid_resolution_;
    marker.lifetime = ros::Duration(3.333);
    marker_array.markers.push_back(marker);
    ++marker_id;
    node = node->GetPreNode();
  }

  pub.publish(marker_array);
}

// node expansion
bool HybridAStarSearchMap::IsTerminateState(std::shared_ptr<Node3d> node) {
  double x_diff = end_node_->GetX() - node->GetX();
  double y_diff = end_node_->GetY() - node->GetY();
  double phi_diff = end_node_->GetPhi() - node->GetPhi();
  return std::sqrt(x_diff * x_diff + y_diff * y_diff) < xy_grid_resolution_ &&
         (std::abs(phi_diff) < phi_grid_resolution_ ||
          std::abs(std::abs(phi_diff) - 2 * M_PI) < phi_grid_resolution_);
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

      auto point = grid_map_.GetNodeFromWorldCoord(last_x, last_y);
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
        p->SetHeuristicCost(grid_map_.GetHeuristic(p->Cal2dIndex()));
        p->SetPathCost(path_cost);
        p->SetPreNode(cur_node);
        next_nodes.emplace_back(p);
      }
    }
  }
}