#include "planner/HybridAStarSearchMap.h"

namespace udrive {
namespace planning {

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

  std::string name = Calc3dIndex(grid_x, grid_y, grid_phi);
  if (map_3d_.find(name) == map_3d_.end()) {
    return nullptr;
  }
  return map_3d_[name];
}

std::string HybridAStarSearchMap::Calc2dIndex(const int grid_x,
                                              const int grid_y) {
  return std::to_string(grid_x) + "_" + std::to_string(grid_y);
}

std::string HybridAStarSearchMap::Calc3dIndex(const int grid_x,
                                              const int grid_y,
                                              const int grid_phi) {
  return std::to_string(grid_x) + "_" + std::to_string(grid_y) + "_" +
         std::to_string(grid_phi);
}

std::string HybridAStarSearchMap::Calc3dIndex(double x, double y, double phi) {
  while (phi > M_PI) {
    phi -= M_PI;
  }
  while (phi < -M_PI) {
    phi += M_PI;
  }
  int grid_x = static_cast<int>((x - XYbounds_[0]) / xy_grid_resolution_);
  int grid_y = static_cast<int>((y - XYbounds_[2]) / xy_grid_resolution_);
  int grid_phi = static_cast<int>((phi + M_PI) / phi_grid_resolution_);
  return Calc3dIndex(grid_x, grid_y, grid_phi);
}

// initialization
void HybridAStarSearchMap::SetXYResolution(double resolution) {
  grid_map_.SetXYResolution(resolution);
  xy_grid_resolution_ = resolution;
}

void HybridAStarSearchMap::SetPhiResolution(double resolution) {
  grid_map_.SetPhiResolution(resolution);
  phi_grid_resolution_ = resolution;
}

bool HybridAStarSearchMap::SetStartPoint(double x, double y, double phi) {
  // if (GetNodeFromGridCoord(x, y, phi) == nullptr) {
  //   return false;
  // }
  grid_map_.SetStartPoint(x, y);
  start_node_ = CreateNodeFromWorldCoord(x, y, phi);
  start_node_->SetPathCost(0);
  return true;
}

bool HybridAStarSearchMap::SetEndPoint(double x, double y, double phi) {
  // if (GetNodeFromGridCoord(x, y, phi) == nullptr) {
  //   return false;
  // }
  grid_map_.SetEndPoint(x, y);
  end_node_ = CreateNodeFromWorldCoord(x, y, phi);
  return true;
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

bool HybridAStarSearchMap::CheckStartEndPoints() {
  if (!InsideWorldMap(start_node_->GetX(), start_node_->GetY())) {
    std::cout << "start" << std::endl;

    return false;
  }
  if (!InsideWorldMap(end_node_->GetX(), end_node_->GetY())) {
    std::cout << "end" << std::endl;

    return false;
  }

  auto start =
      grid_map_.GetNodeFromWorldCoord(start_node_->GetX(), start_node_->GetY());
  if (start == nullptr || start->IsUnavailable()) {
    return false;
  }
  auto end =
      grid_map_.GetNodeFromWorldCoord(end_node_->GetX(), end_node_->GetY());
  if (end == nullptr || end->IsUnavailable()) {
    return false;
  }
  return true;
}

void HybridAStarSearchMap::Reset() {
  grid_map_.Reset();
  map_3d_.clear();
  count = 0;
}

// search
void HybridAStarSearchMap::AddObstacles(geometry_msgs::Polygon p) {
  grid_map_.AddPolygonObstacles(p);
}

void HybridAStarSearchMap::GenerateHeuristicMap() {
  grid_map_.GenerateDestinationDistanceMap();
  grid_map_.GenerateObstacleDistanceMap();

  start_node_->SetHeuristicCost(
      grid_map_.GetHeuristic(start_node_->Cal2dIndex()));
  end_node_->SetHeuristicCost(grid_map_.GetHeuristic(end_node_->Cal2dIndex()));
}

void HybridAStarSearchMap::Search() {
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
        return;
      }
      open_pq.push(next_node);
    }
  }
  std::cout << "Path not found! Expanded " << count << " nodes!" << std::endl;
  return;
}

double HybridAStarSearchMap::GetObstacleDistance(std::shared_ptr<Node3d> p) {
  return GetObstacleDistance(p->GetGridX(), p->GetGridY());
}

double HybridAStarSearchMap::GetObstacleDistance(double x, double y) {
  auto point = grid_map_.GetNodeFromWorldCoord(x, y);
  if (point == nullptr) {
    return 0;
  }
  return point->GetObstacleDistance();
}

double HybridAStarSearchMap::ObstacleDistancePenalty(double dis) {
  // need to define some better functions for obstacle penalty
  return dis <= 3 ? 2 : 0;
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
  double dx = VEHICLE_L * std::cos(phi) / 2;
  double dy = VEHICLE_L * std::sin(phi) / 2;
  if (!InsideWorldMap(x + dx, y + dy) || !InsideWorldMap(x - dx, y - dy)) {
    return false;
  }

  if (GetObstacleDistance(x + dx, y + dy) <= VEHICLE_W) {
    // std::cout << " head failure:" << GetObstacleDistance(x + dx, y + dy)
    //           << std::endl;
    return false;
  }

  if (GetObstacleDistance(x - dx, y - dy) <= VEHICLE_W) {
    // std::cout << " tail failure:" << GetObstacleDistance(x - dx, y - dy)
    //           << std::endl;
    return false;
  }
  return true;
}

// plot
void HybridAStarSearchMap::PlotHeuristicMap() {
  grid_map_.PlotHeuristicMap(xy_grid_resolution_);
  grid_map_.PlotBorders(xy_grid_resolution_);
  grid_map_.PlotObstacleMap(xy_grid_resolution_);
}

void HybridAStarSearchMap::PlotTrajectory() {
  marker_array.markers.clear();
  marker.header.frame_id = "map";
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
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;
    marker.pose.position.x = node->GetX();
    marker.pose.position.y = node->GetY();
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution_;
    marker.scale.y = xy_grid_resolution_;
    marker.scale.z = xy_grid_resolution_;
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
         std::abs(phi_diff) < phi_grid_resolution_;
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
      double path_cost =
          cur_node->GetPathCost() + MOVEMENT_PENALTY +
          ObstacleDistancePenalty(obstacle_distance) +
          std::abs(steer) * STEER_PENALTY +
          std::abs(steer - cur_node->GetSteer()) * STEER_CHANGE_PENALTY;

      if (p->GetPathCost() > path_cost) {
        p->SetSteer(steer);
        p->SetHeuristicCost(grid_map_.GetHeuristic(p->Cal2dIndex()));
        p->SetPathCost(path_cost);
        p->SetPreNode(cur_node);
        next_nodes.emplace_back(p);
      }
    }
  }
}

}  // namespace planning
}  // namespace udrive