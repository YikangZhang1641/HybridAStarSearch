#include "planner/GridMap.h"

// node generation
std::shared_ptr<Node2d> GridMap::CreateNodeFromWorldCoord(const double x,
                                                          const double y) {
  auto node = std::make_shared<Node2d>(x, y, xy_grid_resolution_, XYbounds_);
  map_2d_[node->GetIndex()] = node;
  return node;
}

std::shared_ptr<Node2d> GridMap::CreateNodeFromGridCoord(const int x_grid,
                                                         const int y_grid) {
  auto node = std::make_shared<Node2d>(x_grid, y_grid, XYbounds_);
  map_2d_[node->GetIndex()] = node;
  return node;
}

std::shared_ptr<Node2d> GridMap::GetNodeFromWorldCoord(const double x,
                                                       const double y) {
  int x_grid = static_cast<int>((x - XYbounds_[0]) / xy_grid_resolution_);
  int y_grid = static_cast<int>((y - XYbounds_[2]) / xy_grid_resolution_);
  return GetNodeFromGridCoord(x_grid, y_grid);
}

std::shared_ptr<Node2d> GridMap::GetNodeFromGridCoord(const int x_grid,
                                                      const int y_grid) {
  std::string name = std::to_string(x_grid) + "_" + std::to_string(y_grid);
  if (map_2d_.find(name) == map_2d_.end()) {
    return nullptr;
  }
  return map_2d_[name];
}

// map configuration
bool GridMap::SetXYResolution(double resolution) {
  xy_grid_resolution_ = resolution;
}

bool GridMap::SetPhiResolution(double resolution) {
  phi_grid_resolution_ = resolution;
}

bool GridMap::SetStartPoint(double x, double y) {
  start_node_ = CreateNodeFromWorldCoord(x, y);
  if (start_node_ == nullptr) {
    std::cout << "start node setting failure!" << std::endl;
    return false;
  }
  return true;
}

bool GridMap::SetEndPoint(double x, double y) {
  end_node_ = CreateNodeFromWorldCoord(x, y);
  end_node_->SetDestinationCost(0);
  return true;
}

bool GridMap::SetBounds(double xmin, double xmax, double ymin, double ymax) {
  if (!XYbounds_.empty()) {
    XYbounds_.clear();
  }
  XYbounds_.emplace_back(xmin);
  XYbounds_.emplace_back(xmax);
  XYbounds_.emplace_back(ymin);
  XYbounds_.emplace_back(ymax);

  max_grid_x_ =
      static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ =
      static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
  return XYbounds_.size() == 4;
}

// add obstacles into the map
void GridMap::AddPolygonObstacles(geometry_msgs::Polygon p,
                                  double LIDAR_TO_REAR) {
  if (p.points.empty()) {
    ROS_INFO("Polygon Obstacle empty!");
    return;
  }

  // the segments are the same size as the points.
  std::vector<double> vec_start_x, vec_end_x, vec_start_y, vec_end_y;
  int size = p.points.size();
  for (int i = 0; i + 1 < size; i++) {
    vec_start_x.emplace_back((double)p.points[i].x);
    vec_start_y.emplace_back((double)p.points[i].y);
    vec_end_x.emplace_back((double)p.points[i + 1].x);
    vec_end_y.emplace_back((double)p.points[i + 1].y);
  }
  vec_start_x.emplace_back((double)p.points[size - 1].x);
  vec_start_y.emplace_back((double)p.points[size - 1].y);
  vec_end_x.emplace_back((double)p.points[0].x);
  vec_end_y.emplace_back((double)p.points[0].y);

  for (int i = 0; i < size; i++) {
    double start_x = vec_start_x[i] + LIDAR_TO_REAR;
    double start_y = vec_start_y[i];
    double end_x = vec_end_x[i] + LIDAR_TO_REAR;
    double end_y = vec_end_y[i];

    // DDA
    int length =
        static_cast<int>(
            std::max(std::abs(start_x - end_x), std::abs(start_y - end_y)) /
            xy_grid_resolution_) +
        1;  // +1 is important!! MUST make sure the segments are enclosed.
    double delta_x = (end_x - start_x) / length;
    double delta_y = (end_y - start_y) / length;

    double x = start_x, y = start_y;
    for (int i = 0; i <= length; ++i) {
      std::shared_ptr<Node2d> grid_p = GetNodeFromWorldCoord(x, y);
      if (grid_p == nullptr) {
        grid_p = CreateNodeFromWorldCoord(x, y);
      }
      grid_p->SetUnavailable();
      grid_p->SetObstacleDistance(0);
      border_unavailable_.emplace(grid_p->GetIndex());

      x += delta_x;
      y += delta_y;
    }
  }
}

// heuristic map & obstacle map
bool GridMap::GenerateDestinationDistanceMap() {
  struct cmp {
    bool operator()(const std::shared_ptr<Node2d> left,
                    const std::shared_ptr<Node2d> right) const {
      return left->GetCost() >= right->GetCost();
    }
  };
  std::priority_queue<std::shared_ptr<Node2d>,
                      std::vector<std::shared_ptr<Node2d>>, cmp>
      pq_;
  pq_.push(end_node_);
  std::set<std::string> visited;

  while (pq_.size() > 0) {
    std::shared_ptr<Node2d> cur_node = pq_.top();
    pq_.pop();
    std::string cur_name = cur_node->GetIndex();
    if (visited.find(cur_name) != visited.end()) {
      continue;
    }
    visited.emplace(cur_name);

    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(cur_node));
    for (auto& next_node : next_nodes) {
      if (!InsideGridMap(next_node->GetGridX(), next_node->GetGridY())) {
        continue;
      }

      if (next_node->IsUnavailable()) {
        border_available_.emplace(cur_name);
        continue;
      }

      max_cost = std::max(max_cost, next_node->GetDestinationCost());
      pq_.push(next_node);
    }
  }
  std::cout << "Heuristic Map generated successfully! visited size: "
            << visited.size() << " map size: " << map_2d_.size() << std::endl;

  return true;
}

bool GridMap::GenerateObstacleDistanceMap() {
  struct cmp {
    bool operator()(const std::shared_ptr<Node2d> left,
                    const std::shared_ptr<Node2d> right) const {
      return left->GetObstacleDistance() >= right->GetObstacleDistance();
    }
  };

  int DIRS[][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
  std::priority_queue<std::shared_ptr<Node2d>,
                      std::vector<std::shared_ptr<Node2d>>, cmp>
      pq;
  std::set<std::string> visited;
  for (std::string node_name : border_available_) {
    auto node = map_2d_[node_name];
    node->SetObstacleDistance(1);
    pq.push(node);
  }

  // for available region
  while (!pq.empty()) {
    auto cur_node = pq.top();
    pq.pop();

    if (visited.find(cur_node->GetIndex()) != visited.end()) {
      continue;
    }
    visited.emplace(cur_node->GetIndex());

    for (int i = 0; i < 4; i++) {
      int nx = cur_node->GetGridX() + DIRS[i][0];
      int ny = cur_node->GetGridY() + DIRS[i][1];

      if (!InsideGridMap(nx, ny)) {
        continue;
      }

      auto next_node = GetNodeFromGridCoord(nx, ny);
      if (next_node == nullptr) {
        next_node = CreateNodeFromWorldCoord(nx, ny);
      }

      if (next_node->IsUnavailable()) {
        continue;
      }

      if (next_node->GetObstacleDistance() >
          cur_node->GetObstacleDistance() + 1) {
        next_node->SetObstacleDistance(cur_node->GetObstacleDistance() + 1);
      }
      pq.emplace(next_node);
    }
  }

  // for unavailable region
  std::deque<std::shared_ptr<Node2d>> dq;
  for (auto name : border_unavailable_) {
    dq.emplace_front(map_2d_[name]);
  }
  while (!dq.empty()) {
    auto cur_node = dq.back();
    dq.pop_back();

    if (visited.find(cur_node->GetIndex()) != visited.end()) {
      continue;
    }
    visited.emplace(cur_node->GetIndex());
    cur_node->SetUnavailable();
    cur_node->SetDestinationCost(std::numeric_limits<double>::max());

    for (int i = 0; i < 4; i++) {
      int nx = cur_node->GetGridX() + DIRS[i][0];
      int ny = cur_node->GetGridY() + DIRS[i][1];
      if (!InsideGridMap(nx, ny)) {
        continue;
      }

      auto next_node = GetNodeFromGridCoord(nx, ny);
      if (next_node == nullptr) {
        next_node = CreateNodeFromGridCoord(nx, ny);
      }
      dq.emplace_front(next_node);
    }
  }

  std::cout << "Obstacle Map generated successfully! visited size: "
            << visited.size() << std::endl;
}

void GridMap::Reset() {
  map_2d_.clear();
  border_available_.clear();
  border_unavailable_.clear();
}

// get the 2d heuristic value
double GridMap::GetHeuristic(std::string s) {
  if (map_2d_.find(s) == map_2d_.end()) {
    return std::numeric_limits<double>::max();
  }
  return map_2d_[s]->GetCost();
}

// plot
void GridMap::PlotHeuristicMap(double xy_grid_resolution) {
  marker_array.markers.clear();
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  int marker_id = 0;

  std::unordered_map<std::string, std::shared_ptr<Node2d>>::iterator iter =
      map_2d_.begin();
  while (iter != map_2d_.end()) {
    auto node = iter->second;
    marker.id = marker_id;
    marker.color.r = 1.0f - node->GetDestinationCost() / 50;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;

    marker.pose.position.x =
        XYbounds_[0] + node->GetGridX() * xy_grid_resolution;
    marker.pose.position.y =
        XYbounds_[2] + node->GetGridY() * xy_grid_resolution;
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution;
    marker.scale.y = xy_grid_resolution;
    marker.scale.z = xy_grid_resolution;
    marker_array.markers.push_back(marker);
    ++marker_id;
    ++iter;
  }
  pub_map.publish(marker_array);
  return;
}

void GridMap::PlotBorders(double xy_grid_resolution) {
  marker_array.markers.clear();
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  int marker_id = 0;

  std::set<std::string, std::shared_ptr<Node2d>>::iterator iter =
      border_unavailable_.begin();
  while (iter != border_unavailable_.end()) {
    std::shared_ptr<Node2d> node = map_2d_[*iter];
    marker.id = marker_id;
    marker.color.r = 0.0;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.2;
    marker.pose.position.x =
        XYbounds_[0] + node->GetGridX() * xy_grid_resolution;
    marker.pose.position.y =
        XYbounds_[2] + node->GetGridY() * xy_grid_resolution;
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution;
    marker.scale.y = xy_grid_resolution;
    marker.scale.z = xy_grid_resolution;
    marker_array.markers.push_back(marker);
    ++marker_id;
    ++iter;
  }
  pub_border.publish(marker_array);
  return;
}

void GridMap::PlotObstacleMap(double xy_grid_resolution) {
  marker_array.markers.clear();
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  int marker_id = 0;

  std::unordered_map<std::string, std::shared_ptr<Node2d>>::iterator iter =
      map_2d_.begin();
  while (iter != map_2d_.end()) {
    auto node = iter->second;
    marker.id = marker_id;
    marker.color.r = 0.0f;
    marker.color.g =
        1.0f - (double)node->GetObstacleDistance() * xy_grid_resolution_ / 10.0;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;

    marker.pose.position.x =
        XYbounds_[0] + node->GetGridX() * xy_grid_resolution;
    marker.pose.position.y =
        XYbounds_[2] + node->GetGridY() * xy_grid_resolution;
    marker.pose.position.z = 0;
    marker.scale.x = xy_grid_resolution;
    marker.scale.y = xy_grid_resolution;
    marker.scale.z = xy_grid_resolution;
    marker_array.markers.push_back(marker);
    ++marker_id;
    ++iter;
  }
  pub_obstacle.publish(marker_array);
  return;
}

// for expansion
std::vector<std::shared_ptr<Node2d>> GridMap::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  int current_node_x = current_node->GetGridX();
  int current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetDestinationCost();

  double next_node_path_cost = current_node_path_cost + 1;
  if (current_node_path_cost == std::numeric_limits<double>::max()) {
    std::cout << "found an infinity" << std::endl;
  }

  int DIRS[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
  for (int i = 0; i < 4; i++) {
    int next_x = current_node_x + DIRS[i][0];
    int next_y = current_node_y + DIRS[i][1];
    if (!InsideGridMap(next_x, next_y)) {
      continue;
    }
    std::shared_ptr<Node2d> next = GetNodeFromGridCoord(next_x, next_y);
    if (next == nullptr) {
      next = CreateNodeFromGridCoord(next_x, next_y);
    }
    if (next->GetDestinationCost() > next_node_path_cost) {
      next->SetDestinationCost(next_node_path_cost);
    }
    next_nodes.emplace_back(next);
  }

  return next_nodes;
}

bool GridMap::InsideGridMap(const int node_grid_x, const int node_grid_y) {
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  return true;
}

bool GridMap::InsideWorldMap(const double x, const double y) {
  if (x < XYbounds_[0] || x > XYbounds_[1] || y < XYbounds_[2] ||
      y > XYbounds_[3]) {
    return false;
  }
  return true;
}