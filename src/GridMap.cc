#include "planner/GridMap.h"

std::shared_ptr<Node2d> GridMap::createNodeFromWorldCoord(double x, double y) {
  return std::make_shared<Node2d>(x, y, xy_grid_resolution_, XYbounds_);
}

std::shared_ptr<Node2d> GridMap::createNodeFromGridCoord(int x, int y) {
  return std::make_shared<Node2d>(x, y, XYbounds_);
}

bool GridMap::setStartPoint(double x, double y) {
  start_node_ = createNodeFromWorldCoord(x, y);
  if (start_node_ == nullptr) {
    std::cout << "start node setting failure!" << std::endl;
    return false;
  }
  return true;
}

bool GridMap::setEndPoint(double x, double y) {
  end_node_ = createNodeFromWorldCoord(x, y);
  end_node_->SetDestCost(0);
  if (end_node_ == nullptr) {
    std::cout << "end node setting failure!" << std::endl;
    return false;
  }
  return true;
}

bool GridMap::setBounds(double xmin, double xmax, double ymin, double ymax) {
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

void GridMap::addObstacles(double xmin, double xmax, double ymin, double ymax) {
  std::vector<int> grid_ob{
      static_cast<int>((xmin - XYbounds_[0]) / xy_grid_resolution_),
      static_cast<int>((xmax - XYbounds_[0]) / xy_grid_resolution_) + 1,
      static_cast<int>((ymin - XYbounds_[2]) / xy_grid_resolution_),
      static_cast<int>((ymax - XYbounds_[2]) / xy_grid_resolution_) + 1};
  grid_obstacles_.emplace_back(grid_ob);
}

bool GridMap::setXYResolution(double resolution) {
  xy_grid_resolution_ = resolution;
  // max_grid_x_ = (XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_ + 1;
  // max_grid_y_ = (XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_ + 1;
}

// bool GridMap::CheckConstraints(std::shared_ptr<Node2d> node) {
//   const int node_grid_x = node->GetGridX();
//   const int node_grid_y = node->GetGridY();
//   if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
//       node_grid_y > max_grid_y_ || node_grid_y < 0) {
//     return false;
//   }
//   // if (grid_obstacles_.empty()) {
//   //   return true;
//   // }
//   // for (const auto& ob : grid_obstacles_) {
//   //   if (ob[0] < node->GetGridX() && node->GetGridX() < ob[1] &&
//   //       ob[2] < node->GetGridY() && node->GetGridY() < ob[3]) {
//   //     return false;
//   //   }
//   // }
//   return true;
// }

bool GridMap::insideMapRange(const int node_grid_x, const int node_grid_y) {
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  return true;
}

std::vector<std::shared_ptr<Node2d>> GridMap::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetCost();
  double diagonal_distance = std::sqrt(2.0);
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up =
      createNodeFromGridCoord(current_node_x, current_node_y + 1.0);
  up->SetDestCost(current_node_path_cost + 1.0);
  // std::shared_ptr<Node2d> up_right =
  //     createNodeFromGridCoord(current_node_x + 1.0, current_node_y + 1.0);
  // up_right->SetDestCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      createNodeFromGridCoord(current_node_x + 1.0, current_node_y);
  right->SetDestCost(current_node_path_cost + 1.0);
  // std::shared_ptr<Node2d> down_right =
  //     createNodeFromGridCoord(current_node_x + 1.0, current_node_y - 1.0);
  // down_right->SetDestCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      createNodeFromGridCoord(current_node_x, current_node_y - 1.0);
  down->SetDestCost(current_node_path_cost + 1.0);
  // std::shared_ptr<Node2d> down_left =
  //     createNodeFromGridCoord(current_node_x - 1.0, current_node_y - 1.0);
  // down_left->SetDestCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      createNodeFromGridCoord(current_node_x - 1.0, current_node_y);
  left->SetDestCost(current_node_path_cost + 1.0);
  // std::shared_ptr<Node2d> up_left =
  //     createNodeFromGridCoord(current_node_x - 1.0, current_node_y + 1.0);
  // up_left->SetDestCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  // next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  // next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  // next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  // next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridMap::GenerateDistanceMap() {
  struct cmp {
    bool operator()(const std::shared_ptr<Node2d> left,
                    const std::shared_ptr<Node2d> right) const {
      return left->GetDist() >= right->GetDist();
    }
  };
  
    std::priority_queue<std::shared_ptr<Node2d>,
                      std::vector<std::shared_ptr<Node2d>>, cmp>
      pq_;
      for (std::string node_name : border_available_) {
          pq_.push(heuristic_map_[node_name]);
      }
  
      // to do: use the border to find dist map (the cloest obstacle for each node)
  }

bool GridMap::GenerateHeuristicMap() {
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

  while (pq_.size() > 0) {
    std::shared_ptr<Node2d> cur_node = pq_.top();
    pq_.pop();
    if (heuristic_map_.find(cur_node->GetIndex()) != heuristic_map_.end()) {
      continue;
    }
    heuristic_map_[cur_node->GetIndex()] = cur_node;

    std::string cur_name = cur_node->GetIndex();
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(cur_node));
    for (auto& next_node : next_nodes) {
      if (!insideMapRange(next_node->GetGridX(), next_node->GetGridY()) ||
          next_node->GetCost() == std::numeric_limits<double>::max()) {
        border_available_.emplace(cur_node->GetIndex());
        continue;
      }

      max_cost = std::max(max_cost, next_node->GetCost());
      pq_.push(next_node);
    }
  }
  std::cout << "Heuristic Map generated successfully! size: "
            << heuristic_map_.size() << std::endl;

  return true;
}

bool GridMap::mapInitialization() {
  int DIRS[][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
  std::deque<std::shared_ptr<Node2d>> dq;

  dq.push_front(start_node_);
  std::cout << "start node " << start_node_->GetGridX() << " "
            << start_node_->GetGridY() << std::endl;

  while (!dq.empty()) {
    std::shared_ptr<Node2d> node = dq.back();
    dq.pop_back();
    for (int i = 0; i < 4; i++) {
      int nx = node->GetGridX() + DIRS[i][0];
      int ny = node->GetGridY() + DIRS[i][1];
      if (!insideMapRange(nx, ny)) {
        continue;
      }
      std::string next_name = Node2d::ComputeStringIndex(nx, ny);
      if (heuristic_map_.find(next_name) != heuristic_map_.end()) {
        continue;
      }

      auto next_node = createNodeFromGridCoord(nx, ny);
      next_node->SetDestCost(0);
      dq.push_front(next_node);
      heuristic_map_[next_name] = next_node;
      std::cout << "  next node " << next_node->GetGridX() << " "
                << next_node->GetGridY() << std::endl;
    }
  }
}

void GridMap::addPolygonObstacles(geometry_msgs::Polygon p) {
  if (p.points.empty()) {
    ROS_INFO("Polygon Obstacle empty!");
    return;
  }

  // the segments are as many as the points.
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
    double start_x = vec_start_x[i];
    double start_y = vec_start_y[i];
    double end_x = vec_end_x[i];
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
      std::shared_ptr<Node2d> grid_p = getNodeFromWorldCoord(x, y);
      if (grid_p == nullptr) {
        grid_p = createNodeFromWorldCoord(x, y);
      }
      grid_p->SetDestCost(std::numeric_limits<double>::max());
      heuristic_map_[grid_p->GetIndex()] = grid_p;
      // std::cout << "X: " << grid_p->GetGridX() * xy_grid_resolution_ << " Y:"
      // << grid_p->GetGridY() * xy_grid_resolution_
      //           << " " << grid_p->GetCost() << std::endl;

      x += delta_x;
      y += delta_y;
    }
  }
}

std::shared_ptr<Node2d> GridMap::getNodeFromWorldCoord(double x, double y) {
  int x_grid = static_cast<int>((x - XYbounds_[0]) / xy_grid_resolution_);
  int y_grid = static_cast<int>((y - XYbounds_[2]) / xy_grid_resolution_);
  std::string name = std::to_string(x_grid) + "_" + std::to_string(y_grid);
  if (heuristic_map_.find(name) == heuristic_map_.end()) {
    return nullptr;
  }
  return heuristic_map_[name];
}

void GridMap::plotHeuristicMap(double xy_grid_resolution) {
  marker_array.markers.clear();
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";

  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  int marker_id = 0;

  std::unordered_map<std::string, std::shared_ptr<Node2d>>::iterator iter =
      heuristic_map_.begin();
  while (iter != heuristic_map_.end()) {
    std::shared_ptr<Node2d> node = iter->second;
    marker.id = marker_id;
    marker.color.r = 1.0f - node->GetCost() / max_cost;
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

  pub.publish(marker_array);
  std::cout << "marker array with size" << marker_array.markers.size()
            << " published!" << std::endl;
  return;
}

double GridMap::getHeuristic(std::string s) {
  if (heuristic_map_.find(s) == heuristic_map_.end()) {
    return std::numeric_limits<double>::max();
  }
  return heuristic_map_[s]->GetCost();
}

void GridMap::clearObstacles() { grid_obstacles_.clear(); }

void GridMap::clearMap() { heuristic_map_.clear(); }