#include "planner/HeuristicMap.h"

bool HeuristicMap::setStartPoint(double x, double y, double phi) {
  start_node_ = std::make_shared<Node2d>(x, y, xy_grid_resolution_, XYbounds_);
  if (start_node_ == nullptr) {
    std::cout << "start node setting failure!" << std::endl;
    return false;
  }
  return true;
}

bool HeuristicMap::setEndPoint(double x, double y, double phi) {
  end_node_ = std::make_shared<Node2d>(x, y, xy_grid_resolution_, XYbounds_);
  end_node_->SetCost(0);
  if (end_node_ == nullptr) {
    std::cout << "end node setting failure!" << std::endl;
    return false;
  }
  return true;
}

bool HeuristicMap::setBounds(double xmin, double xmax, double ymin,
                             double ymax) {
  if (!XYbounds_.empty()) {
    XYbounds_.clear();
  }
  XYbounds_.emplace_back(xmin);
  XYbounds_.emplace_back(xmax);
  XYbounds_.emplace_back(ymin);
  XYbounds_.emplace_back(ymax);

  max_grid_x_ = static_cast<int>((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_) + 1;
  max_grid_y_ = static_cast<int>((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_) + 1;
  return XYbounds_.size() == 4;
}

bool HeuristicMap::addObstacles(double xmin, double xmax, double ymin,
                                double ymax) {
  std::vector<int> grid_ob{
      static_cast<int>((xmin - XYbounds_[0]) / xy_grid_resolution_),
      static_cast<int>((xmax - XYbounds_[0]) / xy_grid_resolution_) + 1,
      static_cast<int>((ymin - XYbounds_[2]) / xy_grid_resolution_),
      static_cast<int>((ymax - XYbounds_[2]) / xy_grid_resolution_) + 1};
  grid_obstacles_.emplace_back(grid_ob);
  return !grid_obstacles_.empty();
}

bool HeuristicMap::setXYResolution(double resolution) {
  xy_grid_resolution_ = resolution;
  // max_grid_x_ = (XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_ + 1;
  // max_grid_y_ = (XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_ + 1;
}

bool HeuristicMap::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  if (grid_obstacles_.empty()) {
    return true;
  }
  for (const auto& ob : grid_obstacles_) {
    if (ob[0] < node->GetGridX() && node->GetGridX() < ob[1] &&
        ob[2] < node->GetGridY() && node->GetGridY() < ob[3]) {
      return false;
    }
  }
  return true;
}

std::vector<std::shared_ptr<Node2d>> HeuristicMap::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetCost();
  double diagonal_distance = std::sqrt(2.0);
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up =
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool HeuristicMap::GenerateHeuristicMap() {
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

    std::string cur_name = cur_node->GetIndex();
    if (heuristic_map_.find(cur_name) == heuristic_map_.end() ||
        heuristic_map_[cur_name]->GetCost() > cur_node->GetCost()) {
      heuristic_map_.emplace(cur_node->GetIndex(), cur_node);
    }

    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(cur_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }

      max_cost = std::max(max_cost, next_node->GetCost());

      if (heuristic_map_.find(next_node->GetIndex()) == heuristic_map_.end() ||
          heuristic_map_[next_node->GetIndex()]->GetCost() >
              next_node->GetCost()) {
        heuristic_map_[next_node->GetIndex()] = next_node;
        pq_.push(next_node);
        // std::cout << "generate map node: " << next_node->GetIndex()
                  // << std::endl;
        continue;
      }
    }
  }

  std::cout << "Heuristic Map generated successfully! size: "
            << heuristic_map_.size() << std::endl;

  // start_node_->SetCost(heuristic_map_[start_node_->GetIndex()]->GetCost());
  return true;
}

void HeuristicMap::plotHeuristicMap() {
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
    marker.pose.position.x = node->GetGridX();
    marker.pose.position.y = node->GetGridY();
    marker.pose.position.z = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker_array.markers.push_back(marker);
    ++marker_id;
    ++iter;
  }

  pub.publish(marker_array);
  std::cout << "marker array with size" << marker_array.markers.size()
            << " published!" << std::endl;
  return;
}

double HeuristicMap::getHeuristic(std::string s) {
  if (heuristic_map_.find(s) == heuristic_map_.end()) {
    return std::numeric_limits<double>::max();
  }
  return heuristic_map_[s]->GetCost();
}