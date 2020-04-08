#include "planner/HybridAStarSearchMap.h"

void HybridAStarSearchMap::setXYResolution(double resolution) {
  heuristic_map_.setXYResolution(resolution);
  xy_grid_resolution_ = resolution;
}

void HybridAStarSearchMap::setStartPoint(double x, double y, double phi) {
  heuristic_map_.setStartPoint(x, y, phi);
  start_node_ = std::make_shared<Node3d>(x, y, phi, xy_grid_resolution_,
                                         phi_grid_resolution_, XYbounds_);
  start_node_->SetPathCost(0);
}

void HybridAStarSearchMap::setEndPoint(double x, double y, double phi) {
  heuristic_map_.setEndPoint(x, y, phi);
  end_node_ = std::make_shared<Node3d>(x, y, phi, xy_grid_resolution_,
                                       phi_grid_resolution_, XYbounds_);
}

void HybridAStarSearchMap::setBounds(double xmin, double xmax, double ymin,
                                     double ymax) {
  heuristic_map_.setBounds(xmin, xmax, ymin, ymax);
  if (!XYbounds_.empty()) {
    XYbounds_.clear();
  }
  XYbounds_.emplace_back(xmin);
  XYbounds_.emplace_back(xmax);
  XYbounds_.emplace_back(ymin);
  XYbounds_.emplace_back(ymax);
}

bool HybridAStarSearchMap::pointIsValid(double x, double y) {
  if (x <= XYbounds_[0] || x >= XYbounds_[1] || y <= XYbounds_[2] ||
      y >= XYbounds_[3]) {
    return false;
  }

  for (auto ob : obstacles_) {
    if (ob[0] <= x && x <= ob[1] && ob[2] <= y && y <= ob[3]) {
      return false;
    }
  }
  return true;
}

void HybridAStarSearchMap::addObstacles(double xmin, double xmax, double ymin,
                                        double ymax) {
  heuristic_map_.addObstacles(xmin, xmax, ymin, ymax);
  std::vector<double> ob{xmin, xmax, ymin, ymax};
  obstacles_.emplace_back(ob);
}

void HybridAStarSearchMap::plot() {
  heuristic_map_.plotHeuristicMap();

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
    node = node->GetPreNode();
  }

  pub.publish(marker_array);
}

void HybridAStarSearchMap::GenerateHeuristicMap() {
  heuristic_map_.GenerateHeuristicMap();
  start_node_->SetHeuristicCost(
      heuristic_map_.getHeuristic(start_node_->cal2dIndex()));
  end_node_->SetHeuristicCost(
      heuristic_map_.getHeuristic(end_node_->cal2dIndex()));
}

bool HybridAStarSearchMap::isTerminateState(std::shared_ptr<Node3d> node) {
  double x_diff = end_node_->GetX() - node->GetX();
  double y_diff = end_node_->GetY() - node->GetY();
  double phi_diff = end_node_->GetPhi() - node->GetPhi();
  return std::sqrt(x_diff * x_diff + y_diff * y_diff) < xy_grid_resolution_ &&
         std::abs(phi_diff) < phi_grid_resolution_;
}

void HybridAStarSearchMap::update(double& x, double& y, double& phi,
                                  double steer, double dis) {
  x += dis * std::cos(phi);
  y += dis * std::sin(phi);
  phi += dis / WHEEL_BASE * std::tan(steer);
}

void HybridAStarSearchMap::nextNodeGenerator(
    std::vector<std::shared_ptr<Node3d>>& next_nodes,
    std::shared_ptr<Node3d> cur_node, double step_size) {
  for (int index = 0; index <= next_node_num_; index++) {
    double steer = -MAX_STEER + 2 * MAX_STEER / next_node_num_ * index;
    double last_x = cur_node->GetX();
    double last_y = cur_node->GetY();
    double last_phi = cur_node->GetPhi();
    bool flag = true;

    for (int i = 0; i <= xy_grid_resolution_ * 1.41 / std::abs(step_size_);
         i++) {
      update(last_x, last_y, last_phi, steer, step_size_);
      if (!pointIsValid(last_x, last_y)) {
        flag = false;
        break;
      }
    }

    if (flag) {
      double path_cost =
          cur_node->GetPathCost() + MOVEMENT_PENALTY +
          std::abs(steer - cur_node->GetSteer() * STEER_CHANGE_PENALTY) +
          std::abs(steer) * STEER_PENALTY;
      if (std::isinf(path_cost)) {
        continue;
      }

      std::shared_ptr<Node3d> p = std::make_shared<Node3d>(
          last_x, last_y, last_phi, xy_grid_resolution_, phi_grid_resolution_,
          XYbounds_);
      p->SetSteer(steer);
      p->SetHeuristicCost(heuristic_map_.getHeuristic(p->cal2dIndex()));
      p->SetPathCost(path_cost);

      p->SetPreNode(cur_node);
      next_nodes.emplace_back(p);
    }
  }
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

  while (open_pq.size() > 0) {
    std::shared_ptr<Node3d> node = open_pq.top();
    open_pq.pop();

    map_[node->GetIndex()] = node;
    max_cost = std::max(max_cost, node->GetCost());

    std::vector<std::shared_ptr<Node3d>> nextnodes;
    nextNodeGenerator(nextnodes, node, step_size_);
    nextNodeGenerator(nextnodes, node, -step_size_);

    for (std::shared_ptr<Node3d> nextnode : nextnodes) {
      if (isTerminateState(nextnode)) {
        final_node_ = nextnode;
        std::cout << "Trajectory found!" << std::endl;
        return;
      }

      if (map_.find(nextnode->GetIndex()) == map_.end() ||
          map_[nextnode->GetIndex()]->GetCost() > nextnode->GetCost()) {
        map_[nextnode->GetIndex()] = nextnode;
        open_pq.push(nextnode);
      }
    }
  }

  std::cout << "Path not found!" << std::endl;
  return;
}
