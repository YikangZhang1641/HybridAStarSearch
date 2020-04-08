#include "planner/HybridAStarSearchMap.h"

void HybridAStarSearchMap::setXYResolution(double resolution) {
  heuristic_map.setXYResolution(resolution);
  xy_grid_resolution_ = resolution;
}

void HybridAStarSearchMap::setStartPoint(double x, double y, double phi) {
  heuristic_map.setStartPoint(x, y, phi);
  start_node_ = std::make_shared<Node3d>(x, y, phi, xy_grid_resolution_,
                                         phi_grid_resolution_, XYbounds_);
}

void HybridAStarSearchMap::setEndPoint(double x, double y, double phi) {
  heuristic_map.setEndPoint(x, y, phi);
  end_node_ = std::make_shared<Node3d>(x, y, phi, xy_grid_resolution_,
                                       phi_grid_resolution_, XYbounds_);
}

void HybridAStarSearchMap::setBounds(double xmin, double xmax, double ymin,
                                     double ymax) {
  heuristic_map.setBounds(xmin, xmax, ymin, ymax);
  if (!XYbounds_.empty()) {
    XYbounds_.clear();
  }
  XYbounds_.emplace_back(xmin);
  XYbounds_.emplace_back(xmax);
  XYbounds_.emplace_back(ymin);
  XYbounds_.emplace_back(ymax);
}

void HybridAStarSearchMap::addObstacles(double xmin, double xmax, double ymin,
                                        double ymax) {
  heuristic_map.addObstacles(xmin, xmax, ymin, ymax);
  std::vector<double> ob{xmin, xmax, ymin, ymax};
  obstacles_.emplace_back(ob);
}

void HybridAStarSearchMap::plot() { heuristic_map.plotHeuristicMap(); }

void HybridAStarSearchMap::GenerateHeuristicMap() {
  heuristic_map.GenerateHeuristicMap();
  readFromHeuristicMap(start_node_);
  readFromHeuristicMap(end_node_);
}

void HybridAStarSearchMap::readFromHeuristicMap(std::shared_ptr<Node3d>& node) {
  node->SetHeuristic(heuristic_map.getHeuristic(node->Get2dIndex()));
}

void HybridAStarSearchMap::update(double& x, double& y, double& phi,
                                  double steer, double dis) {
  x += dis * std::cos(phi);
  y += dis * std::sin(phi);
  phi += dis / WHEEL_BASE * std::tan(steer);
}

void HybridAStarSearchMap::nextNodeGenerator(
    std::vector<std::shared_ptr<Node3d>>& nextnodes,
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

    std::vector<std::shared_ptr<Node3d>> nextnodes;
    nextNodeGenerator(nextnodes, node, step_size_);
    nextNodeGenerator(nextnodes, node, -step_size_);

    for (std::shared_ptr<Node3d> nextnode : nextnodes) {
    }
  }
}
