#pragma once
#include "Node.h"

class Node3d : public Node {
 public:
  Node3d(const double x, const double y, const double phi,
         const double xy_resolution, const double phi_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    x_ = x;
    y_ = y;
    phi_ = phi;
    while (phi_ >= 2 * M_PI) {
      phi_ -= 2 * M_PI;
    }
    while (phi_ < 0) {
      phi_ += 2 * M_PI;
    }

    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    grid_phi_ = static_cast<int>((phi) / phi_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_, grid_phi_);
  }

  Node3d(const int grid_x, const int grid_y, const int grid_phi) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    grid_phi_ = grid_phi;
    index_ = ComputeStringIndex(grid_x_, grid_y_, grid_phi_);
  }

  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    if (path_cost_ == std::numeric_limits<double>::max() ||
        heuristic_cost_ == std::numeric_limits<double>::max()) {
      return;
    }
    cost_ = path_cost_ + heuristic_cost_;
  }
  
  void SetHeuristicCost(const double heuristic) {
    heuristic_cost_ = heuristic;
    if (path_cost_ == std::numeric_limits<double>::max() ||
        heuristic_cost_ == std::numeric_limits<double>::max()) {
      return;
    }
    cost_ = path_cost_ + heuristic_cost_;
  }

  void SetDirection(bool forward) { forward_ = forward; }
  void SetSteer(double steer) { steer_ = steer; }
  void SetPreNode(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }

  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() {
    while (phi_ >= 2 * M_PI) {
      phi_ -= 2 * M_PI;
    }
    while (phi_ < 0) {
      phi_ += 2 * M_PI;
    }
    return phi_;
  }
  double GetSteer() const { return steer_; }
  bool GetDirection() { return forward_; }
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }

  double GetPathCost() const { return path_cost_; }
  double GetCost() const { return cost_; }

  const std::string Cal2dIndex() {
    return ComputeStringIndex(grid_x_, grid_y_);
  }

  bool operator==(const Node3d& right) const {
    return right.GetIndex() == index_;
  }
  int analysis_step = 0;

 private:
  double x_ = 0;
  double y_ = 0;
  double phi_ = 0;
  int grid_phi_ = 0;

  double path_cost_ = std::numeric_limits<double>::max();
  double heuristic_cost_ = std::numeric_limits<double>::max();
  double cost_ = std::numeric_limits<double>::max();
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  double steer_ = 0;
  bool forward_ = true;
};