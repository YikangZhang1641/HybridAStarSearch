#pragma once
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "math.h"

class Node3d {
 public:
  bool operator<(Node3d& n) const { return GetCost() > n.GetCost(); }

  Node3d(const double x, const double y, const double phi,
         const double xy_resolution, const double phi_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    x_ = x;
    y_ = y;
    phi_ = phi;
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    grid_phi_ = static_cast<int>((phi + M_PI) / phi_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_, grid_phi_);
  }

  Node3d(const int grid_x, const int grid_y, const int grid_phi) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    grid_phi_ = grid_phi;
    index_ = ComputeStringIndex(grid_x_, grid_y_, grid_phi_);
  }

  void SetPathCost(const double path_cost) { path_cost_ = path_cost; }
  void SetHeuristicCost(const double heuristic) { heuristic_cost_ = heuristic; }


  void SetSteer(double steer) { steer_ = steer; }
  void SetPreNode(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }

  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }

  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }

  double GetPathCost() const { return path_cost_; }
  double GetHeuCost() const { return heuristic_cost_; }

  double GetCost() const {
    if (path_cost_ == std::numeric_limits<double>::max() ||
        heuristic_cost_ == std::numeric_limits<double>::max()) {
      return std::numeric_limits<double>::max();
    }
    return path_cost_ + heuristic_cost_;
  }

  double GetSteer() const { return steer_; }

  const std::string& GetIndex() const { return index_; }
  const std::string Cal2dIndex() {
    return std::to_string(grid_x_) + "_" + std::to_string(grid_y_);
  }

  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }

  bool operator==(const Node3d& right) const {
    return right.GetIndex() == index_;
  }

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
    return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
           std::to_string(phi_grid);
  }

 private:
  double x_ = 0, y_ = 0, phi_ = 0;
  int grid_x_ = 0;
  int grid_y_ = 0;
  int grid_phi_ = 0;
  double path_cost_ = std::numeric_limits<double>::max();
  double heuristic_cost_ = std::numeric_limits<double>::max();
  double obstacle_cost_ = std::numeric_limits<double>::max();
  std::string index_;
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  double steer_ = 0;

  bool unavailable = false;
};