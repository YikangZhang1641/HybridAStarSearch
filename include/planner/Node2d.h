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

class Node2d {
 public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }

  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }

  void SetDestinationCost(const double dest_cost) {
    destination_cost_ = dest_cost;
  }
  void SetObstacleDistance(const double dis) { obstacle_distance_ = dis; }

  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetDestinationCost() const { return destination_cost_; }
  double GetObstacleDistance() const { return obstacle_distance_; }
  double GetCost() const {
    if (unavailable_) {
      return std::numeric_limits<double>::max();
    }
    return destination_cost_ ;
  }

  bool IsUnavailable() { return unavailable_; }
  void SetUnavailable() { unavailable_ = true; }

  const std::string& GetIndex() const { return index_; }

  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return std::to_string(x_grid) + "_" + std::to_string(y_grid);
  }

 private:
  int grid_x_ = 0;
  int grid_y_ = 0;
  double destination_cost_ = std::numeric_limits<double>::max();
  double obstacle_distance_ = 1e-5;
  std::string index_;
  bool unavailable_ = false;
};