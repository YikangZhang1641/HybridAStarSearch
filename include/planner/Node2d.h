#pragma once
#include "planner/Node.h"

class Node2d : public Node {
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
  void SetObstacleDistance(const int dis) { obstacle_distance_ = dis; }

  double GetDestinationCost() const { return destination_cost_; }
  double GetObstacleDistance() const { return obstacle_distance_; }
  double GetCost() const {
    if (unavailable_) {
      return std::numeric_limits<double>::max();
    }
    return destination_cost_;
  }

  bool IsUnavailable() { return unavailable_; }
  void SetUnavailable() { unavailable_ = true; }
  void SetAvailable() { unavailable_ = false; }

  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }
  int expanded = 0;
  double debug_cost = std::numeric_limits<double>::max();

 private:
  double destination_cost_ = std::numeric_limits<double>::max();
  int obstacle_distance_ = std::numeric_limits<int>::max();
};