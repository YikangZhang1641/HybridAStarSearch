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


class Node {
 public:
  int GetGridX() const { return grid_x_; }
  int GetGridY() const { return grid_y_; }

  const std::string& GetIndex() const { return index_; }

  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return std::to_string(x_grid) + "_" + std::to_string(y_grid);
  }
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
    return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
           std::to_string(phi_grid);
  }

 protected:
  int grid_x_ = 0;
  int grid_y_ = 0;
  std::string index_;
  bool unavailable_ = false;
};