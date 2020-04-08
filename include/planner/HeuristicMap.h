#pragma once
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "planner/Node2d.h"
#include "planner/Node3d.h"

// struct GridAStartResult {
//   std::vector<double> x;
//   std::vector<double> y;
//   double path_cost = 0.0;
// };

class HeuristicMap {
 public:
  HeuristicMap() {
    pub = nh.advertise<visualization_msgs::MarkerArray>("vis_obstacle", 10);
  };
  ~HeuristicMap() = default;

  bool setXYResolution(double resolution);
  bool setStartPoint(double x, double y, double phi);
  bool setEndPoint(double x, double y, double phi);
  bool setBounds(double xmin, double xmax, double ymin, double ymax);
  bool addObstacles(double left, double right, double up, double bottom);
  bool GenerateHeuristicMap();
  void plotHeuristicMap();
  double getHeuristic(std::string s);

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
  //   void LoadGridAStarResult(GridAStartResult* result);

  double xy_grid_resolution_ = 0.1;
  double phi_grid_resolution_ = 0.2;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_{0, 10, 0, 10};
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;

  std::vector<std::vector<int>> grid_obstacles_;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  std::unordered_map<std::string, std::shared_ptr<Node2d>> heuristic_map_;

  ros::NodeHandle nh;
  ros::Publisher pub;

  double max_cost = std::numeric_limits<double>::min();
};
