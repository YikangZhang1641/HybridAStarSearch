#pragma once
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <deque>
#include <set>
#include <vector>

#include "hydra_utils/hydra_utils/proto/proto_file_utils_lib.hpp"
#include "planner/Node2d.h"
#include "planner/Node3d.h"

namespace udrive {
namespace planning {

class GridMap {
 public:
  GridMap() {
    pub_map =
        nh.advertise<visualization_msgs::MarkerArray>("vis_heuristic_map", 10);
    pub_border =
        nh.advertise<visualization_msgs::MarkerArray>("vis_border", 10);
    pub_obstacle =
        nh.advertise<visualization_msgs::MarkerArray>("vis_obstacle", 10);
  };
  ~GridMap() = default;

  // configuration
  bool SetXYResolution(double resolution);
  bool SetStartPoint(double x, double y);
  bool SetEndPoint(double x, double y);
  bool SetBounds(double xmin, double xmax, double ymin, double ymax);

  // node
  std::shared_ptr<Node2d> CreateNodeFromWorldCoord(double x, double y);
  std::shared_ptr<Node2d> CreateNodeFromGridCoord(int x, int y);
  std::shared_ptr<Node2d> GetNodeFromWorldCoord(double x, double y);
  std::shared_ptr<Node2d> GetNodeFromGridCoord(int x_grid, int y_grid);

  // add obstacles into the map
  void AddPolygonObstacles(geometry_msgs::Polygon p);

  // heuristic map & obstacle map
  bool GenerateDestinationDistanceMap();
  bool GenerateObstacleDistanceMap();
  void ClearMap();

  // get the 2d heuristic value
  double GetHeuristic(std::string s);

  // plot
  void PlotHeuristicMap(double xy_grid_resolution);
  void PlotBorders(double xy_grid_resolution);
  void PlotObstacleMap(double xy_grid_resolution);

 private:
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool InsideMapRange(const int node_grid_x, const int node_grid_y);

  std::unordered_map<std::string, std::shared_ptr<Node2d>> map_2d_;

  double xy_grid_resolution_ = 0.3;
  double phi_grid_resolution_ = 0.2;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_{0, 10, 0, 10};
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_ = nullptr;
  std::shared_ptr<Node2d> end_node_ = nullptr;

  std::vector<std::vector<int>> grid_obstacles_;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  ros::NodeHandle nh;
  ros::Publisher pub_map, pub_border, pub_obstacle;

  double max_cost = std::numeric_limits<double>::min();
  std::set<std::string> border_available_;
  std::set<std::string> border_unavailable_;
};
}  // namespace planning
}  // namespace udrive