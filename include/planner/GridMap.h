#pragma once
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <deque>
#include <set>
#include <vector>

#include "planner/Node2d.h"
#include "planner/Node3d.h"

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

  // node generation
  std::shared_ptr<Node2d> CreateNodeFromWorldCoord(const double x,
                                                   const double y);
  std::shared_ptr<Node2d> CreateNodeFromGridCoord(const int x, const int y);
  std::shared_ptr<Node2d> GetNodeFromWorldCoord(const double x, const double y);
  std::shared_ptr<Node2d> GetNodeFromGridCoord(const int x_grid,
                                               const int y_grid);

  // map configuration
  bool SetXYResolution(double resolution);
  bool SetPhiResolution(double resolution);
  bool SetStartPoint(double x, double y);
  bool SetEndPoint(double x, double y);
  bool SetBounds(double xmin, double xmax, double ymin, double ymax);

  // add obstacles into the map
  void AddPolygonObstacles(geometry_msgs::Polygon p, double LIDAR_TO_REAR);

  // heuristic map & obstacle map
  bool GenerateDestinationDistanceMap();
  bool GenerateObstacleDistanceMap();
  void Reset();

  // get the 2d heuristic value
  double GetHeuristic(std::string s);

  // plot
  void PlotHeuristicMap(double xy_grid_resolution);
  void PlotBorders(double xy_grid_resolution);
  void PlotObstacleMap(double xy_grid_resolution);
  std::unordered_map<std::string, std::shared_ptr<Node2d>> map_2d_;

 private:
  // for expansion
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool InsideGridMap(const int node_grid_x, const int node_grid_y);
  bool InsideWorldMap(const double x, const double y);

  // map settings
  std::shared_ptr<Node2d> start_node_ = nullptr;
  std::shared_ptr<Node2d> end_node_ = nullptr;

  std::vector<double> XYbounds_{-10, 10, -10, 10};
  double xy_grid_resolution_ = 0.3;
  double phi_grid_resolution_ = 0.2;

  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;

  // visualization
  ros::NodeHandle nh;
  ros::Publisher pub_map, pub_border, pub_obstacle;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  // obstacle borders
  std::set<std::string> border_available_;
  std::set<std::string> border_unavailable_;

  double max_cost = std::numeric_limits<double>::min();
};