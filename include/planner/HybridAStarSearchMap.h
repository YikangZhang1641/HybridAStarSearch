#include "planner/HeuristicMap.h"

class HybridAStarSearchMap {
 public:
  HybridAStarSearchMap() {
    pub = nh.advertise<visualization_msgs::MarkerArray>("vis_map", 10);
  }
  void Search();
  void setXYResolution(double resolution);
  void setStartPoint(double x, double y, double phi);
  void setEndPoint(double x, double y, double phi);
  void setBounds(double xmin, double xmax, double ymin, double ymax);
  void addObstacles(double left, double right, double up, double bottom);
  void GenerateHeuristicMap();
  bool pointIsValid(double x, double y);
  void plot();

 private:
  HeuristicMap heuristic_map_;
  bool isTerminateState(std::shared_ptr<Node3d> node);
  void nextNodeGenerator(std::vector<std::shared_ptr<Node3d>>& nextnodes,
                         std::shared_ptr<Node3d>, double step_size);
  void update(double& x, double& y, double& phi, double steer, double dis);

  double step_size_ = 0.02;
  std::vector<std::vector<double>> obstacles_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> map_;

  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<double> XYbounds_{0, 10, 0, 10};

  ros::NodeHandle nh;
  ros::Publisher pub;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  double max_cost = std::numeric_limits<double>::min();

  int next_node_num_ = 5;
  double MAX_STEER = 1.0;
  double WHEEL_BASE = 1.0;
  double MOVEMENT_PENALTY = 1.0;
  double STEER_PENALTY = 1;
  double STEER_CHANGE_PENALTY = 2;

  double xy_grid_resolution_ = 0.1;
  double phi_grid_resolution_ = 0.2;
};