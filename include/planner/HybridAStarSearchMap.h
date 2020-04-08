#include "planner/HeuristicMap.h"

class HybridAStarSearchMap {
 public:
  HybridAStarSearchMap() = default;
  void Search();
  void setXYResolution(double resolution);
  void setStartPoint(double x, double y, double phi);
  void setEndPoint(double x, double y, double phi);
  void setBounds(double xmin, double xmax, double ymin, double ymax);
  void addObstacles(double left, double right, double up, double bottom);
  void GenerateHeuristicMap();
  void readFromHeuristicMap(std::shared_ptr<Node3d> node);
  bool pointIsValid(double x, double y);
  void plot();

 private:
  void nextNodeGenerator(std::vector<std::shared_ptr<Node3d>>& nextnodes,
                         std::shared_ptr<Node3d>, double step_size);
  void update(double& x, double& y, double& phi, double steer, double dis);
  HeuristicMap heuristic_map;

  double step_size_ = 0.02;
  std::vector<std::vector<double>> obstacles_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> map_;

  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<double> XYbounds_{0, 10, 0, 10};

  int next_node_num_ = 5;
  double MAX_STEER = 1.0;
  double WHEEL_BASE = 1.0;
  double xy_grid_resolution_ = 0.1;
  double phi_grid_resolution_ = 0.2;
};