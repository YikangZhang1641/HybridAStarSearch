#include "planner/GridMap.h"

namespace udrive {
namespace planning {

class HybridAStarSearchMap {
 public:
  HybridAStarSearchMap() {
    pub = nh.advertise<visualization_msgs::MarkerArray>("vis_map", 10);
  }

  std::shared_ptr<Node3d> CreateNodeFromWorldCoord(const double x,
                                                   const double y,
                                                   const double phi);
  std::shared_ptr<Node3d> GetNodeFromWorldCoord(const double x, const double y,
                                                const double phi);
  std::shared_ptr<Node3d> GetNodeFromGridCoord(const int x, const int y,
                                               const int phi);
  std::string Calc3dIndex(double x, double y, double phi);

  // initialization
  void SetXYResolution(double resolution);
  void SetPhiResolution(double resolution);
  bool SetStartPoint(double x, double y, double phi);
  bool SetEndPoint(double x, double y, double phi);
  void SetBounds(double xmin, double xmax, double ymin, double ymax);
  bool CheckStartEndPoints();
  void Reset();

  // search
  void AddObstacles(geometry_msgs::Polygon p);
  void GenerateHeuristicMap();
  void Search();
  double GetObstacleDistance(std::shared_ptr<Node3d> p);
  double GetObstacleDistance(double x, double y);
  double ObstacleDistancePenalty(double dis);

  // state check
  bool InsideGridMap(const int node_grid_x, const int node_grid_y);
  bool InsideWorldMap(const double x, const double y);
  bool CollisionDection(double x, double y, double phi);

  // plot
  void PlotHeuristicMap();
  void PlotTrajectory();

 private:
  // node expansion
  bool IsTerminateState(std::shared_ptr<Node3d> node);
  void Update(double& x, double& y, double& phi, double steer, double dis);
  void NextNodeGenerator(std::vector<std::shared_ptr<Node3d>>& next_nodes,
                         std::shared_ptr<Node3d>, double step_size);

  // heuristic map and search map
  GridMap grid_map_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> map_3d_;
  std::shared_ptr<Node3d> start_node_ = nullptr;
  std::shared_ptr<Node3d> end_node_ = nullptr;
  std::shared_ptr<Node3d> final_node_ = nullptr;

  // map params
  std::vector<double> XYbounds_{0, 10, 0, 10};
  int max_grid_x_ = 0;
  int max_grid_y_ = 0;

  // visualization
  ros::NodeHandle nh;
  ros::Publisher pub;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  // params nees to be set manually
  double xy_grid_resolution_ = 0.3;
  double phi_grid_resolution_ = 0.2;
  int next_node_num_ = 5;
  double step_size_ = 0.02;
  double MAX_STEER = 0.8;
  double WHEEL_BASE = 2.0;
  double VEHICLE_L = 2.0;
  double VEHICLE_W = 1.0;

  // penalty
  double FORWARD_PENALTY = 1;
  double BACKWARD_PENALTY = 5;
  double STEER_PENALTY = 1;
  double STEER_CHANGE_PENALTY = 1;

  // counter
  double max_cost = std::numeric_limits<double>::min();
  int count = 0;
};
}  // namespace planning
}  // namespace udrive