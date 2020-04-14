#include "planner/GridMap.h"

class HybridAStarSearchMap {
 public:
  HybridAStarSearchMap() {
    pub = nh.advertise<visualization_msgs::MarkerArray>("vis_map", 10);
  }

  std::shared_ptr<Node3d> CreateNodeFromWorldCoord(double x, double y,
                                                   double phi);
  std::shared_ptr<Node3d> CreateNodeFromGridCoord(int x, int y, int phi);
  std::shared_ptr<Node3d> GetNodeFromWorldCoord(double x, double y, double phi);
  std::shared_ptr<Node3d> GetNodeFromGridCoord(int x, int y, int phi);

  std::string Calc2dIndex(const int grid_x, const int grid_y);
  std::string Calc3dIndex(const int grid_x, const int grid_y,
                          const int grid_phi);

  void Search();
  void SetXYResolution(double resolution);
  bool SetStartPoint(double x, double y, double phi);
  bool SetEndPoint(double x, double y, double phi);
  void SetBounds(double xmin, double xmax, double ymin, double ymax);
  void ClearObstacles();
  bool CheckStartEndPoints();
  void GenerateHeuristicMap();
  void ClearMap();
  bool InsideWorldMap(double x, double y);
  bool CollisionDection(double x, double y, double phi);
  void PlotHeuristicMap();
  void PlotTrajectory();
  void AddObstacles(geometry_msgs::Polygon p);
  double ObsCostMapping(double dis) {
    if (dis <= 3) {
      return 1;
    }
    return 0;
  }

  double GetObstacleDistance(std::shared_ptr<Node3d> p) {
    return heuristic_map_.GetNodeFromGridCoord(p->GetGridX(), p->GetGridY())
        ->GetObstacleDistance();
  }

  double GetObstacleDistance(double x, double y) {
    return heuristic_map_.GetNodeFromWorldCoord(x, y)->GetObstacleDistance();
  }

 private:
  GridMap heuristic_map_;
  bool IsTerminateState(std::shared_ptr<Node3d> node);
  void NextNodeGenerator(std::vector<std::shared_ptr<Node3d>>& next_nodes,
                         std::shared_ptr<Node3d>, double step_size);
  void Update(double& x, double& y, double& phi, double steer, double dis);

  double step_size_ = 0.02;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> map_;

  std::shared_ptr<Node3d> start_node_ = nullptr;
  std::shared_ptr<Node3d> end_node_ = nullptr;
  std::shared_ptr<Node3d> final_node_ = nullptr;
  std::vector<double> XYbounds_{0, 10, 0, 10};

  ros::NodeHandle nh;
  ros::Publisher pub;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  double max_cost = std::numeric_limits<double>::min();

  double xy_grid_resolution_ = 0.3;
  double phi_grid_resolution_ = 0.2;

  int next_node_num_ = 5;
  double MAX_STEER = 0.8;
  double WHEEL_BASE = 2.0;
  double MOVEMENT_PENALTY = 1;
  double STEER_PENALTY = 0.5;
  double STEER_CHANGE_PENALTY = 1;

  double VEHICLE_L = 2.0;
  double VEHICLE_W = 1.0;

  int count = 0;
};