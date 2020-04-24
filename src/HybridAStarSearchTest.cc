#include "planner/HybridAStarSearchMap.h"

class ObstacleAnalyzer {
 public:
  ObstacleAnalyzer() = default;
  void Init() {
    search_map = std::make_shared<HybridAStarSearchMap>(
        "/home/ubuntu/catkin_ws/src/motion_planning/conf/"
        "motion_planning.pb.conf.sample");
    search_map->SetXYResolution(0.3);
    search_map->SetPhiResolution(0.2);
    search_map->SetBounds(-10, 10, -10, 10);

    sub_obstacle_ = nh_.subscribe("/obstacles", 1,
                                  &ObstacleAnalyzer::ObstacleHandler, this);
    sub_rviz_goal_ = nh_.subscribe("/move_base_simple/goal", 1,
                                   &ObstacleAnalyzer::GoalHandler, this);
  }

  void GoalHandler(geometry_msgs::PoseStamped msg) {
    std::cout << "x: " << msg.pose.orientation.x
              << " y: " << msg.pose.orientation.y
              << " z: " << msg.pose.orientation.z
              << " w: " << msg.pose.orientation.w << std::endl;
    dest_x = msg.pose.position.x;
    dest_y = msg.pose.position.y;

    double phi_cos = 2 * std::acos(msg.pose.orientation.w);
    dest_phi = msg.pose.orientation.z > 0 ? phi_cos : -phi_cos;

    std::cout << "x: " << dest_x << " y: " << dest_y << " phi: " << dest_phi
              << std::endl;
  }

  void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) {
    search_map->Reset();
    ros::Time start = ros::Time::now();
    search_map->AddObstacleArrayPtr(msg_ptr);

    search_map->SetStartPoint(0, 0, 0);
    search_map->SetEndPoint(dest_x, dest_y, dest_phi);

    if (!search_map->GenerateHeuristicMap()) {
      ROS_ERROR("Map Initialization Failure!");
      return;
    }
    std::cout << "time for map generation: " << ros::Time::now() - start
              << std::endl;
    // search_map->PlotHeuristicMap();

    start = ros::Time::now();
    search_map->Search();
    std::cout << "time for search: " << ros::Time::now() - start << "\n"
              << std::endl
              << "-------------------------------------------------------------"
                 "--------"
              << std::endl;

    search_map->PlotDebugMap();
    search_map->PlotTrajectory();
  }

 private:
  double dest_x = 0, dest_y = 0, dest_phi = 0;

  std::shared_ptr<HybridAStarSearchMap> search_map;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_obstacle_, sub_rviz_goal_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;

  ObstacleAnalyzer obstacle_analyzer;
  obstacle_analyzer.Init();

  ros::spin();
  return 0;
}