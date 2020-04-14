#include <geometry_msgs/Point32.h>

#include "planner/HybridAStarSearchMap.h"

class ObstacleAnalyzer {
 public:
  ObstacleAnalyzer() = default;
  void Init() {
    search_map = new HybridAStarSearchMap();
    search_map->SetBounds(-10, 10, -10, 10);
    if (!search_map->SetStartPoint(0, 0, 0)) {
      std::cout << "start point not available!" << std::endl;
      return;
    }
    if (!search_map->SetEndPoint(9, 0, 0)) {
      std::cout << "end point not available!" << std::endl;
      return;
    }

    pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("vis_polygon", 10);
    sub_ = nh_.subscribe("/obstacles", 1, &ObstacleAnalyzer::ObstacleHandler,
                         this);
  }

  void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) {
    // search_map->ClearObstacles();
    search_map->ClearMap();

     ros::Time start = ros::Time::now();
    for (costmap_converter::ObstacleMsg ob : msg_ptr->obstacles) {
      // geometry_msgs::PolygonStamped ps;
      // ps.header.stamp = ros::Time::now();
      // ps.header.frame_id = "map";
      // ps.polygon = ob.polygon;
      // pub_.publish(ps);

      search_map->AddObstacles(ob.polygon);
    }
    if (!search_map->CheckStartEndPoints()) {
      std::cout << "start/end points invalid!" << std::endl;
      return;
    }

    search_map->GenerateHeuristicMap();
    std::cout << "time for map generation: " << ros::Time::now() - start <<
    "\n" << std::endl;
    search_map->PlotHeuristicMap();

    start = ros::Time::now();
    search_map->Search();
    std::cout << "time for search: " << ros::Time::now() - start << "\n"
              << std::endl;

    search_map->PlotTrajectory();
  }

 private:
  HybridAStarSearchMap* search_map;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0);

  // HybridAStarSearchMap search_map;

  ObstacleAnalyzer obstacle_analyzer;
  obstacle_analyzer.Init();

  // search_map.SetBounds(0, 10, 0, 10);
  // // search_map.addObstacles(3, 4, 1, 4);
  // // search_map.addObstacles(2, 4, 7, 9);
  // // search_map.addObstacles(6, 7, 5, 10);
  // search_map.addObstacles(0, 5, 5, 6);

  // if (!search_map.setStartPoint(1, 1, 0)) {
  //   std::cout << "start point not available!" << std::endl;
  //   return 0;
  // }
  // if (!search_map.SetEndPoint(1, 9, 3)) {
  //   std::cout << "end point not available!" << std::endl;
  //   return 0;
  // }

  // search_map.GenerateHeuristicMap();

  // std::cout << "start searching..." << std::endl;
  // ros::Time begin = ros::Time::now();
  // search_map.Search();
  // ros::Time end = ros::Time::now();
  // std::cout << "time used for search: " << end - begin << std::endl;

  ros::spin();

  // while (ros::ok()) {
  //   search_map.plotHeuristicMap();
  //   loop_rate.sleep();
  // }
  return 0;
}