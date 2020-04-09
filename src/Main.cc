#include "planner/HybridAStarSearchMap.h"

void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) {
  std::cout << "read obstacle array" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0);

  ros::Subscriber obstacle_sub = nh.subscribe(
      "/obstacles", 1, ObstacleHandler);

  HybridAStarSearchMap search_map;
  search_map.setBounds(0, 10, 0, 10);
  // search_map.addObstacles(3, 4, 1, 4);
  // search_map.addObstacles(2, 4, 7, 9);
  // search_map.addObstacles(6, 7, 5, 10);
  search_map.addObstacles(0, 5, 5, 6);

  if (!search_map.setStartPoint(1, 1, 0)) {
    std::cout << "start point not available!" << std::endl;
    return 0;
  }
  if (!search_map.setEndPoint(1, 9, 3)) {
    std::cout << "end point not available!" << std::endl;
    return 0;
  }

  search_map.GenerateHeuristicMap();

  std::cout << "start searching..." << std::endl;
  ros::Time begin = ros::Time::now();
  search_map.Search();
  ros::Time end = ros::Time::now();
  std::cout << "time used for search: " << end - begin << std::endl;

  ros::spin();

  // while (ros::ok()) {
  //   search_map.plot();
  //   loop_rate.sleep();
  // }
  return 0;
}