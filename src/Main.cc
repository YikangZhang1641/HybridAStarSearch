#include "planner/HybridAStarSearchMap.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0);

  HybridAStarSearchMap search_map;
  search_map.setBounds(0, 10, 0, 10);
  search_map.addObstacles(3, 3.1, 0, 6);
  search_map.addObstacles(6, 6.1, 4, 10);
  search_map.setStartPoint(0.5, 0.5, 0);
  search_map.setEndPoint(9.0, 9.5, 0);

  search_map.GenerateHeuristicMap();

  ros::Time begin = ros::Time::now();
  search_map.Search();
  ros::Time end = ros::Time::now();
  std::cout << "time used for search: " << end - begin << std::endl;

  while (ros::ok()) {
    search_map.plot();
    loop_rate.sleep();
  }
  return 0;
}