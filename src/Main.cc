#include "planner/HybridAStarSearchMap.h"
#include <geometry_msgs/Point32.h>

class ObstacleAnalyzer {
 public:
  ObstacleAnalyzer() = default;
  void Init() {
    search_map = new HybridAStarSearchMap();
    search_map->setBounds(-10, 10, -10, 10);
    if (!search_map->setStartPoint(0, 0, 0)) {
      std::cout << "start point not available!" << std::endl;
      return;
    }
    if (!search_map->setEndPoint(0, 0, 0)) {
      std::cout << "end point not available!" << std::endl;
      return;
    }

    pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("vis_polygon", 10);
    sub_ = nh_.subscribe("/obstacles", 1, &ObstacleAnalyzer::ObstacleHandler,
                         this);
  }

  void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) {
    search_map->clearObstacles();
    search_map->clearMap();
    std::cout << "read obstacle array" << std::endl;
    for (costmap_converter::ObstacleMsg ob : msg_ptr->obstacles) {
      // geometry_msgs::PolygonStamped ps;
      // ps.header.stamp = ros::Time::now();
      // ps.header.frame_id = "map";
      // ps.polygon = ob.polygon;
      // pub_.publish(ps);
      
      double xmin = std::numeric_limits<double>::max();
      double ymin = std::numeric_limits<double>::max();
      double xmax = std::numeric_limits<double>::min();
      double ymax = std::numeric_limits<double>::min();

      for (geometry_msgs::Point32 p : ob.polygon.points) {
        xmin = std::min(xmin, (double)p.x / 3);
        ymin = std::min(ymin, (double)p.y / 3);
        xmax = std::max(xmax, (double)p.x / 3);
        ymax = std::max(ymax, (double)p.y / 3);
      }
      // std::cout << xmin << xmax << ymin << ymax << std::endl;
      search_map->addObstacles(xmin, xmax, ymin, ymax);
    }
    search_map->GenerateHeuristicMap();

    search_map->plotHeuristicMap();
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

  // search_map.setBounds(0, 10, 0, 10);
  // // search_map.addObstacles(3, 4, 1, 4);
  // // search_map.addObstacles(2, 4, 7, 9);
  // // search_map.addObstacles(6, 7, 5, 10);
  // search_map.addObstacles(0, 5, 5, 6);

  // if (!search_map.setStartPoint(1, 1, 0)) {
  //   std::cout << "start point not available!" << std::endl;
  //   return 0;
  // }
  // if (!search_map.setEndPoint(1, 9, 3)) {
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