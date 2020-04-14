#include "planner/HybridAStarSearchMap.h"
using namespace udrive::planning;

class ObstacleAnalyzer {
 public:
  ObstacleAnalyzer() = default;
  void Init() {
    search_map = std::make_shared<HybridAStarSearchMap>();
    search_map->SetBounds(-5, 5, -5, 5);

    sub_ = nh_.subscribe("/obstacles", 1, &ObstacleAnalyzer::ObstacleHandler,
                         this);
  }

  void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) {
    search_map->ClearMap();

    ros::Time start = ros::Time::now();
    for (costmap_converter::ObstacleMsg ob : msg_ptr->obstacles) {
      search_map->AddObstacles(ob.polygon);
    }

    search_map->SetStartPoint(0, 0, 0);
    search_map->SetEndPoint(2, 4, 0) ;
    if (!search_map->CheckStartEndPoints()) {
      ROS_ERROR("start/end points invalid!");
      return;
    }

    search_map->GenerateHeuristicMap();
    std::cout << "time for map generation: " << ros::Time::now() - start << "\n"
              << std::endl;
    search_map->PlotHeuristicMap();

    start = ros::Time::now();
    search_map->Search();
    std::cout << "time for search: " << ros::Time::now() - start << "\n"
              << std::endl;

    search_map->PlotTrajectory();
  }

 private:
  std::shared_ptr<HybridAStarSearchMap> search_map;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;

  

  ObstacleAnalyzer obstacle_analyzer;
  obstacle_analyzer.Init();

  ros::spin();
  return 0;
}