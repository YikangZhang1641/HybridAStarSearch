#include "planner/HybridAStarSearchMap.h"

double dest_x = 0, dest_y = 0, dest_phi = 0;

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("rs_path", 10);
  ros::Rate r(1);

  ros::Subscriber sub_rviz_goal_ =
      nh.subscribe("move_base_simple/goal", 10, GoalHandler);

  double max_kappa = 27 * M_PI / 180;
  double step_size = 0.02;

  std::vector<double> bounds;
  bounds.emplace_back(-10);
  bounds.emplace_back(10);
  bounds.emplace_back(-10);
  bounds.emplace_back(10);

  double xygrid = 0.3, phigrid = 0.2;

  while (ros::ok()) {
    std::shared_ptr<Node3d> start_node =
        std::make_shared<Node3d>(0, 0, 0, xygrid, phigrid, bounds);
    std::shared_ptr<Node3d> end_node = std::make_shared<Node3d>(
        dest_x, dest_y, dest_phi, xygrid, phigrid, bounds);

    std::shared_ptr<ReedShepp> reed_shepp_generator_ =
        std::make_shared<ReedShepp>(max_kappa, step_size);

    ReedSheppPath optimal_path;

    ros::Time start = ros::Time::now();

    if (reed_shepp_generator_->ShortestRSP(start_node, end_node,
                                           optimal_path) == false) {
      std::cout << "RS path generation failed!" << std::endl;
    }
    std::cout << "time: " << ros::Time::now() - start << std::endl;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "";
    line_strip.id = 1;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;

    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration(0.1);

    for (int i = 0; i < optimal_path.x.size(); i++) {
      geometry_msgs::Point p;
      p.x = optimal_path.x[i];
      p.y = optimal_path.y[i];
      p.z = 0;
      // std::cout << "x: " << p.x << " y: " << p.y << std::endl;
      line_strip.points.push_back(p);
    }
    marker_pub.publish(line_strip);

    ros::spinOnce();
  }
  return 0;
}