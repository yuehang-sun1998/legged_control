#include <vector>
#include <cmath>
#include <ostream>

std::vector<std::pair<double, double> > figure_eight(double radius, int num_samples) {
    
    double delta_theta = 2 * M_PI / num_samples;

    std::vector<std::pair<double, double> > samples;

    for (int i = 0; i < num_samples; ++i) {
        double theta = i * delta_theta;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        samples.emplace_back(x - radius, y);
    }

    for (int i = 0; i < num_samples; ++i) {
        double theta = i * delta_theta;
        double x = -radius * cos(theta);
        double y = radius * sin(theta);
        samples.emplace_back(x + radius, y);
    }

    return samples;
}


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100, true);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";  // not existed
  goal_pose.header.stamp = ros::Time::now(); 
  std::vector<std::pair<double, double> > samples = figure_eight(5.0, 100);
  int i = 0;
  double dt = 0.2;

  while (ros::ok() && i < samples.size())
  {
    goal_pose.pose.position.x = samples[i].first; 
    goal_pose.pose.position.y = samples[i].second;
    goal_pose.pose.position.z = 0.0; 
    goal_pose.pose.orientation.x = 0.0; 
    goal_pose.pose.orientation.y = 0.0; 
    goal_pose.pose.orientation.z = 0.0; 
    goal_pose.pose.orientation.w = 1.0; 
    goal_pub.publish(goal_pose);
    ros::Duration(dt).sleep();
    i++; 
  }
  return 0;
}
