#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <ostream>

ros::Publisher cmd_vel_pub;
const double EPSILON = 1e-2;
int flag = 1;
int laps = 0; 

void yawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double r = 4;
    double v = 0.4;
    double omega = v / r;
    double roll, pitch, yaw;

    tf::Quaternion quat(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Twist twist_msg;
    
    if (abs(yaw) < EPSILON && flag == 0)
    {
        laps ++;
        flag = 1;
    } 

    if (yaw > 3)
        flag = 0;


    if (laps % 2)
    {
        twist_msg.linear.x = v;
        twist_msg.angular.z = omega;

    }else
    {
        twist_msg.linear.x = v;
        twist_msg.angular.z = -omega;
    }
    
    cmd_vel_pub.publish(twist_msg);
    ROS_INFO("Current yaw: %f and Current lap: %d", yaw, laps);
}

int main(int argc, char **argv)
{
    double pi = M_PI;   
    ros::init(argc, argv, "eight_figure");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1, yawCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();

    return 0;
}
