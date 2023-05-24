#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <ostream>

ros::Publisher cmd_vel_pub;
// ros::Time time_start;
const double EPSILON = 5e-2;
double pi = M_PI;   
int flag = 1;
int laps = 0;
int counter = 0; 
int sample_rate = 50;
double power_summation = 0; 
double r = 2.0;
double v = 0.6; 
double omega = v / r;
int counter_lim = int(4 * pi / omega * sample_rate) + 10;

void OdomStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    if (laps == 2 || counter > counter_lim)
    {   
        // ROS_INFO("%d, %d", counter_lim, counter);
        // ros::Duration time_duration = ros::Time::now() - time_start;    
        // ROS_INFO("Time cost: %f seconds", time_duration.toSec()); // not accurate; better to use the info from ROS_INFO time stamp
        ROS_INFO("Total power consumption: %f W", power_summation);   
        ros::shutdown();
    }
    double roll, pitch, yaw;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    

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
    // ROS_INFO("Current yaw: %f and Current lap: %d", yaw, laps);
    ROS_INFO("Current position: (%f, %f, %f).", x, y, z);
    counter++;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<double> torques = msg->effort;
    std::vector<double> angular_velocities = msg->velocity;
    double instant_power = 0;
    for (int i = 0; i < torques.size(); i++)
        instant_power += std::abs(torques[i] * angular_velocities[i]);
    
    power_summation += instant_power;
    // ROS_INFO("Instant power consumption: %f.", instant_power);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "figure_eight_and_joint_torque_reader");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("/odom", 1, OdomStateCallback);
    ros::Subscriber sub2 = nh.subscribe("/joint_states", 1000, JointStateCallback);
    ros::Rate rate(sample_rate);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    // ros::Time time_start = ros::Time ::now();
    // ros::spin();

    return 0;
}
