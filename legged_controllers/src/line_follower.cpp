#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>



ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel_msg;
bool forward = true;
int round_count = 0;
int sample_rate = 50;
double power_summation = 0; 
double v = 0.6;
int single_count = 0;

void timerCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("time callbacking...");
    if (forward)
    {
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = 0.0;
    }
    else
    {
        cmd_vel_msg.linear.x = -v;
        cmd_vel_msg.angular.z = 0.0;
    }

    cmd_vel_pub.publish(cmd_vel_msg);

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    if (single_count < 400)
        single_count++;
    else
    {
        single_count = 0;
        forward = !forward;
    }

    if (forward && single_count == 0)
    {
        round_count++;
        // ROS_INFO("%d", round_count);
        ROS_INFO("Current position: (%f, %f, %f).", x, y, z);
    }

    if (round_count == 2)
    {
        ROS_INFO("Total power consumption: %f W", power_summation);   
        ros::shutdown();
    }
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_follower");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("/odom", 1, timerCallback);
    ros::Subscriber sub2 = nh.subscribe("/joint_states", 1000, JointStateCallback);
    ros::Rate rate(sample_rate);
    

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
