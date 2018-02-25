#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "wheel.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "mybot_driver");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/mybot/published_odom_state", 50);
  // ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, subscribe_odom);
  // ros::Subscriber sub_rqt=n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, subscribe_cmd_vel);
  //ros::Subscriber sub_odo_from_cmd_vel=n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, subscribe_odom);
  ros::Subscriber sub_odo_from_cmd_vel=n.subscribe<geometry_msgs::Twist>("/mybot/published_wheel_dist", 1, subscribe_published_wheel_dist);
  tf::TransformBroadcaster odom_broadcaster;
  while(n.ok()){
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    odom_trans = set_trans(odom_trans, "odom", "base", x, y, 0, odom_quat);
    odom_broadcaster.sendTransform(odom_trans);
    odom_trans = set_trans(odom_trans, "odom", "laser", x, y, 0, odom_quat);
    odom_broadcaster.sendTransform(odom_trans);
    odom_trans = set_trans(odom_trans, "odom", "left_front_wheel", x, y, 0, odom_quat);
    odom_broadcaster.sendTransform(odom_trans);
    odom_trans = set_trans(odom_trans, "odom", "right_front_wheel", x, y, 0, odom_quat);
    odom_broadcaster.sendTransform(odom_trans);
    odom_trans = set_trans(odom_trans, "odom", "left_back_wheel", x, y, 0, odom_quat);
    odom_broadcaster.sendTransform(odom_trans);
    odom_trans = set_trans(odom_trans, "odom", "right_back_wheel", x, y, 0, odom_quat );
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom = set_odo(odom,"odom","base", x, y, 0, odom_quat, delta_x, delta_y, delta_th, dt);
    // publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }
}
