#ifndef __wheel_h__
#define __wheel_h__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;
double th, ticks, angle;
const double degree = M_PI/180;
float lwheel, rwheel;

double x; 
double y;
// double th;
double v_left;//left motor speed
double v_right;//right motor speed
double vth;//angular velocity of robot
double deltaLeft;//no of ticks in left encoder since last update
double deltaRight;//no of ticks in right encoder since last update
double dt;
double delta_distance;//distance moved by robot since last update
double delta_th;//corresponging change in heading
double delta_x ;//corresponding change in x direction
double delta_y;//corresponding change in y direction
#define PI 3.14159265
#define TwoPI 6.28318531

ros::Time current_time, last_time;
double _PreviousLeftEncoderCounts = 0;
double _PreviousRightEncoderCounts = 0;
double DistancePerCount = (3.14159265 * 0.08)/650; //the wheel diameter is 0.080mm
geometry_msgs::Twist act_pos;

/* void subscribe_cmd_vel(const geometry_msgs::Twist::ConstPtr &speed) { */
/* } */
void subscribe_odom_from_cmd_vel(const geometry_msgs::Twist::ConstPtr &speed) {
  act_pos.linear.x += speed->linear.x;
  act_pos.linear.y += speed->linear.y;
}
// void subscribe_odom(const geometry_msgs::Vector3 msg){
void subscribe_odom(const geometry_msgs::Twist::ConstPtr &speed){
  current_time = ros::Time::now();
  deltaLeft  = speed->linear.x - _PreviousLeftEncoderCounts;
  deltaRight = speed->linear.y - _PreviousRightEncoderCounts;
  dt = (current_time - last_time).toSec();
  v_left = (deltaLeft * DistancePerCount)/dt;
  v_right = (deltaRight * DistancePerCount)/dt;
  delta_distance = 0.5f * (double) (deltaLeft+deltaRight) * DistancePerCount;
  delta_th = (double)(deltaRight-deltaLeft)*DistancePerCount/0.36f; //Distance between the two wheels is 0.36m
  delta_x = delta_distance*(double)cos(th);
  delta_y = delta_distance*(double)sin(th);
  x += delta_x;
  y += delta_y;
  th += delta_th;
  if (th > PI)
    th -= TwoPI;
  else
    if ( th <= -PI)
      th += TwoPI;
  _PreviousLeftEncoderCounts = speed->linear.x;
  _PreviousRightEncoderCounts = speed->linear.y;
  last_time = current_time;

  cout<<"============================================================="<<endl;
  cout<< "l_wheel:["<< x <<"],r_wheel:["<< y <<"]"<<endl;
  cout<<"dt:["<< dt <<"]"<<endl;
  cout<<"v left:["<< v_left <<"]"<<endl;
  cout<<"v right:["<< v_right <<"]"<<endl;
  cout<<"delta distance:["<< delta_distance <<"]"<<endl;
  cout <<"============================================================="<<endl;
}

geometry_msgs::TransformStamped set_trans(geometry_msgs::TransformStamped odo, string header, string child, double _x, double _y, double _z, geometry_msgs::Quaternion _rot){
  odo.header.stamp = current_time;
  odo.header.frame_id =header;
  odo.child_frame_id = child;
  odo.transform.translation.x = _x;
  odo.transform.translation.y = _y;
  odo.transform.translation.z = _z;
  odo.transform.rotation = _rot;  
  return odo;
}
nav_msgs::Odometry set_odo(nav_msgs::Odometry odom, string header, string child, double _x, double _y, double _z, geometry_msgs::Quaternion odom_quat, double dx, double dy, double dth, double _dt){
  odom.header.stamp = current_time;
  odom.header.frame_id = header;
  //set the position
  odom.pose.pose.position.x = _x;
  odom.pose.pose.position.y = _y;
  odom.pose.pose.position.z = _z;
  odom.pose.pose.orientation = odom_quat;
  //set the velocity
  odom.child_frame_id = child;
  odom.twist.twist.linear.x=dx/_dt;
  odom.twist.twist.linear.y=dy/_dt;
  odom.twist.twist.angular.z = dth/_dt;  
  return odom;
}
void subscribe_published_wheel_dist(const geometry_msgs::Twist::ConstPtr &speed){
  current_time = ros::Time::now();
  
  deltaLeft  = speed->linear.x - _PreviousLeftEncoderCounts;
  deltaRight = speed->linear.y - _PreviousRightEncoderCounts;
  dt = (current_time - last_time).toSec();
  /* cout <<"dleft["<<deltaLeft<<"],dright["<<deltaRight<<"],dt["<<dt<<"]"<<endl; */
  
  v_left = (deltaLeft)/dt;
  v_right = (deltaRight)/dt;
  delta_distance = 0.5f * (double) (deltaLeft+deltaRight);// * DistancePerCount;
  delta_th = (double)(deltaRight-deltaLeft)/0.36f; //Distance between the two wheels is 0.36m
  delta_x = delta_distance*(double)cos(th);
  delta_y = delta_distance*(double)sin(th);
  x += delta_x;
  y += delta_y;
  th += delta_th;
  if (th > PI)
    th -= TwoPI;
  else
    if ( th <= -PI)
      th += TwoPI;
  _PreviousLeftEncoderCounts = speed->linear.x;
  _PreviousRightEncoderCounts = speed->linear.y;
  last_time = current_time;

  cout<<"============================================================="<<endl;
  cout<<"dt:["<< dt <<"]"<<endl;
  cout<<"v left:["<< v_left <<"]"<<endl;
  cout<<"v right:["<< v_right <<"]"<<endl;
  /* cout<<"delta distance:["<< delta_distance <<"]"<<endl; */
  cout <<"============================================================="<<endl;
}


#endif
