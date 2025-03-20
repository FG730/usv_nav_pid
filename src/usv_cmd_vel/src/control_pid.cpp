#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <stdio.h>
#include <usv_cmd_vel/cora_pid.hpp>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Twist des_vel;
nav_msgs::Odometry actu_vel;
std_msgs::Float32 left__thrust;
std_msgs::Float32 right_thrust;
std_msgs::Float32 left_angle;
std_msgs::Float32 right_angle;
std_msgs::Float32 lateral__thrust;
std_msgs::Float32 turn_thrust_msg;
std_msgs::Float32 angular_error_msg;

geometry_msgs::PoseStamped goal_pose; // 全局目标位置

// PID参数
double kp_position=0.0, kd_position=0, ki_position=0; // 位置环PID参数
double kp_velocity=0.0, kd_velocity=0, ki_velocity=0; // 速度环PID参数

double kp_angular = 1.3; // 角速度比例增益
double kd_angular = 0.0; //角速度微分
double ki_angular = 0.0; // 角速度积分增益
//回调函数，一旦接收到新数据就将其更新到des_vel
void desire_back(const geometry_msgs::Twist &desire_vel)
{
des_vel = desire_vel;
}
//回调函数，一旦接收到新数据就将其更新到actu_vel
void actual_back(const nav_msgs::Odometry &actual_vel)
{
actu_vel = actual_vel;
}
// 回调函数，一旦接收到新数据就将其更新到goal_pose
void goal_back(const geometry_msgs::PoseStamped &goal)
{
goal_pose = goal;
}

int main(int argc, char* argv[])
{
setlocale(LC_ALL,"");


ros::init(argc,argv,"control_pid",ros::init_options::AnonymousName);
ros::NodeHandle nh;
// 订阅全局目标位置话题
ros::Subscriber sub_goal = nh.subscribe("/move_base_simple/goal", 10, goal_back);
//订阅速度控制话题，使用movebase发布的速度信息控制螺旋桨转速
ros::Subscriber sub_desire = nh.subscribe("/cmd_vel",10,desire_back);
//odom话题由GPS和IMU融合定位提供
ros::Subscriber sub_actual = nh.subscribe("/cora/robot_localization/odometry/filtered",10,actual_back);
//发布消息到左螺旋桨转速控制话题
ros::Publisher pub_left_thrust = nh.advertise<std_msgs::Float32>("/cora/thrusters/left_thrust_cmd",10);
//发布消息到右螺旋桨转速控制话题
ros::Publisher pub_right_thrust = nh.advertise<std_msgs::Float32>("/cora/thrusters/right_thrust_cmd",10);
ros::Publisher pub_turn_thrust = nh.advertise<std_msgs::Float32>("/turn_thrust", 10);
ros::Publisher pub_angular_error = nh.advertise<std_msgs::Float32>("/angular_error", 10);
ros::Rate r(50);
control::pid_wamv pid;
control::pid_wamv pid_angular;
control::pid_wamv pid_position;
while (ros::ok())
{
double pos_error_x = goal_pose.pose.position.x - actu_vel.pose.pose.position.x;
double pos_error_y = goal_pose.pose.position.x - actu_vel.pose.pose.position.y;
double linear_distance_error = std::sqrt(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
if(pos_error_x<0) linear_distance_error = - linear_distance_error;
double target_velocity = pid_position.pid_control(linear_distance_error, 0, kp_position, kd_position, ki_position);

// liner.x x即是前进的方向
double ut_vx = pid.pid_control(target_velocity,actu_vel.twist.twist.linear.x,kp_velocity, kd_velocity, ki_velocity);
// ut_vx = fabs(ut_vx);
if(ut_vx>=1) ut_vx =1;
if(ut_vx<=-1) ut_vx = -1;
left__thrust.data+=ut_vx;
right_thrust.data+=ut_vx;


double angular_error = des_vel.angular.z - actu_vel.twist.twist.angular.z;



double turn_thrust = pid_angular.pid_control(angular_error, 0, kp_angular, kd_angular, ki_angular);
// **转向优先策略**
double angular_velocity_threshold = 0.3;
if (fabs(angular_error) > angular_velocity_threshold ) {
ut_vx=0;
}
// 分配左右推进器推力
left__thrust.data = ut_vx - turn_thrust;
right_thrust.data = ut_vx + turn_thrust;

pub_left_thrust.publish(left__thrust);
pub_right_thrust.publish(right_thrust);
turn_thrust_msg.data = turn_thrust;
angular_error_msg.data = angular_error;
pub_turn_thrust.publish(turn_thrust_msg);
pub_angular_error.publish(angular_error_msg);
r.sleep();
ros::spinOnce();

}


return 0;
}