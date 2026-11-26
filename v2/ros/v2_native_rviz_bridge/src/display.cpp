#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
namespace v2_native_rviz_bridge {
class Display : public moveit_rviz_plugin::MotionPlanningDisplay {
  ros::NodeHandle nh_; ros::Publisher start_, goal_; double elapsed_=0;
public:
 Display() { start_=nh_.advertise<moveit_msgs::RobotState>("/v2/native_motionplanning/start",1,true); goal_=nh_.advertise<moveit_msgs::RobotState>("/v2/native_motionplanning/goal",1,true); }
 void update(float wall_dt,float ros_dt) override {
  MotionPlanningDisplay::update(wall_dt,ros_dt); elapsed_+=wall_dt; if(elapsed_<0.2)return; elapsed_=0;
  if(!getQueryStartStateHandler() || !getQueryGoalStateHandler())return;
  auto s=getQueryStartState(), g=getQueryGoalState(); if(!s || !g)return;
  moveit_msgs::RobotState a,b; moveit::core::robotStateToRobotStateMsg(*s,a); moveit::core::robotStateToRobotStateMsg(*g,b); start_.publish(a);goal_.publish(b);
 }
}; }
PLUGINLIB_EXPORT_CLASS(v2_native_rviz_bridge::Display,rviz::Display)
