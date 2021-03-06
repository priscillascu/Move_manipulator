#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include<iostream>
#include <cmath>

#define PI 3.14
using namespace std;

class RPY2qua  //定义一个RPY角转为四元数的类
{
   public:
	double roll_deg;
	double pitch_deg;
	double yaw_deg;
 
      // 成员函数声明
    double getQua_x(void);
    double getQua_y(void);
    double getQua_z(void);
    double getQua_w(void);
};
 
// 成员函数定义
double RPY2qua::getQua_x(void)
{
	return sin(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)-cos(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}

double RPY2qua::getQua_y(void)
{
	return cos(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)+sin(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}
double RPY2qua::getQua_z(void)
{
	return cos(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2)-sin(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2);
}
double RPY2qua::getQua_w(void)
{
	return cos(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)+sin(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}

//每当修改，都要重新catkin_make
int main(int argc, char **argv)
{
    RPY2qua my_RPY2qua;
    ros::init(argc, argv, "draw_cricle");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("arm");
    geometry_msgs::Pose start_pose;

    //输入绕固定坐标轴x的角度，单位°
    my_RPY2qua.roll_deg = 0;
    //输入绕固定坐标轴y的角度，单位°
    my_RPY2qua.yaw_deg = 0;
    //输入绕固定坐标轴z的角度，单位° 
    my_RPY2qua.pitch_deg = 90;
    //使用我们编写的转换类，将RPY转为四元数
    start_pose.orientation.x= my_RPY2qua.getQua_x();
    start_pose.orientation.y = my_RPY2qua.getQua_y();
    start_pose.orientation.z = my_RPY2qua.getQua_z();
    start_pose.orientation.w = my_RPY2qua.getQua_w();
    start_pose.position.x = 0;
    start_pose.position.y = 0.8;
    start_pose.position.z = 0.5;

    group.setPoseTarget(start_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);    
        
    if(success)
    {
      cout << "Nice! You got it!"<<endl;
      cout << "Executing your plan..." << endl;
      group.execute(my_plan);
      cout << "In position! Go on!" << endl;
    }
      
    else
    {
       cout << "Oh no, it's falled!!!"<<endl;
       cout << "Please try again..." << endl;
    }

    sleep(2.0);
    ros::shutdown();
}