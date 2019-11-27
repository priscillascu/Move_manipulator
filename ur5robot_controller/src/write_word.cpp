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

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 2.75; // above head of PR2
    geometry_msgs::Vector3 scale;
    scale.x=0.3;
    scale.y=0.3;
    scale.z=0.3;

    moveit::planning_interface::MoveGroupInterface group("arm");

    const robot_state::JointModelGroup* joint_model_group =
    group.getCurrentState()->getJointModelGroup("arm");

    std::string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);

    std::string end_effector_link=group.getEndEffectorLink();
    std::vector<geometry_msgs::Pose> waypoints;

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

    waypoints.push_back(start_pose);
    geometry_msgs::Pose target_pose;
    target_pose = start_pose;

    float centerX = start_pose.position.x;
    float centerY = start_pose.position.y;
    float centerZ = start_pose.position.z;
    
    float wang[11][3] = 
    {
        {0.1, 0, 0.1},
        {-0.1, 0 , 0.1},
        
        {0.05, -0.05, 0},
        {0.05, -0.05, 0.1},
        {-0.05, -0.05, 0.1},

        {0.1, -0.1, 0},
        {0.1, -0.1, 0.1},
        {-0.1, -0.1, 0.1},

        {0, 0, 0},
        {0, 0, 0.1},
        {0, -0.1, 0.1},
    };

    float yang[23][3] = 
    {
        {0.1, 0, 0.1},
        {-0.1, 0, 0.1},
        {0, 0.1, 0},

        {0, 0.1, 0.1},
        {0, -0.2, 0.1},
        {0, 0, 0},

        {0, 0, 0.1},
        {0.1, -0.17, 0.1},
        {0, 0, 0},

        {0, 0, 0.1},
        {-0.1, -0.1, 0.1},

        {-0.1, 0.1, 0},
        {-0.1, 0.1, 0.1},
        {-0.2, 0.1, 0.1},
        {-0.1, 0, 0.1},
        {-0.25, 0, 0.1},
        {-0.25, -0.2, 0.1},
        {-0.175, 0, 0},
        {-0.175, 0, 0.1},
        {-0.1, -0.1, 0.1},
        {-0.175, -0.05, 0},
        {-0.175, -0.05, 0.1},
        {-0.1, -0.15, 0.1},
    };
    for(int i = 0; i < 23; i++)
    {
        target_pose.position.x = centerX + yang[i][0];
        target_pose.position.y = centerY + yang[i][2];
        target_pose.position.z = centerZ + yang[i][1];
        waypoints.push_back(target_pose);
    }
    
    target_pose = start_pose;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
 
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
 
	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
 
	    // 执行运动
	    group.execute(plan);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
        visual_tools.deleteAllMarkers();
       // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, scale);//rvt::XLARGE);
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
       // for (std::size_t i = 0; i < waypoints.size(); ++i)
           // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
       visual_tools.trigger();
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    sleep(2.0);
    ros::shutdown();
}