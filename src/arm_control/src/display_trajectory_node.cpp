#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_trajectory_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string ARM_PLANNING_GROUP = "kuka_arm";
    moveit::planning_interface::MoveGroupInterface arm_group_interface(ARM_PLANNING_GROUP);

    static const std::string TABLE_PLANNING_GROUP = "table_arm";
    moveit::planning_interface::MoveGroupInterface table_group_interface(TABLE_PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("tableLink2"); //tableLink2 tableEeffortLink
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ros::Rate rate(10); // 1 Hz
    while (ros::ok())
    {
        geometry_msgs::PoseStamped link7_pose = arm_group_interface.getCurrentPose("link7");

        geometry_msgs::PoseStamped table_pose = arm_group_interface.getCurrentPose("tableLink2");


        // 绘制link7和的位置点
        visual_tools.publishSphere(link7_pose.pose.position, rvt::colors::RED, rvt::scales::LARGE);
        visual_tools.trigger();

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}