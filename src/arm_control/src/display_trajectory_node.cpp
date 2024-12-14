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

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("tableEeffortLink"); //tableLink2 tableEeffortLink
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ros::Rate rate(10); // 1 Hz
    while (ros::ok())
    {
        geometry_msgs::PoseStamped link7_pose = arm_group_interface.getCurrentPose("link6");
        // 绘制link7和的位置点
        link7_pose.pose.position.x = link7_pose.pose.position.x - 1;
        link7_pose.pose.position.y = link7_pose.pose.position.y;
        link7_pose.pose.position.z = link7_pose.pose.position.z - 0.151 ;//- 0.226

        // double x = link7_pose.pose.position.x;
        // double y = link7_pose.pose.position.y;
        // double z = link7_pose.pose.position.z;
        // link7_pose.pose.position.y = z;
        // link7_pose.pose.position.z = -y;

        visual_tools.publishSphere(link7_pose.pose.position, rvt::colors::RED, rvt::scales::LARGE);
        visual_tools.trigger();

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}