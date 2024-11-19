#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;

struct Point
{
    double x, y;
};

// 生成希尔伯特曲线的递归函数
void hilbert(int n, std::vector<Point> &points, double x0, double y0)
{
    if (n <= 0)
    {
        points.push_back(Point{x0, y0}); // 添加坐标点
    }
    else
    {
        // 存储上一层的坐标
        std::vector<Point> prev_points;
        hilbert(n - 1, prev_points, 0.0, 0.0); // 递归获取前一层的点

        // 进行坐标转换
        for (const auto &p : prev_points)
        {
            points.push_back(Point{0.5 * (-0.5 + p.y), 0.5 * (-0.5 + p.x)});
        }
        for (const auto &p : prev_points)
        {
            points.push_back(Point{0.5 * (-0.5 + p.x), 0.5 * (0.5 + p.y)});
        }
        for (const auto &p : prev_points)
        {
            points.push_back(Point{0.5 * (0.5 + p.x), 0.5 * (0.5 + p.y)});
        }
        for (const auto &p : prev_points)
        {
            points.push_back(Point{0.5 * (0.5 - p.y), 0.5 * (-0.5 - p.x)});
        }
    }
}

bool computeAndExecuteCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                                    const geometry_msgs::Pose& target_pose,
                                    const geometry_msgs::Point& sphere_center,
                                    double eef_step,
                                    int max_attempts)
{
    // 计算从球心到目标点的向量
    Eigen::Vector3d target_point_vector(target_pose.position.x - sphere_center.x,
                                        target_pose.position.y - sphere_center.y,
                                        target_pose.position.z - sphere_center.z);

    // 计算单位向量
    Eigen::Vector3d target_unit_vector = target_point_vector.normalized();

    // y轴正方向
    Eigen::Vector3d y_axis(0, -1, 0);

    // 计算旋转轴
    Eigen::Vector3d rotation_axis = y_axis.cross(target_unit_vector);
    rotation_axis.normalize();

    // 计算旋转角度
    double rotation_angle = acos(y_axis.dot(target_unit_vector));

    // 计算旋转四元数
    Eigen::AngleAxisd angle_axis(rotation_angle, rotation_axis);
    Eigen::Quaterniond quaternion(angle_axis);

    Eigen::Vector3d euler_angles = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);

    // 打印目标位置和四元数
    ROS_INFO("Target Position - x: %.6f, y: %.6f, z: %.6f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    ROS_INFO("Target Orientation (Euler angles) - roll: %.6f, pitch: %.6f, yaw: %.6f", euler_angles[2], euler_angles[1], euler_angles[0]);

    geometry_msgs::Pose adjusted_pose = target_pose;
    adjusted_pose.orientation.x = std::round(quaternion.x() * 1e8) / 1e8;
    adjusted_pose.orientation.y = std::round(quaternion.y() * 1e8) / 1e8;
    adjusted_pose.orientation.z = std::round(quaternion.z() * 1e8) / 1e8;
    adjusted_pose.orientation.w = std::round(quaternion.w() * 1e8) / 1e8;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(adjusted_pose);

    // 逐个点规划并执行
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0.0;
    int attempts = 0;

    while (fraction < 0.99 && attempts < max_attempts)
    {
        fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
        attempts++;
    }

    if (fraction > 0.10) // 如果规划成功率高于10%
    {
        ROS_INFO("Successfully computed Cartesian path (%.2f%% achieved) after %d attempts", fraction * 100.0, attempts);
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group_interface.execute(cartesian_plan);
        return true;
    }
    else
    {
        ROS_WARN("Failed to compute Cartesian path for this waypoint (%.2f%% achieved) after %d attempts", fraction * 100.0, attempts);
        return false;
    }
}


// 生成完整的摆线点
std::vector<Point> generateCycloid(const std::vector<Point>& hilbertPoints, double radius, int frequency) {
    std::vector<Point> cycloidPoints;

    for (size_t i = 0; i < hilbertPoints.size() - 1; ++i) {
        Point p1 = hilbertPoints[i];
        Point p2 = hilbertPoints[i + 1];

        // 计算向量 v 和单位法线 n
        double vx = p2.x - p1.x;
        double vy = p2.y - p1.y;
        double length = std::sqrt(vx * vx + vy * vy);
        double nx = -vy / length;
        double ny = vx / length;

        // 在两点之间生成摆线点
        for (int j = 0; j <= frequency; ++j) {
            double t = static_cast<double>(j) / frequency; // 插值系数
            double px = p1.x + t * vx;                    // 当前点的x
            double py = p1.y + t * vy;                    // 当前点的y
            double offset = radius * std::sin(M_PI * 2 * t); // 摆线偏移量

            cycloidPoints.push_back(Point{px + offset * nx, py + offset * ny});
        }
    }

    return cycloidPoints;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "kuka_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link6");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' to move to Point A");

    // // 设置关节空间目标
    // std::vector<double> joint_group_positions;
    // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // // 设置A点的关节位置
    // joint_group_positions[0] = -M_PI / 2; // 示例值，请根据实际情况调整
    // joint_group_positions[1] = M_PI / 6;  // 示例值，请根据实际情况调整
    // // ...设置其他关节位置...

    // move_group_interface.setJointValueTarget(joint_group_positions);

    // // 规划并移动到A点
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group_interface.move();
    // }
    // else
    // {
    //     ROS_ERROR("Failed to plan to A point");
    // }

    // // 可视化轨迹
    // visual_tools.publishText(text_pose, "Moving to Point A", rvt::WHITE, rvt::XLARGE);
    // //   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' to move to Point B");

    // // 设置B点的关节位置
    // joint_group_positions[0] = M_PI / 2; // 示例值，请根据实际情况调整
    // joint_group_positions[1] = M_PI / 6; // 示例值，请根据实际情况调整
    // // ...设置其他关节位置...

    // move_group_interface.setJointValueTarget(joint_group_positions);

    // // 规划并移动到B点
    // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group_interface.move();
    // }
    // else
    // {
    //     ROS_INFO("Failed to move to Point B");
    // }

    // // 可视化轨迹
    // visual_tools.publishText(text_pose, "Moving to Point B", rvt::WHITE, rvt::XLARGE);
    // //   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

    int order = 4; // 希尔伯特曲线的阶数
    std::vector<Point> hilbert_raw_points;
    hilbert(order, hilbert_raw_points, 0, 0);

    // 将生成的点缩放到范围内
    // double w = 1.2;
    // double h = 1.2;
    double w = 1;
    double h = 1;
    // 设置笛卡尔路径的目标位姿
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;

    std::vector<Point> cycloid_points = generateCycloid(hilbert_raw_points, 0.01, 15);//生成摆线轨迹
    for (const auto &point : cycloid_points)
    {
        target_pose.position.x = 0.8; // 固定高度，确保末端垂直于运动平面
        target_pose.position.y = (point.x) * w;
        target_pose.position.z = (point.y + 0.5) * h + 0.1;
        waypoints.push_back(target_pose);
    }

    // 规划笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
    
    ROS_INFO("Successfully computed Cartesian path (%.2f%% achieved)", fraction * 100.0);
    visual_tools.publishText(text_pose, "Executing Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    
    // 创建 DisplayTrajectory 对象
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.push_back(trajectory);
    display_trajectory.model_id = move_group_interface.getRobotModel()->getName();
    
    // 发布轨迹
    visual_tools.publishTrajectoryPath(display_trajectory);
    
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);

    /*****************************************************/
    // 附加任务一：现有直径为0.5m的半球体，在笛卡尔空间下，机械臂末端在空
    // 间实现半球面的 4 阶希尔伯特曲线运动（即将平面的希尔伯特曲线映射到半球
    // 面），且机械臂末端始终垂直于当前运动点在半球上的切平面），并显示轨迹；

    // // 将生成的点映射到半球面上
    // double radius = 0.25; // 半径为0.25米，直径为0.5米
    // geometry_msgs::Point sphere_center;
    // sphere_center.x = 0.65; // 设置球心的x坐标
    // sphere_center.y = 0.65; // 设置球心的y坐标
    // sphere_center.z = 0.65; // 设置球心的z坐标

    // // 设置笛卡尔路径的目标位姿
    // // geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;

    // for (const auto &point : hilbert_raw_points)
    // {
    //     double theta = point.x * M_PI; // 将x映射到[0, π]
    //     double phi = point.y * M_PI;   // 将y映射到[0, π]
    //     target_pose.position.x = sphere_center.x + radius * sin(phi) * cos(theta);
    //     target_pose.position.y = sphere_center.y + radius * sin(phi) * sin(theta);
    //     target_pose.position.z = sphere_center.z + radius * cos(phi);

    //     computeAndExecuteCartesianPath(move_group_interface, target_pose, sphere_center, 0.05, 30);
    // }

    // //附加任务二：基于附加任务一，将希尔伯特曲线改为希尔伯特曲线作为引导线
    // //           的摆线轨迹，重新实现附加任务一，并显示轨迹；
    // for (const auto &point : hilbert_raw_points)
    // {
    //     double theta = point.x * M_PI; // 将x映射到[0, π]
    //     double phi = point.y * M_PI;   // 将y映射到[0, π]
    //     target_pose.position.x = sphere_center.x + radius * sin(phi) * cos(theta);
    //     target_pose.position.y = sphere_center.y + radius * sin(phi) * sin(theta);
    //     target_pose.position.z = sphere_center.z + radius * cos(phi);

    //     computeAndExecuteCartesianPath(move_group_interface, target_pose, sphere_center, 0.05, 30);
    // }

    ros::shutdown();
    return 0;
}