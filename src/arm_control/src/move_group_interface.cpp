#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

const double tau = 2 * M_PI;

struct Point
{
    double x, y;
};

void hilbert(int n, std::vector<Point> &points, double x0, double y0);
void Execute_CartesianPath_AllAtOnce(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                                     const moveit::core::JointModelGroup *arm_joint_model_group,
                                     moveit_visual_tools::MoveItVisualTools &visual_tools,
                                     const std::vector<geometry_msgs::Pose> &target_poses,
                                     const geometry_msgs::Point &sphere_center,
                                     double eef_step,
                                     int max_attempts,
                                     int per_execute_cnt);
std::vector<Point> generateCycloid(const std::vector<Point> &hilbertPoints, double radius, int frequency);
void missionOne(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                const moveit::core::JointModelGroup *arm_joint_model_group,
                moveit_visual_tools::MoveItVisualTools &visual_tools,
                const Eigen::Isometry3d &text_pose);
void missionTwo(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                const moveit::core::JointModelGroup *arm_joint_model_group,
                moveit_visual_tools::MoveItVisualTools &visual_tools,
                const Eigen::Isometry3d &text_pose);
void additonalTaskOne(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                      const moveit::core::JointModelGroup *arm_joint_model_group,
                      moveit_visual_tools::MoveItVisualTools &visual_tools,
                      const Eigen::Isometry3d &text_pose);
void additonalTaskTwo(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                      const moveit::core::JointModelGroup *arm_joint_model_group,
                      moveit_visual_tools::MoveItVisualTools &visual_tools,
                      const Eigen::Isometry3d &text_pose);
void additonalTaskThree(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                        const moveit::core::JointModelGroup *arm_joint_model_group,
                        moveit::planning_interface::MoveGroupInterface &table_group_interface,
                        const moveit::core::JointModelGroup *table_joint_model_group,
                        moveit::planning_interface::MoveGroupInterface &combine_group_interface,
                        const moveit::core::JointModelGroup *combine_joint_model_group,
                        const Eigen::Isometry3d &text_pose);

int main(int argc, char **argv)
{

    /********************************************************************************* */
    /*************************             初始化           ************************** */
    /********************************************************************************* */
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string ARM_PLANNING_GROUP = "kuka_arm";
    moveit::planning_interface::MoveGroupInterface arm_group_interface(ARM_PLANNING_GROUP);
    const moveit::core::JointModelGroup *arm_joint_model_group = arm_group_interface.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

    static const std::string TABLE_PLANNING_GROUP = "table_arm";
    moveit::planning_interface::MoveGroupInterface table_group_interface(TABLE_PLANNING_GROUP);
    const moveit::core::JointModelGroup *table_joint_model_group = table_group_interface.getCurrentState()->getJointModelGroup(TABLE_PLANNING_GROUP);

    static const std::string COMBINE_PLANNING_GROUP = "combine_group";
    moveit::planning_interface::MoveGroupInterface combine_group_interface(COMBINE_PLANNING_GROUP);
    const moveit::core::JointModelGroup *combine_joint_model_group = combine_group_interface.getCurrentState()->getJointModelGroup(COMBINE_PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    // 文字显示
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    text_pose.translation().y() = -1.0;
    text_pose.translation().x() = -1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to move to Point A");

    /********************************************************************************* */
    /*************************             任务一           ************************** */
    /********************************************************************************* */
    missionOne(arm_group_interface, arm_joint_model_group, visual_tools, text_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    /********************************************************************************* */
    /*************************             任务二           ************************** */
    /********************************************************************************* */
    missionTwo(arm_group_interface, arm_joint_model_group, visual_tools, text_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    /********************************************************************************* */
    /***********************             附加任务一           ************************* */
    /********************************************************************************* */
    additonalTaskOne(arm_group_interface, arm_joint_model_group, visual_tools, text_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    /********************************************************************************* */
    /***********************             附加任务二           ************************* */
    /********************************************************************************* */
    additonalTaskTwo(arm_group_interface, arm_joint_model_group, visual_tools, text_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    /********************************************************************************* */
    /***********************             附加任务三           ************************* */
    /********************************************************************************* */
    additonalTaskThree(arm_group_interface, arm_joint_model_group, table_group_interface, 
                       table_joint_model_group, combine_group_interface, 
                       combine_joint_model_group, text_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    ros::shutdown();
    return 0;
}

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

void Execute_CartesianPath_AllAtOnce(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                                     const moveit::core::JointModelGroup *arm_joint_model_group,
                                     moveit_visual_tools::MoveItVisualTools &visual_tools,
                                     const std::vector<geometry_msgs::Pose> &target_poses,
                                     const geometry_msgs::Point &sphere_center,
                                     double eef_step,
                                     int max_attempts,
                                     int per_execute_cnt = 7)
{
    std::vector<geometry_msgs::Pose> waypoints;
    for (const auto &target_pose : target_poses)
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
        adjusted_pose.orientation.x = quaternion.x();
        adjusted_pose.orientation.y = quaternion.y();
        adjusted_pose.orientation.z = quaternion.z();
        adjusted_pose.orientation.w = quaternion.w();

        waypoints.push_back(adjusted_pose);
    }

    std::vector<geometry_msgs::Pose> temp_points;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0.0;
    int attempts = 0;
    int temp_point_cnt;
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto &waypoint = waypoints[i];
        temp_points.push_back(waypoint);
        temp_point_cnt++;
        if (temp_point_cnt >= per_execute_cnt || i == waypoints.size() - 1) // 满数量就规划一次
        {
            ros::Time start_time = ros::Time::now(); // 记录开始时间
            while (fraction < 0.99 && attempts < max_attempts)
            {
                fraction = arm_group_interface.computeCartesianPath(temp_points, eef_step, trajectory);
                attempts++;
            }

            if (fraction > 0.90) // 如果规划成功率高于90%
            {
                ROS_INFO("Successfully computed Cartesian path (%.2f%% achieved) after %d attempts", fraction * 100.0, attempts);
                ros::Duration solve_time = ros::Time::now() - start_time; // 计算求解时间
                ROS_INFO("%d point planning time: %.2f ms", temp_point_cnt, solve_time.toSec() * 1000.0); // 打印求解时间
                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                cartesian_plan.trajectory_ = trajectory;
                visual_tools.publishTrajectoryLine(trajectory, arm_joint_model_group);
                visual_tools.trigger();
                arm_group_interface.execute(cartesian_plan);
            }
            else
            {
                ROS_WARN("Failed to compute Cartesian path for these waypoints (%.2f%% achieved) after %d attempts", fraction * 100.0, attempts);
            }
            fraction = 0;
            attempts = 0;
            temp_point_cnt = 0;
            temp_points.clear();
        }
    }
}

// 生成完整的摆线点
std::vector<Point> generateCycloid(const std::vector<Point> &hilbertPoints, double radius, int frequency)
{
    std::vector<Point> cycloidPoints;

    for (size_t i = 0; i < hilbertPoints.size() - 1; ++i)
    {
        Point p1 = hilbertPoints[i];
        Point p2 = hilbertPoints[i + 1];

        // 计算向量 v 和单位法线 n
        double vx = p2.x - p1.x;
        double vy = p2.y - p1.y;
        double length = std::sqrt(vx * vx + vy * vy);
        double nx = -vy / length;
        double ny = vx / length;

        // 在两点之间生成摆线点
        for (double j = 0; j <= frequency; ++j)
        {
            double t = j / frequency;                        // 插值系数
            double px = p1.x + t * vx;                       // 当前点的x
            double py = p1.y + t * vy;                       // 当前点的y
            double offset = radius * std::sin(M_PI * 2 * t); // 摆线偏移量

            cycloidPoints.push_back(Point{px + offset * nx, py + offset * ny});
        }
    }

    return cycloidPoints;
}

void missionOne(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                const moveit::core::JointModelGroup *arm_joint_model_group,
                moveit_visual_tools::MoveItVisualTools &visual_tools,
                const Eigen::Isometry3d &text_pose)
{
    namespace rvt = rviz_visual_tools;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = arm_group_interface.getCurrentState();
    current_state->copyJointGroupPositions(arm_joint_model_group, joint_group_positions);
    // 设置A点的关节位置
    joint_group_positions[0] = -M_PI / 2;
    joint_group_positions[1] = M_PI / 9;
    arm_group_interface.setJointValueTarget(joint_group_positions);

    // 规划并移动到A点
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        // 可视化轨迹
        visual_tools.publishText(text_pose, "Moving to Point A", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_joint_model_group);
        visual_tools.trigger();
        arm_group_interface.move();
    }
    else
    {
        ROS_ERROR("Failed to plan to A point");
    }

    visual_tools.prompt("Press 'next' to move to Home");

    // 设置home姿态为目标位置
    arm_group_interface.setNamedTarget("home");

    // 规划并移动到home姿态
    success = (arm_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        // 可视化轨迹
        visual_tools.publishText(text_pose, "Moving to Home Pose", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_joint_model_group);
        visual_tools.trigger();
        arm_group_interface.move();
    }
    else
    {
        ROS_INFO("Failed to move to Home Pose");
    }
}

void missionTwo(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                const moveit::core::JointModelGroup *arm_joint_model_group,
                moveit_visual_tools::MoveItVisualTools &visual_tools,
                const Eigen::Isometry3d &text_pose)
{
    namespace rvt = rviz_visual_tools;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = arm_group_interface.getCurrentState();
    current_state->copyJointGroupPositions(arm_joint_model_group, joint_group_positions);

    // 设置A点的关节位置
    joint_group_positions[0] = M_PI;

    arm_group_interface.setJointValueTarget(joint_group_positions);

    // 规划并移动到点
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        arm_group_interface.move();
    }

    int order = 4; // 希尔伯特曲线的阶数
    std::vector<Point> hilbert_raw_points;
    hilbert(order, hilbert_raw_points, 0, 0);
    // 将生成的点缩放到范围内
    double w = 0.8;
    double h = 0.8;
    // 设置笛卡尔路径的目标位姿
    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<geometry_msgs::Pose> firstpoint;
    geometry_msgs::Pose target_pose = arm_group_interface.getCurrentPose().pose;
    int i = 0;
    for (const auto &point : hilbert_raw_points)
    {
        target_pose.position.x = - 1.2; // 固定高度，确保末端垂直于运动平面
        target_pose.position.y = (point.x) * w;
        target_pose.position.z = (point.y + 0.5) * h + 0.13;
        target_pose.orientation.z = 0.707107;
        target_pose.orientation.x = 0;
        target_pose.orientation.y = 0;
        target_pose.orientation.w = 0.707107;
        waypoints.push_back(target_pose);
        if (i == 0)
        {
            firstpoint.push_back(target_pose);
            i++;
        }
        
    }

    // 规划笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory1;
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    const double eef_step = 0.01;
    
    arm_group_interface.computeCartesianPath(firstpoint, eef_step, trajectory1);
    plan1.trajectory_ = trajectory1;
    arm_group_interface.execute(plan1);

    double fraction = 0.0;
    int attempts = 0;
    while (fraction < 0.99 && attempts < 100)
    {
        fraction = arm_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
        attempts++;
    } 

    ROS_INFO("Successfully computed Cartesian path (%.2f%% achieved)", fraction * 100.0);
    visual_tools.publishText(text_pose, "Executing Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(trajectory, arm_joint_model_group); // 可视化轨迹
    cartesian_plan.trajectory_ = trajectory;
    visual_tools.trigger();
    arm_group_interface.execute(cartesian_plan);
}

void additonalTaskOne(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                      const moveit::core::JointModelGroup *arm_joint_model_group,
                      moveit_visual_tools::MoveItVisualTools &visual_tools,
                      const Eigen::Isometry3d &text_pose)
{
    namespace rvt = rviz_visual_tools;

    // 生成希尔伯特曲线的原始点
    int order = 4; // 希尔伯特曲线的阶数
    std::vector<Point> hilbert_raw_points;
    hilbert(order, hilbert_raw_points, 0, 0);

    // 将生成的点映射到半球面上
    double radius = 0.25; // 半径为0.25米，直径为0.5米
    geometry_msgs::Point sphere_center;
    sphere_center.x = 0.65; // 设置球心的x坐标
    sphere_center.y = 0.65; // 设置球心的y坐标
    sphere_center.z = 0.65; // 设置球心的z坐标
    // 设置笛卡尔路径的目标位姿
    geometry_msgs::Pose temp_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    for (const auto &point : hilbert_raw_points)
    {
        double theta = point.x * M_PI;                     // 将x映射到[0, π/2]
        double phi = point.y * 2 * (5.0 / 6.0) * M_PI / 2; // 将y映射到[0, π/2]
        temp_pose.position.x = sphere_center.x + radius * sin(phi) * cos(theta);
        temp_pose.position.y = sphere_center.y + radius * sin(phi) * sin(theta);
        temp_pose.position.z = sphere_center.z + radius * cos(phi);
        waypoints.push_back(temp_pose);
    }
    Execute_CartesianPath_AllAtOnce(arm_group_interface, arm_joint_model_group, visual_tools, 
                                    waypoints, sphere_center, 0.05, 100, 5000);
}

void additonalTaskTwo(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                      const moveit::core::JointModelGroup *arm_joint_model_group,
                      moveit_visual_tools::MoveItVisualTools &visual_tools,
                      const Eigen::Isometry3d &text_pose)
{
    namespace rvt = rviz_visual_tools;

    // 生成希尔伯特曲线的原始点
    int order = 4; // 希尔伯特曲线的阶数
    std::vector<Point> hilbert_raw_points;
    hilbert(order, hilbert_raw_points, 0, 0);

    // 生成完整的摆线点
    std::vector<Point> cycloid_points = generateCycloid(hilbert_raw_points, 0.01, 5);

    // 将生成的点映射到半球面上
    double radius = 0.25; // 半径为0.25米，直径为0.5米
    geometry_msgs::Point sphere_center;
    sphere_center.x = 0.65; // 设置球心的x坐标
    sphere_center.y = 0.65; // 设置球心的y坐标
    sphere_center.z = 0.65; // 设置球心的z坐标
    // 设置笛卡尔路径的目标位姿
    geometry_msgs::Pose temp_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    for (const auto &point : cycloid_points)
    {
        double theta = point.x * M_PI;             // 将x映射到[0, π]
        double phi = point.y * (5.0 / 6.0) * M_PI; // 将y映射到[0, π]
        temp_pose.position.x = sphere_center.x + radius * sin(phi) * cos(theta);
        temp_pose.position.y = sphere_center.y + radius * sin(phi) * sin(theta);
        temp_pose.position.z = sphere_center.z + radius * cos(phi);
        waypoints.push_back(temp_pose);
    }
    Execute_CartesianPath_AllAtOnce(arm_group_interface, arm_joint_model_group, visual_tools, waypoints, sphere_center, 0.05, 1000, 5000);
}

void additonalTaskThree(moveit::planning_interface::MoveGroupInterface &arm_group_interface,
                        const moveit::core::JointModelGroup *arm_joint_model_group,
                        moveit::planning_interface::MoveGroupInterface &table_group_interface,
                        const moveit::core::JointModelGroup *table_joint_model_group,
                        moveit::planning_interface::MoveGroupInterface &combine_group_interface,
                        const moveit::core::JointModelGroup *combine_joint_model_group,
                        const Eigen::Isometry3d &text_pose)
{
    namespace rvt = rviz_visual_tools;
    
    moveit_visual_tools::MoveItVisualTools visual_tools("tableEeffortLink");

    // 使用线程启动节点
    std::thread display_trajectory_thread([](){
        system("rosrun arm_control display_trajectory_node");
    });
    display_trajectory_thread.detach(); // 分离线程

    // 生成希尔伯特曲线的原始点
    int order = 3; // 希尔伯特曲线的阶数
    std::vector<Point> hilbert_raw_points;
    hilbert(order, hilbert_raw_points, 0, 0);

    int first_angel_flag = 0; // 第一个角度标志位，只改一次角度
    for (const auto &point : hilbert_raw_points)
    {
        // 转台角度计算
        double theta = point.x * M_PI;                     // 将x映射到[0, π] 经度
        double phi = point.y * 2 * (5.0 / 9.0) * M_PI / 2; // 将y映射到[0, π/2] 纬度
        // 臂末端位置计算
        double D = 1.0;      // 转台中心到原点的距离
        double d = 0.059;    // 旋转中心到球心的距离
        double r = 0.25;     // 球的半径
        double offset = 0.42; // 臂末端执行器offset
        double x = sin(phi) * d + D;
        double z = -cos(phi) * d + r + offset;

        // 设置臂末端笛卡尔路径目标位置
        geometry_msgs::Pose target_pose = arm_group_interface.getCurrentPose().pose;
        target_pose.position.x = x;
        target_pose.position.y = 0;
        target_pose.position.z = z;

        if (first_angel_flag == 0)
        {
            // 设置臂末端姿态，使其垂直于地面
            target_pose.orientation.x = -0.707107;
            target_pose.orientation.y = 0;
            target_pose.orientation.z = 0;
            target_pose.orientation.w = 0.707107;
            first_angel_flag = 1;
        }
        

        arm_group_interface.setPoseTarget(target_pose);

        // 规划并执行转台和臂末端的路径
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        // 获取当前转台关节角度
        std::vector<double> combine_joint_group_positions;
        moveit::core::RobotStatePtr current_state = combine_group_interface.getCurrentState();
        current_state->copyJointGroupPositions(combine_joint_model_group, combine_joint_group_positions);
        // 设置转台目标关节角度
        combine_joint_group_positions[6] = phi;
        combine_joint_group_positions[7] = theta;
        
        if (success)
        {
            // 获取轨迹中的所有关节角度
            moveit_msgs::RobotTrajectory& trajectory = my_plan.trajectory_;
            trajectory_msgs::JointTrajectory& joint_trajectory = trajectory.joint_trajectory;
            std::vector<trajectory_msgs::JointTrajectoryPoint>& points = joint_trajectory.points;
            // 读取最后关节角度
            for (int i = 0; i < 6; i++)
            {
                combine_joint_group_positions[i] = points.back().positions[i];
            }
            combine_group_interface.setPlannerId("RRTConnectkConfigDefault");
            combine_group_interface.setJointValueTarget(combine_joint_group_positions);
            moveit::planning_interface::MoveGroupInterface::Plan combine_plan;
            combine_group_interface.plan(combine_plan);
            // 执行
            combine_group_interface.execute(combine_plan);
        }
    }
    system("rosnode kill /display_trajectory_node");
}