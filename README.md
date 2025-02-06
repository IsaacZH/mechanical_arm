<div align="center">
  
# Mechanical Arm 💪
<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow"></a>
[![Email](https://img.shields.io/badge/Email-1812924685@qq.com-green)](mailto:1812924685@qq.com)

![](https://raw.githubusercontent.com/IsaacZH/FigureBed/master/20250206183321.png)

</div>

<br>

--------

<br>
在ROS中使用Moveit！实现对机械臂的运动学规划。

### 实现功能
- Gazebo联合Moveit仿真。
- 关节角度空间规划。
- 笛卡尔空间规划。
- 球面规划算法。
- 联合二轴转台规划。

### 求解器
- 求解器更改为trac_ik，求解更加高效。
https://bitbucket.org/traclabs/trac_ik/src/master/

### 转台和机械臂联合规划
- 在setup assistant 中配置一个虚拟联合组，在这个联合组中一起规划。
![](https://raw.githubusercontent.com/IsaacZH/FigureBed/master/20250206184009.png)

### 运行方法
- 启动Rviz
```
roslaunch kuka_moveit_config demo.launch
```
- 启动控制节点
```
rosrun arm_control move_group_interface
```
- 每个任务结束后，在RvizVisualGUI窗口中点击NEXT即可运行下一个任务

### 运行展示
https://github.com/user-attachments/assets/5f22f164-376e-4176-b523-8445d49a9e82
