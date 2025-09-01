<div align="center">
  
# Mechanical Arm 💪
<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow"></a>
[![Email](https://img.shields.io/badge/Email-1812924685@qq.com-green)](mailto:1812924685@qq.com)

![](https://raw.githubusercontent.com/IsaacZH/FigureBed/master/20250206183321.png)

</div>

<br>

--------

<br>

This project demonstrates motion planning of a robotic arm using **MoveIt!** in ROS.

# Features
- Gazebo simulation integrated with MoveIt!.
- Joint space motion planning.
- Cartesian space motion planning.
- Spherical planning algorithm.
- Joint planning with a dual-axis turntable.

# Solver
- Replaced the default solver with **trac_ik** for more efficient IK solving.  
https://bitbucket.org/traclabs/trac_ik/src/master/

# Combined Planning with Turntable and Arm
- Configure a virtual joint group in Setup Assistant to plan both the arm and turntable together.  
![](https://raw.githubusercontent.com/IsaacZH/FigureBed/master/20250206184009.png)

# How to Run
- Launch Rviz:
```bash
roslaunch kuka_moveit_config demo.launch
````

* Start the control node:

```bash
rosrun arm_control move_group_interface
```

* After each task finishes, click **NEXT** in the RvizVisualGUI window to run the next task.

# Demo

[https://github.com/user-attachments/assets/5f22f164-376e-4176-b523-8445d49a9e82](https://github.com/user-attachments/assets/5f22f164-376e-4176-b523-8445d49a9e82)

