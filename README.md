# UR10e_Arm_With_Lift
This package demonstrates **constant-velocity Cartesian motion** of a UR10e arm with a lift joint, using MoveIt 2 in ROS 2 Humble.  

## Setps to Run

### Make a Workspace
```
mkdir <path_to_ws>/UR10e_with_lift_ws
```

### Install required packages
```
cd <path_to_ws>/UR10e_with_lift_ws
mkdir src
cd src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd <path_to_ws>/UR10e_with_lift_ws
colcon build
```

Source the workspace
```
source <path_to_ws>/UR10e_with_lift_ws/install/setup.bash
```

### Clone and Build this repository
```
cd <path_to_ws>/UR10e_with_lift_ws/src
git clone https://github.com/Rishabh96M/UR10e_Arm_With_Lift.git
cd <path_to_ws>/UR10e_with_lift_ws
colcon build --packages-select ur10e_arm_with_lift
```

### Launch on Gazebo and RViz
You can launch the robot on Gazebo and RViz, control it with joint_state_publisher_gui

```
source <path_to_ws>/UR10e_with_lift_ws/install/setup.bash
ros2 launch ur10e_arm_with_lift ur_with_lift_sim_control.launch
```

### Controlling the Robot with Moveit
Launch the robot on Gazebo and RViz with moveit
```
source <path_to_ws>/UR10e_with_lift_ws/install/setup.bash
ros2 launch ur10e_arm_with_lift ur_with_lift_sim_moveit.launch
```

If you want to go to a point, in a new terminal
```
source <path_to_ws>/UR10e_with_lift_ws/install/setup.bash
ros2 run ur10e_arm_with_lift go_to_point <tx> <ty> <tz> <rx> <ry> <rz> <rw>

# Where, 
# <tx> <ty> <tz> are X, Y and Z co-ordinates 
# <rx> <ry> <rz> <rw> are orientation in Quaternion
```

If you want to draw a circle on a wall parallel to Y=0

In a new terminal
```
source <path_to_ws>/UR10e_with_lift_ws/install/setup.bash
ros2 run ur10e_arm_with_lift draw_circle_cartesian <x> <y> <z> <r>

# Where, 
# <x> <y> <z> are X, Y and Z of the center of circle 
# <r> is the radius
```
