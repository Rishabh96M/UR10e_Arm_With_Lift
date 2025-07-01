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

### Clone this repository
```
git clone 
```