# Franka-arm_planning_scene

ARMLab Planning scene and ros package for franka arm in Kinetic devel. The planning scene ensures no collision with table or pillar in the lab.



## 

To use the armlab planning scene, first build the file by using catkin_bt or catkin build. 

Next, execute following code: 

```

roslaunch panda_moveit_custom moveit_basic.launch

```

This launch file runs Rviz, franka position controller, and planning scene code. Planning scene code is located in 'panda_moveit_custom/src/armlab_planningscene.cpp'.

Inside of Rviz, click 'next' in RvizVisualToolsGui tab. The robot will move below the object avoiding obstacles. 

![robot_img](https://imgur.com/WquQYp1)


## Log

### 07.28.2020:

- we are currently using: git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git


- changed pandra_arm.xacro / panda_arm_hand.srdf.xacro inside of config- recommendation from https://github.com/frankaemika/franka_ros/issues/39

- changed default controller value: inside of roscd franka_controllers/config, 

	changed the goal for each joint from 0.05 into 0.0 
	added stopped_velocity_tolerance: 0 

- changed launch file 'moveit_basic.launch' inside panda_moveit_custom to ensure the controller of the robot with ros - for now, we are using default controller inside franka_controllers/config.

### 07.30.2020:

- combined 'moveit_basic.launch' with 'moveit_onlycode.launch'

- included panda_moveit_config repo
