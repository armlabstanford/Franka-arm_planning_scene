# Franka-arm_planning_scene
Planning scene and ros package for franka arm in Kinetic devel

##Log

### 2020-0728:

- we are currently using: git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git


- changed pandra_arm.xacro / panda_arm_hand.srdf.xacro inside of config- recommendation from https://github.com/frankaemika/franka_ros/issues/39

- changed default controller value: inside of roscd franka_controllers/config, 

	changed the goal for each joint from 0.05 into 0.0 
	added stopped_velocity_tolerance: 0 

- changed launch file 'moveit_basic.launch' inside panda_moveit_custom to ensure the controller of the robot with ros - for now, we are using default controller inside franka_controllers/config.
