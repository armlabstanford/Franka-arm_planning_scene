#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);


 move_group.setStartState (* move_group.getCurrentState ());


  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);



  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Path constraints can easily be specified for a link on the robot.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);



  visual_tools.deleteAllMarkers();

/////////////////////// move robot into the initial state
  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose beginning_pose;
  beginning_pose.orientation.w = 0.0;
  beginning_pose.orientation.x = 1.0;
  beginning_pose.position.x = 0.4;
  beginning_pose.position.y = 0.0;
  beginning_pose.position.z = 0.4;
  move_group.setPoseTarget(beginning_pose);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  //move_group.move();

  visual_tools.trigger();



  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Define a collision object ROS message.
   moveit_msgs::CollisionObject collision_object;
  moveit_msgs::CollisionObject collision_desk_left;  
  moveit_msgs::CollisionObject collision_desk_right;
  moveit_msgs::CollisionObject collision_desk_middle;
  moveit_msgs::CollisionObject collision_desk2;
  moveit_msgs::CollisionObject collision_test;

  collision_desk_left.header.frame_id = move_group.getPlanningFrame();
  collision_desk_right.header.frame_id = move_group.getPlanningFrame();
  collision_desk_middle.header.frame_id = move_group.getPlanningFrame();
  collision_desk2.header.frame_id = move_group.getPlanningFrame();
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_test.header.frame_id = move_group.getPlanningFrame();


  // The id of the object is used to identify it.
  collision_object.id = "box1";
  collision_desk_left.id = "desk_left";
  collision_desk_right.id = "desk_right";
  collision_desk_middle.id = "desk_middle";

  collision_desk2.id = "desk2";
  collision_test.id = "test_obj";


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // red: x,
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 0.9;
  primitive.dimensions[2] = 2.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  // Think it is the center of mass
  box_pose.orientation.w = 1.0;
  //- actual safety distance - 0.4, x: 60cm
  box_pose.position.x = -1.0;
  // y:0cm
  box_pose.position.y = -0.0;
  box_pose.position.z = 1.2;

//////////////// main desk
  // Define a desk left side to add to the world.
  shape_msgs::SolidPrimitive primitive_left;
  primitive_left.type = primitive_left.BOX;

  primitive_left.dimensions.resize(3);
  // red: x,
  primitive_left.dimensions[0] = 0.92;
  primitive_left.dimensions[1] = 0.48;
  primitive_left.dimensions[2] = 0.78;

  // Define a pose_left for the box (specified relative to frame_id)
  geometry_msgs::Pose desk_pose_left;
  // Think it is the center of mass
  desk_pose_left.orientation.w = 1.0;

  //- actual safety distance - 0.4, x: 60cm
  desk_pose_left.position.x = -0.28+0.46;
  // y:0cm
  desk_pose_left.position.y = -0.40;
  desk_pose_left.position.z = -0.40;

  // Define a desk right side to add to the world.
  shape_msgs::SolidPrimitive primitive_right;
  primitive_right.type = primitive_right.BOX;

  primitive_right.dimensions.resize(3);
  // red: x,
  primitive_right.dimensions[0] = 0.92;
  primitive_right.dimensions[1] = 0.48;
  primitive_right.dimensions[2] = 0.78;

  // Define a pose_right for the box (specified relative to frame_id)
  geometry_msgs::Pose desk_pose_right;
  // Think it is the center of mass
  desk_pose_right.orientation.w = 1.0;

  //- actual safety distance - 0.4, x: 60cm
  desk_pose_right.position.x = -0.28+0.46;
  // y:0cm
  desk_pose_right.position.y = 0.40;
  desk_pose_right.position.z = -0.40;

 // Define a desk right side to add to the world.
  shape_msgs::SolidPrimitive primitive_middle;
  primitive_middle.type = primitive_middle.BOX;

  primitive_middle.dimensions.resize(3);
  // red: x,
  primitive_middle.dimensions[0] = 0.48;
  primitive_middle.dimensions[1] = 0.48;
  primitive_middle.dimensions[2] = 0.78;

  // Define a pose_right for the box (specified relative to frame_id)
  geometry_msgs::Pose desk_pose_middle;
  // Think it is the center of mass
  desk_pose_middle.orientation.w = 1.0;

  //- actual safety distance - 0.4, x: 60cm
  desk_pose_middle.position.x = 0.40;
  // y:0cm
  desk_pose_middle.position.y = 0.0;
  desk_pose_middle.position.z = -0.40;




/////////////////main desk end


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive3;
  primitive3.type = primitive3.BOX;

  primitive3.dimensions.resize(3);
  // red: x,
  primitive3.dimensions[0] = 1.22;
  primitive3.dimensions[1] = 0.92;
  primitive3.dimensions[2] = 0.78;
  // Define a pose for the other desk (specified relative to frame_id)
  geometry_msgs::Pose desk_pose2;
  // Think it is the center of mass
  desk_pose2.orientation.w = 1.0;

  //- actual safety distance - 0.4, x: 60cm
  desk_pose2.position.x = -0.55;
  // y:0cm
  desk_pose2.position.y = 1.22;
// because of collision 
  desk_pose2.position.z = -0.40;


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive4;
  primitive4.type = primitive4.BOX;

  primitive4.dimensions.resize(3);
  // red: x,
  primitive4.dimensions[0] = 0.25;
  primitive4.dimensions[1] = 0.30;
  primitive4.dimensions[2] = 0.20;
  // Define a pose for the other desk (specified relative to frame_id)
  geometry_msgs::Pose desk_pose3;
  // Think it is the center of mass
  desk_pose3.orientation.w = 1.0;

  //- actual safety distance - 0.4, x: 60cm
  desk_pose3.position.x = 0.50;
  // y:0cm
  desk_pose3.position.y = -0.3;
  desk_pose3.position.z = 0.6;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
////// main desk 
  collision_desk_left.primitives.push_back(primitive_left);
  collision_desk_left.primitive_poses.push_back(desk_pose_left);
  collision_desk_left.operation = collision_desk_left.ADD;

  collision_desk_right.primitives.push_back(primitive_right);
  collision_desk_right.primitive_poses.push_back(desk_pose_right);
  collision_desk_right.operation = collision_desk_right.ADD;

  collision_desk_middle.primitives.push_back(primitive_middle);
  collision_desk_middle.primitive_poses.push_back(desk_pose_middle);
  collision_desk_middle.operation = collision_desk_middle.ADD;

////// main desk end

  collision_desk2.primitives.push_back(primitive3);
  collision_desk2.primitive_poses.push_back(desk_pose2);
  collision_desk2.operation = collision_desk2.ADD;

  collision_test.primitives.push_back(primitive4);
  collision_test.primitive_poses.push_back(desk_pose3);
  collision_test.operation = collision_test.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
////// desk insertion
  collision_objects.push_back(collision_desk_left);
  collision_objects.push_back(collision_desk_right);
  collision_objects.push_back(collision_desk_middle);
//////
  collision_objects.push_back(collision_desk2);
  collision_objects.push_back(collision_test);


  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());

  geometry_msgs::Pose another_pose;
//  another_pose.orientation.w = 1.0;
  another_pose.orientation.x = 1.0;
// doesn't work
//  another_pose.orientation = move_group.getCurrentPose().rotation;

  another_pose.orientation = move_group.getCurrentPose().pose.orientation;
  another_pose.position.x = 0.2;
  another_pose.position.y = -0.3;
  another_pose.position.z = 0.2;
  move_group.setPoseTarget(another_pose);


  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");
  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the path planning is over");

// before move, save current state 
  geometry_msgs::Pose previous_pose;
  previous_pose = move_group.getCurrentPose().pose;
 
   move_group.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the robot moves");
  visual_tools.trigger();


  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(previous_pose);


  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7(move robot in previous pos) %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the robot moves");
  visual_tools.trigger();
  move_group.move();

visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the robot moves");
  visual_tools.trigger();

  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(collision_desk_left.id);
  object_ids.push_back(collision_desk_right.id);
  object_ids.push_back(collision_desk_middle.id);
  object_ids.push_back(collision_desk2.id);
  object_ids.push_back(collision_test.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  // END_TUTORIAL
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");


  ros::shutdown();
  return 0;
}

