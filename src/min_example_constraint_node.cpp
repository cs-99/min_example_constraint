#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_example_constraint");
	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Duration(5).sleep();

    moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	std::vector<double> joint_vals = move_group_interface.getCurrentJointValues();
	joint_vals[5] += 0.4;
	move_group_interface.setJointValueTarget(joint_vals);

	moveit::planning_interface::MoveGroupInterface::Plan rot_plan;
	move_group_interface.plan(rot_plan);
	move_group_interface.execute(rot_plan);
	
	geometry_msgs::PoseStamped pose_st = move_group_interface.getCurrentPose();
	pose_st.pose.position.x = -0.6;
	pose_st.pose.position.y = 0.0;
	pose_st.pose.position.z = 0.6;
	move_group_interface.setPoseTarget(pose_st);

	moveit_msgs::OrientationConstraint orientation_constraint;
	orientation_constraint.weight = 1.0;
	orientation_constraint.link_name = "panda_link8";
	orientation_constraint.header.frame_id = "world";
	orientation_constraint.orientation = pose_st.pose.orientation;
	orientation_constraint.absolute_x_axis_tolerance = 0.1;
	orientation_constraint.absolute_y_axis_tolerance = 0.1;
	orientation_constraint.absolute_z_axis_tolerance = 0.1;
	moveit_msgs::Constraints constraints;
	constraints.orientation_constraints.push_back(orientation_constraint);
	move_group_interface.setPathConstraints(constraints);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	move_group_interface.plan(plan);
	move_group_interface.execute(plan);

	ros::waitForShutdown();
	return 0;
}