#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
  {
	vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
	cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
  }
  void start(const std::string& base_frame)
	{
       
	 ROS_INFO("Attempting to localize part");
	 //Localize the part
	 myworkcell_core::LocalizePart srv;
	srv.request.base_frame = base_frame;
	
       ROS_INFO_STREAM("Requesting pose in base frame: " <<base_frame);
	if (!vision_client_.call(srv))
	{
		ROS_ERROR("Could not localize the part");
		return;
	}
	geometry_msgs::Pose move_target = srv.response.pose;
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	
	// Plan for robot to move to part
	move_group.setPoseReferenceFrame(base_frame);
	move_group.setPoseTarget(move_target); 
	move_group.move();

  	ROS_INFO_STREAM("part localized: " << srv.response);
	
	// Plan cartesian path
	myworkcell_core::PlanCartesianPath cartesian_srv;
	cartesian_srv.request.pose = move_target;
	if (!cartesian_client_.call(cartesian_srv))
		{
  		ROS_ERROR("Could not plan for path");
  		return;
		}
	// Execute descartes-planned path directly (bypassing MoveIt)
	ROS_INFO("Got cart path, executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = cartesian_srv.response.trajectory;
	ac_.sendGoal(goal);
	ac_.waitForResult();
	ROS_INFO("Done");
	}
  
  
private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");
  	std::string base_frame;
	private_node_handle.param<std::string>("base_frame", base_frame, "world"); //parameter name, string object reference, default value
  
ScanNPlan app(nh);
  
  ros::Duration(.5).sleep(); //wait for class to initialize
  app.start(base_frame);

  ROS_INFO("Scan and Plan node has been initialized");

  //ros::spin();
  ros::waitForShutdown();
}
