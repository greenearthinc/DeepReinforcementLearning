/** tutorial ROS Node **/

#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/transform_listener.h>

class Localizer
{
public:
	Localizer(ros::NodeHandle& nh)
	{
		ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, &Localizer::visionCallback, this);
		server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
	}
	void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
	{
	last_msg_ = msg;
	//ROS_INFO_STREAM(last_msg_->pose.pose);
	}

	tf::TransformListener listener_;

	ros::Subscriber ar_sub_;
	fake_ar_publisher::ARMarkerConstPtr last_msg_;

	ros::ServiceServer server_;


bool localizePart(myworkcell_core::LocalizePart::Request& req,
                  myworkcell_core::LocalizePart::Response& res)
{
	//read last message
	fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
	if (!p) return false;

//Transform over-the-wire format of 'geometry_msgs::Pose' to 'tf::Transform object'
	tf::Transform cam_to_target;
	tf::poseMsgToTF(p->pose.pose, cam_to_target);

//Use the listener object to lookup the latest transform between the request.base_frame and the reference frame from the ARMarker message (which should be “camera_frame”):
	tf::StampedTransform req_to_cam;
	listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);

//Using the above information transform the object pose into the target frame
	tf::Transform req_to_target;
	req_to_target = req_to_cam * cam_to_target;

//Return the transformed pose in the service response
	tf::poseTFToMsg(req_to_target, res.pose);

	// res.pose = p->pose.pose;
	return true;	
}
};

int main(int argc, char* argv[])
{

// This must be called before anything else ROS-related
ros::init(argc, argv, "vision_node");

//Creating a ROS node handle
ros::NodeHandle nh;

// The Localizer class provides this node's ROS interfaces

Localizer localizer(nh);

ROS_INFO("Vision Node running");

//Keep program running
ros::spin();

}
