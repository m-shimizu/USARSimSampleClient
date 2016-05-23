#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_transform");
	ros::NodeHandle nh;
	
	std::string globalFrame;
	if(!nh.getParam("usarsim/globalFrame",globalFrame))
	{
	  ROS_WARN("usarsim/globalFrame not specified. Using default global frame.");
	  exit(1);
	}
	ros::Rate r(2.0);
	tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0));
	//reflect y and z axes
	btMatrix3x3 rotationMatrix(
	  1,  0,  0,
	  0, -1,  0,
	  0,  0, -1);
	transform.setBasis(rotationMatrix);
	while(ros::ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), globalFrame, "odom"));
		r.sleep();
	}
	return 0;
}
