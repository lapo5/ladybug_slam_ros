#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
/**
 * @file
 */

/** x coordinate of the camera with respect to the robot */	double x_camera = 0.0; 
/** y coordinate of the camera with respect to the robot */	double y_camera = 0.0;
/** z coordinate of the camera with respect to the robot */	double z_camera = 0.0;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle n("~");

	// position of the camera with respect to the robot
	n.param<double>("x_camera", x_camera, 0.3);
	n.param<double>("y_camera", y_camera, 0.3);
	n.param<double>("z_camera", z_camera, 0.3);

	ros::Rate r(10000);
	tf::TransformBroadcaster broadcaster;

	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(x_camera, y_camera, z_camera));

	tf::Quaternion frame_rotation;
	frame_rotation.setRPY(0, 0, 0); 
	change_frame.setRotation(frame_rotation);
    
	while(ros::ok()){
		
		broadcaster.sendTransform(tf::StampedTransform(change_frame, ros::Time::now(), "/ladybug_slam/base_link", "/ladybug_slam/camera"));
				
		ros::spinOnce();
		r.sleep();
	}

    return 0;
}
