#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <sstream>

//std_msgs::header correct_time_header; 
sensor_msgs::CameraInfo msg;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void getTimeCallback(const sensor_msgs::Image::ConstPtr& depth_msg)
{
  //correct_time = depth_msg.header.stamp;
 msg.header = depth_msg->header;
 msg.header.frame_id =  "camera_link";
  ROS_INFO("I got time");
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "depth_info");
  ros::NodeHandle n;
  ros::Publisher depth_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1000);
  ros::Subscriber get_time = n.subscribe("/camera/depth/image_raw", 1000, getTimeCallback);

  

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    //sensor_msgs::CameraInfo msg;

    //msg.header = correct_time_header;

    // msg.header.seq =  count; 
    // msg.header.stamp=  correct_time; 
    msg.header.frame_id =  "camera_link"; 
	
    msg.height = 480;
    msg.width = 640;
    msg.distortion_model ="plumb_bob";
    msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    msg.K= {575.8157348632812, 0.0, 314.5, 0.0, 575.8157348632812, 235.5, 0.0, 0.0, 1.0};
    msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.P = {575.8157348632812, 0.0, 314.5, 0.0, 0.0, 575.8157348632812, 235.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    msg.binning_x = 0;
    msg.binning_y = 0;
    msg.roi.x_offset = 0;
    msg.roi.y_offset = 0;
    msg.roi.height = 0;
    msg.roi.height = 0;
    msg.roi.do_rectify = false;
    
    depth_camera_info_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}



















