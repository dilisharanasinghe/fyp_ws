#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>

#define M_PI           3.14159265358979323846
#define GYRO_OFFSET 0.0125 //-0.02

class imuPublisher{
  public:
    imuPublisher(){
      ros::NodeHandle nh_;
      sub_ = nh_.subscribe("base_controller_data", 1000, &imuPublisher::baseCallback,this);
      pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 50);


    } 

    void baseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
	    double yaw_rate = msg->data[6];
	    yaw_rate = yaw_rate/180.0*M_PI - GYRO_OFFSET;

      double accelerometer_rate = msg->data[7];
	  
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "imu_link";

  	  imu_msg.angular_velocity.x = 0;
  	  imu_msg.angular_velocity.y = 0;
  	  imu_msg.angular_velocity.z = yaw_rate;
  	  imu_msg.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0.001};

  	  imu_msg.linear_acceleration.x = accelerometer_rate;
      imu_msg.linear_acceleration.y = 0; 
      imu_msg.linear_acceleration.z = 9.8;
  	  imu_msg.linear_acceleration_covariance = { 0.01, 0, 0, 0, 0, 0, 0, 0, 0 };

  	  imu_msg.orientation_covariance = {-1,   0,   0, 0,  -1,   0, 0,   0,  -1 };

      pub_.publish(imu_msg);

    }


  protected:
    ros::Subscriber sub_;
    ros::Publisher pub_;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher");

  imuPublisher imu_publisher;

  ros::spin();
  
}
