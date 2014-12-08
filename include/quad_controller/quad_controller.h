#ifndef BACKSTEPPING_CONTROLLER_H_
	#define BACKSTEPPING_CONTROLLER_H_
	#include <iostream>
	#include <ros/ros.h>
	#include <geometry_msgs/Vector3Stamped.h>
	#include <sensor_msgs/Imu.h>
	#include <ardrone_autonomy/Navdata.h>	
#endif

#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

class QuadController{
private:
	Eigen::Vector3f desired_linear_position;
	Eigen::Vector3f desired_angular_position;
	Eigen::Vector3f desired_linear_velocity; //Hardcoded in constructor
	Eigen::Vector3f desired_angular_velocity; //Hardcoded in constructor
	Eigen::Matrix3f identity;
	
	//Integration Paramters
	Eigen::Vector3f r1, r2;

	sensor_msgs::Imu imu_data;
	geometry_msgs::Vector3Stamped mag_data;
	ardrone_autonomy::Navdata nav_data;
	tf::Quaternion rpy;
	ros::Subscriber imu_sub;
	ros::Subscriber nav_data_sub;
	ros::Subscriber mag_sub;
	ros::NodeHandle nh;
	Eigen::VectorXf current_state;
	
	Eigen::VectorXf w1_col;
	Eigen::VectorXf w2_col;
	
	
public:
	QuadController(double dest_lin_vel = 0.2, double dest_ang_vel = 0.2);
	void imu_callback(sensor_msgs::Imu msg);
	void mag_callback(geometry_msgs::Vector3Stamped);
	void navData_callback(ardrone_autonomy::Navdata msg);
	void build_state();
	void set_desired_position(double x_dest, double y_dest, double z_dest, double roll_dest, double pitch_dest, double yaw_dest);
	void calc_sliding_error_r1();
	void calc_sliding_error_r2();
	//Current State
	Eigen::VectorXf currentState;
};
