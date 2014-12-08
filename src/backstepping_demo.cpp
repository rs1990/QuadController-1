#include <quad_controller/quadSYSspline.h>
#include <quad_controller/backstepping_demo.h>
#include <quad_controller/quad_controller.h>

ros::Publisher lin_pos;
ros::Publisher lin_pos_error;
ros::Publisher ang_pos;
ros::Publisher ang_pos_error;

QuadController::QuadController(double dest_lin_vel, double dest_ang_vel){
	this->imu_sub = nh.subscribe<sensor_msgs::Imu>("/ardrone/imu",1,&QuadController::imu_callback,this);
	this->nav_data_sub = nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata",1,&QuadController::navData_callback,this);
	this->mag_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/ardrone/mag",1,&QuadController::mag_callback,this);
	
	this->desired_linear_velocity(0) = 0.2;
	this->desired_linear_velocity(1) = 0.2;
	this->desired_linear_velocity(2) = 0.2;
	this->desired_angular_velocity(0) = 0.2;
	this->desired_angular_velocity(1) = 0.2;
	this->desired_angular_velocity(2) = 0.2;
	
	this->identity = Eigen::Matrix3f::Identity(); 
	
	double nn = 10;
	this->w1_col = Eigen::VectorXf::Zero(nn*3);
	this->w2_col = Eigen::VectorXf::Zero(nn*3);
}

void QuadController::imu_callback(sensor_msgs::Imu msg){
	this->imu_data = msg;
}

void QuadController::mag_callback(geometry_msgs::Vector3Stamped msg){
	this->mag_data = msg;
}

void QuadController::navData_callback(ardrone_autonomy::Navdata msg){
	this->nav_data = msg;
}

void QuadController::set_desired_position(double x_dest, double y_dest, double z_dest, double roll_dest, double pitch_dest, double yaw_dest){
	this->desired_linear_position(0) = x_dest;
	this->desired_linear_position(1) = y_dest;
	this->desired_linear_position(2) = z_dest;
	this->desired_angular_position(0) = roll_dest;
	this->desired_angular_position(1) = pitch_dest;
	this->desired_angular_position(2) = yaw_dest;
}

void QuadController::build_state(){
	Eigen::VectorXf current_state(84);
	
	/***Input current state for quadrotor***/
	current_state(0) = this->mag_data.vector.x;
	current_state(1) = this->mag_data.vector.y;
	current_state(2) = this->mag_data.vector.z;
	//Get quaternion and conver to rpy
	tf::Quaternion quat(this->imu_data.orientation.x,this->imu_data.orientation.y,this->imu_data.orientation.z,this->imu_data.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	current_state(3) = roll;
	current_state(4) = pitch;
	current_state(5) = yaw;
	current_state(6) = this->nav_data.vx;
	current_state(7) = this->nav_data.vy;
	current_state(8) = this->nav_data.vz;
	current_state(9) = this->imu_data.angular_velocity.x;
	current_state(10) = this->imu_data.angular_velocity.y;
	current_state(11) = this->imu_data.angular_velocity.z;
	/***Diff Parameters(error)***/
	current_state(12) = this->desired_linear_position(0) - current_state(0);
	current_state(13) = this->desired_linear_position(1) - current_state(1);
	current_state(14) = this->desired_linear_position(2) - current_state(2);
	current_state(15) = this->desired_angular_position(0) - current_state(3);
	current_state(16) = this->desired_angular_position(1) - current_state(4);
	current_state(17) = this->desired_angular_position(2) - current_state(5);
	
	/***Integration Parameters(sliding errors)***/
	Eigen::Vector3f error_lin_vel, error_lin_pos;
	error_lin_vel(0) = this->desired_linear_velocity(0) - current_state(6);
	error_lin_vel(1) = this->desired_linear_velocity(1) - current_state(7);
	error_lin_vel(2) = this->desired_linear_velocity(2) - current_state(8);
	error_lin_pos(0) = current_state(12);
	error_lin_pos(1) = current_state(13);
	error_lin_pos(2) = current_state(14);
	r1 = error_lin_vel + ((10*this->identity) * error_lin_pos);
	
	Eigen::Vector3f error_ang_vel, error_ang_pos;
	error_ang_vel(0) = this->desired_angular_velocity(0) - current_state(9);
	error_ang_vel(1) = this->desired_angular_velocity(1) - current_state(10);
	error_ang_vel(2) = this->desired_angular_velocity(2) - current_state(11);
	error_ang_pos(0) = current_state(15);
	error_ang_pos(1) = current_state(16);
	error_ang_pos(2) = current_state(17);
	r2 = error_ang_vel + ((100*this->identity) * error_ang_pos);
	
	current_state(18) = r1(0);
	current_state(19) = r1(1);
	current_state(20) = r1(2);
	current_state(21) = r2(0);
	current_state(22) = r2(1);
	current_state(23) = r2(2);
	
	for(int i=0;i<this->w1_col.size();i++){
		current_state(24+i) = w1_col(i);
    }
    
    for(int i=0;i<this->w2_col.size();i++){
		current_state(24+w1_col.size()+i) = w2_col(i);
	}
	
	std::cout << "Time:" << ros::Time::now() << std::endl;
	std::cout << "Current Pos" << std::endl << current_state(0) << " " << current_state(1) << " " << current_state(2) << std::endl;
	std::cout << "Current Angle"<< std::endl << current_state(3) << " " << current_state(4) << " " << current_state(5) << std::endl;
	std::cout << "Linear Velocity" << std::endl << current_state(6) << " " << current_state(7) << " " << current_state(8) << std::endl;
	std::cout << "Angular Velocity" << std::endl << current_state(9) << " " << current_state(10) << " " << current_state(11) << std::endl;
	std::cout << "Linear Pos Error" << std::endl << current_state(12) << " " << current_state(13) << " " << current_state(14) << std::endl;
	std::cout << "Angular Pos Error" << std::endl << current_state(15) << " " << current_state(16) << " " << current_state(17) << std::endl;
	std::cout << "Linear Sliding Error" << std::endl << current_state(18) << " " << current_state(19) << " " << current_state(20) << std::endl;
	std::cout << "Angular Sliding Error" << std::endl << current_state(21) << " " << current_state(22) << " " << current_state(23) << std::endl;
	std::cout << "Final size: " << 24 + w1_col.size() + w2_col.size() << std::endl;
	std::cout << std::endl;
	
	/***Pubish parts of the current state***/
	geometry_msgs::Vector3 tb_publish;
	tb_publish.x = current_state(0);
	tb_publish.y = current_state(1);
	tb_publish.z = current_state(2);
	lin_pos.publish(tb_publish);
	tb_publish.x = current_state(12);
	tb_publish.y = current_state(13);
	tb_publish.z = current_state(14);
	lin_pos_error.publish(tb_publish);
	tb_publish.x = current_state(3);
	tb_publish.y = current_state(4);
	tb_publish.z = current_state(5);
	ang_pos.publish(tb_publish);
	tb_publish.x = current_state(15);
	tb_publish.y = current_state(16);
	tb_publish.z = current_state(17);
	ang_pos_error.publish(tb_publish);
	
	
	this->currentState = current_state;
}

int main(int argc, char** argv){
	
	//Set up ROS
	ros::init(argc,argv,"backstepping_demo");
	ros::NodeHandle n;
	
	//Create quadController object
	QuadController control;
	quadSYSspline backstepping_controller;
	int time = 0;
	
	//Initialize Publisher
	lin_pos = n.advertise<geometry_msgs::Vector3>("linear_position",1000);
	lin_pos_error = n.advertise<geometry_msgs::Vector3>("linear_position_error",1000);
	ang_pos = n.advertise<geometry_msgs::Vector3>("angular_position",1000);
	ang_pos_error = n.advertise<geometry_msgs::Vector3>("angular_position_error",1000);
	
	//Set init and desired position values
	ros::spinOnce();
	control.build_state();
	backstepping_controller.setX0(control.currentState(0));
	backstepping_controller.setY0(control.currentState(1));
	backstepping_controller.setZ0(control.currentState(2));
	backstepping_controller.setXTf(control.currentState(0));
	backstepping_controller.setYTf(control.currentState(1));
	backstepping_controller.setZTf(control.currentState(2));
	

	
	//Rate to run
	int rate = 1000;
	ros::Rate r(rate);
	
	//Main loop
	while(n.ok()){
		ros::spinOnce();
		control.build_state();
		//backstepping_controller.process(ros::Time::now().sec,control.currentState);
		backstepping_controller.process(time,control.currentState);
		time++;
		r.sleep();
	}

	return 0;
}
