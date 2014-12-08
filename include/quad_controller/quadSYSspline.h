#ifndef QUAD_SYS_SPLINE_H_
#define QUAD_SYS_SPLINE_H_

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>


class quadSYSspline{
private:
	int stop;
	int time;
	Eigen::VectorXf current_state;

	//Quadrotor state-space system
	Eigen::Vector3f zeta;
	Eigen::Vector3f eta;
	Eigen::Vector3f etaDot;
	Eigen::Vector3f zetaDot;
	//
	double phiDot;
	double thetaDot;
	double siDot;
	//
	double phi;
	double theta;
	double si;
	
	//Diff parameters
	Eigen::Vector3f sd1;
	Eigen::Vector3f sd2;
	
	//Integration parameters
	Eigen::Vector3f r1i;
	Eigen::Vector3f r2i;
	
	//NN weight parameters
	//Need to be properly initialized
	Eigen::VectorXf w1col;
	Eigen::VectorXf w2col;
	Eigen::MatrixXf w1;
	Eigen::MatrixXf w2;

	//Trajectory Planning
	Eigen::MatrixXf xa3to5;
	Eigen::MatrixXf ya3to5;
	Eigen::MatrixXf za3to5;

	//NN estimation
	Eigen::VectorXf phi1;
	Eigen::MatrixXf vNN1;
	Eigen::MatrixXf error1;
	Eigen::MatrixXf error1Dot;
	Eigen::Vector3f r1;

	//Global parameters
	Eigen::Vector3f zetaDDot1;
	Eigen::Vector3f zetaDDot2;

	//Quadrotor Parameters
	Eigen::Matrix3f M1;
	Eigen::Vector3f MG;
	Eigen::Matrix3f I;

	//Proportional and Integral gains
	Eigen::Matrix3f lambda1;
	Eigen::Matrix3f lambda2;
	Eigen::Matrix3f Kr1;
	Eigen::Matrix3f Kr2;
	double K_i1;
	double K_i2;

	//NN information
	double nn;
	double ip1;
	double ip2;
	double op;
	Eigen::Matrix3f F1;
	Eigen::Matrix3f F2;
	double kali1;
	double kali2;

	//Differentiator Parameters
	Eigen::Matrix3f AA;
	Eigen::Matrix3f BB;
	Eigen::Matrix3f CC;
	Eigen::Matrix3f DD;

	//Trajectory Planning
	double x0;
	double y0;
	double z0;
	double xTf;
	double yTf;
	double zTf;

	//Fd calculation
	Eigen::VectorXf Fd;

	//Solve for thetaD, phiD and uD from Fd
	double a;
	double b;
	double u;
	double thetaD;
	double phiD;
	double siD;

	//Used in J calculation
	double Ix;
	double Iy;
	double Iz;
	Eigen::Matrix3f J;

	//r2 calculation
	Eigen::Vector3f etaD;
	Eigen::Vector3f diffDot1;
	Eigen::Vector3f etaDDot1;
	Eigen::Vector3f diffDot2;
	Eigen::Vector3f e2;
	Eigen::Vector3f e2Dot;
	Eigen::Vector3f r2;

	//NN2 calculations
	Eigen::MatrixXf V2;
	Eigen::VectorXf inx2;
	Eigen::VectorXf phi2;
	Eigen::Vector3f vNN2;

	//Tao calculation
	Eigen::Vector3f tao;

	//Quadrotor System equations
	Eigen::Vector3f Axyz;
	Eigen::Vector3f Aeta;
	Eigen::Matrix3f Teta;
	Eigen::Matrix3f TetaDot;
	Eigen::Matrix3f C1;
	Eigen::Matrix3f C2;
	Eigen::Matrix3f Ceta;
	Eigen::Vector3f zetaDot2;
	Eigen::Vector3f etaDot2;

	//NN Dynamics
	Eigen::VectorXf r;
	double norm_r;
	Eigen::MatrixXf wDot1;
	Eigen::VectorXf w1Dot;
	Eigen::MatrixXf wDot2;
	Eigen::VectorXf w2Dot;
	Eigen::VectorXf wDot;

	//Constructing sDot vector
	Eigen::VectorXf sDot;

	//Constants
	Eigen::MatrixXf V1;
	
	Eigen::Vector3f zetaD;
public:
	quadSYSspline(double nn = 10);
	void calculateNNWeightParameters();
	void trajectoryPlanning();
	void NN1Estimation();
	void FdCalc();
	void calcFromFd();
	void calcJ();
	void calcR2();
	void calcNN2();
	void calcTao();
	void calcSystemEqations();
	void calcNNDynamics();
	void buildSDot();
	void process(int t, Eigen::VectorXf current_state);

	//Getters and Setters
	void setW1col(Eigen::VectorXf w1col);
	void setW2col(Eigen::VectorXf w2col);
	void setNN(double nn = 10);
	void setM1(double m = 1);
	void setMG(double mass = 1,double gravity = 9.8);
	void setI(double constant = 5);
	void setLambda1(double constant=10);
	void setLambda2(double constant=100);
	void setKr1(double constant=15);
	void setKr2(double constant=1000);
	void setK_i1(double constant = 75);
	void setK_i2(double constant = 50);
	void setAA(double constant = -100);
	void setBB(double constant = 1);
	void setCC(double constant = -100);
	void setDD(double constant = 0.01);
	void setIp1(double constant = 15);
	void setIp2(double constant = 18);
	void setOp(double constant = 3);
	void setF1(double constant = 5);
	void setF2(double constant = 5);
	void setKali1(double constant = 0.01);
	void setKali2(double constant = 0.01);
	void setX0(double constant = 0.01);
	void setY0(double constant = 0.01);
	void setZ0(double constant = 0.01);
	void setXTf(double constant = 0.01);
	void setYTf(double constant = 0.01);
	void setZTf(double constant = 0.01);
	Eigen::VectorXf getSDot();

	//Debug
	void printParameters();

	//Helper functions
	void signNum(Eigen::Vector3f& x);

};



#endif /**QUAD_SYS_SPLINE_H_**/
