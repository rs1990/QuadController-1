#include <quad_controller/quadSYSspline.h>

quadSYSspline::quadSYSspline(double nn){

	//Initialize NN Parameters
	setNN(nn);
	setIp1();
	setIp2();
	setOp();
	setF1();
	setF2();
	setKali1();
	setKali2();

	//
	//V1 construction
	int ip1 = 15;
	V1 = Eigen::MatrixXf(ip1,this->nn);
	V1 << Eigen::MatrixXf::Identity(10,10), Eigen::MatrixXf::Zero(5,10);
	//std::cout << "Hello quadSYSspline!" << std::endl << V1 << std::endl;

	//Initialize QuadrotorParameters
	setMG();
	setI();
	setM1();

	//Initialize Proportional and Integral Gains
	setLambda1();
	setLambda2();
	setKr1();
	setKr2();
	setK_i1();
	setK_i2();

	//Initialize Diffentiator Parameters
	setAA();
	setBB();
	setCC();
	setDD();

	//Initalize Trajectory Planning
	setX0();
	setY0();
	setZ0();
	setXTf();
	setYTf();
	setZTf();

}

void quadSYSspline::process(int t, Eigen::VectorXf current_state){
	std::cout << "Current state size: " << current_state.size() << std::endl;
	this->time = t;
	//Initialize Quadrotor system
	this->current_state = current_state;
	this->zeta << this->current_state(0) , this->current_state(1) , this->current_state(2);
    this->eta << this->current_state(3) , this->current_state(4) , this->current_state(5);
	this->zetaDot << this->current_state(6) , this->current_state(7) , this->current_state(8);
	this->etaDot << this->current_state(9) , this->current_state(10) , this->current_state(11);
	//
	this->phiDot = this->etaDot(0);
	this->thetaDot = this->etaDot(1);
	this->siDot = this->etaDot(2);
	//
	this->phi = this->eta(0);
	this->theta = this->eta(1);
	this->si = this->eta(2);

	//Diff parameters
	this->sd1 << this->current_state(12), this->current_state(13), this->current_state(14);
	this->sd2 << this->current_state(15), this->current_state(16), this->current_state(17);

	//Integration Parameters
	this->r1i << this->current_state(18), this->current_state(19), this->current_state(20);
	this->r2i << this->current_state(21), this->current_state(22), this->current_state(23);
	//Construct NN weight parameter vector
	//TODO make sure the w1col and w2col are being built properly
	this->w1col = Eigen::VectorXf::Zero(nn*3);
	this->w2col = Eigen::VectorXf::Zero(nn*3);
	for(int i=0;i<this->w1col.size();i++){
		this->w1col(i) = this->current_state(24+i);
		this->w2col(i) = current_state(24+nn+i);
	}
	
	w1 = Eigen::MatrixXf::Zero(nn,3);
	w2 = Eigen::MatrixXf::Zero(nn,3);

	//Populating w1 and w2 matricies from the vectors
	for(int i=0;i<nn;i++){
		this->w1(i,0) = this->w1col(i);
		this->w1(i,1) = this->w1col(nn+i-1);
		this->w1(i,2) = this->w1col((2*nn)+i-1);
		this->w2(i,0) = this->w2col(i);
		this->w2(i,1) = this->w2col(nn+i-1);
		this->w2(i,2) = this->w2col((2*nn)+i-1);
	}

	trajectoryPlanning();
	NN1Estimation();
	FdCalc();
	//printParameters();
	
	/*std::cout << "Zeta:\n" << this->zeta << std::endl;
	std::cout << "Eta:\n" << this->eta << std::endl;
	std::cout << "zetaDot:\n" << this->zetaDot << std::endl;
	std::cout << "etaDot:\n" << this->etaDot << std::endl;*/
	std::cout << "Desired Force: \n" << this->Fd << std::endl;
}

void quadSYSspline::trajectoryPlanning(){
	/*double xtf = 20;
	double ytf = 5;
	double ztf = 10;*/
	int tf = 1; 
	int t = this->time;
	std::cout << "Time: " << tf << std::endl;

	//X-axis trajectory planning
	xa3to5 = Eigen::Matrix3f::Zero();
	xa3to5(0,0) = 3;
	xa3to5(1,0) = 6;
	xa3to5(2,0) = 1;
	xa3to5(0,1) = 4*tf;
	xa3to5(1,1) = 12*tf;
	xa3to5(2,1) = tf;
	xa3to5(0,2) = 5*tf*tf;
	xa3to5(1,2) = 20*tf*tf;
	xa3to5(2,2) = tf*tf;
	Eigen::Vector3f xDiff(0,0,(this->xTf-x0)/(tf*tf*tf));

	//y-axis trajectory planning
	ya3to5 = Eigen::Matrix3f::Zero();
	ya3to5(0,0) = 3;
	ya3to5(1,0) = 6;
	ya3to5(2,0) = 1;
	ya3to5(0,1) = 4*tf;
	ya3to5(1,1) = 12*tf;
	ya3to5(2,1) = tf;
	ya3to5(0,2) = 5*tf*tf;
	ya3to5(1,2) = 20*tf*tf;
	ya3to5(2,2) = tf*tf;
	Eigen::Vector3f yDiff(0,0,(this->yTf-y0)/(tf*tf*tf));

	//z-axis trajectory planning
	za3to5 = Eigen::Matrix3f::Zero();
	za3to5(0,0) = 3;
	za3to5(1,0) = 6;
	za3to5(2,0) = 1;
	za3to5(0,1) = 4*tf;
	za3to5(1,1) = 12*tf;
	za3to5(2,1) = tf;
	za3to5(0,2) = 5*tf*tf;
	za3to5(1,2) = 20*tf*tf;
	za3to5(2,2) = tf*tf;
	Eigen::Vector3f zDiff(0,0,(this->zTf-z0)/(tf*tf*tf));

	Eigen::Vector3f xa3to5Vec = xa3to5.inverse()*xDiff;
	Eigen::Vector3f ya3to5Vec = ya3to5.inverse()*yDiff;
	Eigen::Vector3f za3to5Vec = za3to5.inverse()*zDiff;

	float xD = x0 + xa3to5Vec.transpose()*Eigen::Vector3f(t*t*t,t*t*t*t,t*t*t*t*t);
	float yD = y0 + ya3to5Vec.transpose()*Eigen::Vector3f(t*t*t,t*t*t*t,t*t*t*t*t);
	float zD = z0 + za3to5Vec.transpose()*Eigen::Vector3f(t*t*t,t*t*t*t,t*t*t*t*t);

	//Eigen::Vector3f zetaD(xD,yD,zD);
	
	zetaD = Eigen::Vector3f(xD,yD,zD);
	Eigen::Matrix3f zetaD1_term;
	zetaD1_term = Eigen::Matrix3f::Zero();
	zetaD1_term(0,0) = 3*xa3to5Vec(0);
	zetaD1_term(0,1) = 4*xa3to5Vec(1);
	zetaD1_term(0,2) = 5*xa3to5Vec(2);
	zetaD1_term(1,0) = 3*ya3to5Vec(0);
	zetaD1_term(1,1) = 4*ya3to5Vec(1);
	zetaD1_term(1,2) = 5*ya3to5Vec(2);
	zetaD1_term(2,0) = 3*za3to5Vec(0);
	zetaD1_term(2,1) = 4*za3to5Vec(1);
	zetaD1_term(2,2) = 5*za3to5Vec(2);
	Eigen::Vector3f zetaD1_term_2(t*t,t*t*t,t*t*t);
	zetaDDot1 = zetaD1_term * zetaD1_term_2;

	Eigen::Matrix3f zetaD2_term;
	zetaD2_term = Eigen::Matrix3f::Zero();
	zetaD2_term(0,0) = 3*xa3to5Vec(0);
	zetaD2_term(0,1) = 4*xa3to5Vec(1);
	zetaD2_term(0,2) = 5*xa3to5Vec(2);
	zetaD2_term(1,0) = 3*ya3to5Vec(0);
	zetaD2_term(1,1) = 4*ya3to5Vec(1);
	zetaD2_term(1,2) = 5*ya3to5Vec(2);
	zetaD2_term(2,0) = 3*za3to5Vec(0);
	zetaD2_term(2,1) = 4*za3to5Vec(1);
	zetaD2_term(2,2) = 5*za3to5Vec(2);
	Eigen::Vector3f zetaD2_term_2(t*t,t*t*t,t*t*t);
	zetaDDot2 = zetaD2_term * zetaD2_term_2;
}

void quadSYSspline::calculateNNWeightParameters(){
	w1 = Eigen::MatrixXf(nn,3);
	w2 = Eigen::MatrixXf(nn,3);

	//Populating w1 and w2 matricies from the vectors
	for(int i=0;i<nn;i++){
		w1(i,1) = w1col(i);
		w1(i,2) = w1col(nn+i);
		w1(i,3) = w1col((2*nn)+i);
		w2(i,1) = w2col(i);
		w2(i,2) = w2col(nn+i);
		w2(i,3) = w2col((2*nn)+i);
	}
}


//Needs to be checked
void quadSYSspline::NN1Estimation(){
	//One of the variables used to calculate Fd
	error1 = this->zetaD - this->zeta;
	error1Dot = this->zetaDDot1 - this->zetaDot;
	//this->r1 = error1Dot + (lambda1 * error1);

	Eigen::VectorXf inx1_2;
	inx1_2 = Eigen::VectorXf::Zero(15);
	inx1_2 << this->zeta , this->eta , this->zetaDot , this->etaDot , r1;
	//This needs to be checked. Inx1_2 needs to be a column vector
	Eigen::MatrixXf inx1 = V1.transpose()*inx1_2;
	int inx1_size = inx1.size();
	phi1 = Eigen::VectorXf::Zero(inx1_size);
	for(int i=0;i<inx1_size;i++){
		double value = 1/(1+std::exp(-inx1(i)));
		phi1(i) = value;
	}

	this->vNN1 = this->w1.transpose() *  phi1;
}

void quadSYSspline::setW1col(Eigen::VectorXf w1col){
	//w1col = Eigen::VectorXf(nn*3);
	this->w1col = w1col;
}

void quadSYSspline::setW2col(Eigen::VectorXf w2col){
	this->w2col = w2col;
}

void quadSYSspline::setM1(double m){
	this->M1 = m*Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::FdCalc(){
	//Print statements
	//Constants
	/*std::cout << "Constant" << std::endl;
	std::cout << "M1:\n " << this->M1 << std::endl;
	std::cout << "lambda1:\n" << this->lambda1 << std::endl;
	std::cout << "MG:\n" << this->MG << std::endl;
	std::cout << "Kr1:\n" << this->Kr1 << std::endl;
	std::cout << "K_i1:\n" << this->K_i1 << std::endl;
	std::cout << std::endl;
	//Vairables
	std::cout << "Variables" << std::endl;
	std::cout << "zetaDDot2:\n" << this->zetaDDot2 << std::endl;
	std::cout << "error1:\n" << this->error1 << std::endl;
	std::cout << "vNN1:\n" << this->vNN1 << std::endl;
	std::cout << "r1:\n" << this->r1 << std::endl;
	std::cout << "r1i:\n" << this->r1i << std::endl;
	std::cout << std::endl;
	//Used to calculate r1
	std::cout << "Variables used to calculate r1" << std::endl;
	std::cout << "Error: " << this->error1 << std::endl;
	std::cout << "ErrorDot: " << this->error1Dot << std::endl;*/
	
	this->Fd = this->M1*this->zetaDDot2 - this->M1*this->lambda1*this->lambda1*this->error1-this->MG-this->vNN1+this->Kr1*this->r1+this->K_i1*this->r1i;
}

void quadSYSspline::calcFromFd(){
	this->a = -1*this->Fd(1);
	this->b = std::sqrt(std::pow(this->Fd(2),2) + std::pow(this->Fd(3),2));
	this->u = std::sqrt(std::pow(a,2) + std::pow(b,2));
	this->thetaD = std::atan(a/b);
	this->phiD = std::atan(this->Fd(2)/this->Fd(3));
	this->siD = 0;
}

void quadSYSspline::calcJ(){
	this->Ix = this->I(0,0);
	this->Iy = this->I(1,1);
	this->Iz = this->I(2,2);
	this->J(0,0) = (this->Ix*std::pow(std::sin(this->theta),2)) + (this->Iy*std::pow(std::cos(this->theta),2)*std::sin(this->si)) + (this->Iz*(std::pow(std::cos(this->theta),2))*(std::pow(std::cos(this->si),2)));
	this->J(0,1) = (this->Iy-this->Iz)*(std::cos(this->theta))*(std::cos(this->si))*(std::sin(this->si));
	this->J(0,2) = (this->Iz)*(std::sin(this->si));
	this->J(1,0) = (this->Iy-this->Iz)*(std::cos(this->theta))*(std::cos(this->si))*(std::sin(this->si));
	this->J(1,1) = (this->Iy*std::pow(std::cos(this->si),2)) + (this->Iz*std::pow(std::sin(this->si),2));
	this->J(1,2) = 0;
	this->J(2,0) = -this->Iz*std::sin(this->theta);
	this->J(2,1) = 0;
	this->J(2,2) = this->Iz;
}

void quadSYSspline::calcR2(){
	this->etaD << this->phiD , this->thetaD, this->siD;
	this->diffDot1 = (this->AA*this->sd1) + (this->BB*this->etaD);
	this->etaDDot1 = (this->CC*this->sd1) + (this->DD*this->etaD);
	this->diffDot2 = (this->AA*this->sd2) + (this->BB*this->etaD);
	this->e2 = this->etaD - this->eta;
	this->e2Dot = this->etaDDot1 - this->etaDot;
	this->r2 = this->lambda2*this->e2 + this->e2Dot;
}

void quadSYSspline::calcNN2(){
	Eigen::VectorXf inx2_second_term;

	this->V2 = Eigen::MatrixXf::Identity(this->ip2,this->nn);
	inx2_second_term << this->zeta, this->eta, this->zetaDot, this->e2Dot, this->r1, this->r2;
	this->inx2 = this->V2.transpose()*inx2_second_term;
	int inx2_size = this->inx2.cols();
	for(int i=0;i<inx2_size;i++){
		this->phi2(i) = 1/(1+std::exp(-this->inx2(i)));
	}
	this->vNN2 = this->w2.transpose() * this->phi2;
}

void quadSYSspline::calcTao(){
	this->tao = (this->vNN2) + (this->J*this->lambda2*this->r2) - (this->J*this->lambda2*this->lambda2*this->e2) + (this->Kr2*this->r2) + (this->K_i2*this->r2i);
	if(this->time <= 0.1){
		signNum(tao);
		tao = 0.3*tao;
	}
}

void quadSYSspline::calcSystemEqations(){
	Eigen::Vector3f dot_term;
	dot_term << std::pow(this->zetaDot(0),2),std::pow(this->zetaDot(1),2),std::pow(this->zetaDot(2),2);
	//this->Axyz = this->etaDot*dot_term;
	this->Axyz(0) = this->etaDot(0)*dot_term(0);
	this->Axyz(1) = this->etaDot(1)*dot_term(1);
	this->Axyz(2) = this->etaDot(2)*dot_term(2);

	dot_term << std::pow(this->etaDot(0),2),std::pow(this->etaDot(1),2),std::pow(this->etaDot(2),2);
	this->Aeta(0) = this->zetaDot(0)*dot_term(0);
	this->Aeta(1) = this->zetaDot(1)*dot_term(1);
	this->Aeta(2) = this->zetaDot(2)*dot_term(2);

	this->Teta << -std::sin(this->theta) , 0, 1, std::sin(this->theta)*std::cos(this->si), std::cos(this->si), 0, std::cos(this->theta)*std::cos(this->si), -std::sin(this->si), 0;
	this->TetaDot(0,0) =-std::cos(this->theta)*this->thetaDot;
	this->TetaDot(0,1) = 0;
	this->TetaDot(0,2) = 0;
	this->TetaDot(1,0) = (std::cos(this->theta)*std::sin(this->si)*this->siDot) - (std::sin(this->si)*std::sin(this->theta)*this->thetaDot);
	this->TetaDot(1,1) = -std::sin(this->si)*this->siDot;
	this->TetaDot(1,2) = 0;
	this->TetaDot(2,0) = (-std::cos(this->theta)*std::sin(this->si)*this->siDot) - (std::sin(this->theta)*std::cos(this->si)*this->thetaDot);
	this->TetaDot(2,1) = -std::cos(this->si)*this->siDot;
	this->TetaDot(2,2) = 0;

	this->C1 = 2*this->Teta.transpose()*Eigen::Matrix3f::Identity(3,3)*this->TetaDot;

	this->C2(0,0) = 0.5*(0);
	this->C2(0,1) = 0.5*(this->phiDot*((2*this->Ix*std::sin(this->theta)*std::cos(this->theta))-(2*(this->Iy)*std::cos(this->theta)*std::pow(std::sin(this->si),2)*std::sin(this->theta))-(2*this->Iz*std::cos(this->theta)*std::pow(std::cos(this->si),2)*std::sin(this->theta))-(this->thetaDot*(this->Iy-this->Iz)*std::sin(this->theta)*std::cos(this->si)*std::sin(this->si))-(this->siDot*this->Ix*std::cos(this->theta))));
	this->C2(0,2) = 0.5*(this->phiDot*((2*this->Iy*std::pow(std::cos(this->theta),2)*std::sin(this->si)*std::cos(this->si))-(2*this->Iz*std::pow(std::cos(this->theta),2)*std::cos(this->si)*std::sin(this->si))-(this->thetaDot*(this->Iy-this->Iz)*std::cos(this->theta)*std::pow(std::sin(this->si),2))+(this->thetaDot*(this->Iy-this->Iz)*std::cos(this->theta)*std::pow(std::cos(this->si),2))));
	this->C2(1,0) = 0.5*(0);
	this->C2(1,1) = 0.5*(-(this->phiDot)*(this->Iy-this->Iz)*std::sin(this->theta)*std::cos(this->si)*std::sin(this->si));
	this->C2(1,2) = 0.5*((-(this->phiDot)*(this->Iy-this->Iz)*std::cos(this->theta)*std::pow(std::sin(this->si),2))+(this->phiDot*(this->Iy-this->Iz)*std::cos(this->theta)*std::pow(std::cos(this->si),2))+(this->thetaDot*(-2*this->Iy*std::cos(this->si)*std::sin(this->si)))+(2*this->Iz*std::sin(this->si)*std::cos(this->si)));
	this->C2(2,0) = 0.5*(0);
	this->C2(2,1) = 0.5*(-this->phiDot*this->Ix*std::cos(this->theta));
	this->C2(2,2) = 0.5*(0);

	this->Ceta = this->C1-this->C2;

	Eigen::Vector3f zetaDot2_second_term;
	zetaDot2_second_term(0) = -this->u*std::sin(this->theta);
	zetaDot2_second_term(1) = this->u*std::cos(this->theta)*std::sin(this->phi);
	zetaDot2_second_term(2) = this->u*std::cos(this->theta)*std::cos(this->phi+this->MG(2));
	this->zetaDot2 = this->M1.inverse()*zetaDot2_second_term + this->Axyz;
	this->etaDot2 = this->J.inverse()*(this->tao-this->Ceta*this->etaDot+this->Aeta);
}

void quadSYSspline::calcNNDynamics(){

	//ww1
	this->r << this->r1 , this->r2;
	this->norm_r = this->r.norm();
	this->wDot1 = (this->F1*this->phi1*this->r1.transpose())-(this->kali1*this->norm_r*this->F1*this->w1);
	for(int i = 0;i<this->w1Dot.cols();i++){
		this->w1Dot << this->wDot1.col(i);
	}

	//ww2
	this->wDot2 = this->F2*this->phi2*this->r2.transpose()-this->kali2*this->norm_r*this->F2*this->w2;
	for(int i = 0;i<this->wDot2.cols();i++){
		this->w2Dot << this->wDot2.col(i);
	}

	this->wDot << this->wDot1 , this->wDot2;
}

void quadSYSspline::buildSDot(){
	this->sDot << this->zetaDot , this->etaDot, this->zetaDot2, this->etaDot2, this->diffDot1, this->diffDot2, this->r1, this->r2, this->wDot;
}

Eigen::VectorXf quadSYSspline::getSDot(){
	return this->sDot;
}

//Set mass gravity matrix for quadrotor parameters
void quadSYSspline::setMG(double mass,double gravity){
	this->MG(0) = 0;
	this->MG(1) = 0;
	this->MG(2) = -mass*gravity;
	//std::cout << this->mg << std::endl;
}

void quadSYSspline::setI(double constant){
	I = constant * Eigen::Matrix3f::Identity(3,3);
}

void quadSYSspline::setLambda1(double constant){
	this->lambda1 = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setLambda2(double constant){
	this->lambda2 = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setKr1(double constant){
	this->Kr1 = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setKr2(double constant){
	this->Kr2 = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setK_i1(double constant){
	this->K_i1 = constant;
}

void quadSYSspline::setK_i2(double constant){
	this->K_i2 = constant;
}

void quadSYSspline::setAA(double constant){
	this->AA = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setBB(double constant){
	this->BB = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setCC(double constant){
	this->CC = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setDD(double constant){
	this->DD = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setNN(double constant){
	this->nn = constant;
}

void quadSYSspline::setIp1(double constant){
	this->ip1 = constant;
}

void quadSYSspline::setIp2(double constant){
	this->ip2 = constant;
}

void quadSYSspline::setOp(double constant){
	this->op = constant;
}

void quadSYSspline::setF1(double constant){
	F1 << (constant * Eigen::MatrixXf::Identity(3,3));
}

void quadSYSspline::setF2(double constant){
	this->F2 = constant * Eigen::MatrixXf::Identity(3,3);
}

void quadSYSspline::setKali1(double constant){
	this->kali1 = constant;
}

void quadSYSspline::setKali2(double constant){
	this->kali2 = constant;
}

void quadSYSspline::setX0(double constant){
	this->x0 = constant;
}

void quadSYSspline::setY0(double constant){
	this->y0 = constant;
}

void quadSYSspline::setZ0(double constant){
	this->z0 = constant;
}

void quadSYSspline::setXTf(double constant){
	this->xTf = constant;
}

void quadSYSspline::setYTf(double constant){
	this->yTf = constant;
}

void quadSYSspline::setZTf(double constant){
	this->zTf = constant;
}

void quadSYSspline::printParameters(){
	std::cout << "NN Parameters" << std::endl;
	std::cout << "\tNN: " << this->nn << std::endl;
	std::cout << "\tip1: " << this->ip1 << std::endl;
	std::cout << "\tip2: " << this->ip2 << std::endl;
	std::cout << "\top: " << this->op << std::endl;
	std::cout << "\tF1: " << std::endl << this->F1 << std::endl;
	std::cout << "\tF2: " << std::endl << this->F2 << std::endl;
	std::cout << "\tKail1: " << this->kali1 << std::endl;
	std::cout << "\tKail2: " << this->kali2 << std::endl;

	std::cout << std::endl << "Platform Parameters" << std::endl;
	std::cout << "\t-mg: " << std::endl << this->MG << std::endl;

	std::cout << "\nProportional and Integral Gains" << std::endl;
	std::cout << "\tLamda1: " << std::endl << this->lambda1 << std::endl;
	std::cout << "\tLamda2: " << std::endl << this->lambda2 << std::endl;
	std::cout << "\tKr1: " << std::endl << this->Kr1 << std::endl;
	std::cout << "\tKr2: " << std::endl << this->Kr2 << std::endl;
	std::cout << "\tK_i1: " << this->K_i1 << std::endl;
	std::cout << "\tK_i2: " << this->K_i2 << std::endl;

	std::cout << "\nDifferentiator Parameters" << std::endl;
	std::cout << "\tAA: " << std::endl << this->AA << std::endl;
	std::cout << "\tBB: " << std::endl << this->BB << std::endl;
	std::cout << "\tCC: " << std::endl << this->CC << std::endl;
	std::cout << "\tDD: " << std::endl << this->DD << std::endl;

	std::cout << "\nTrajectory Planning Parameters " << std::endl;
	std::cout << "\tX0: " << this->x0 << std::endl;
	std::cout << "\tY0: " << this->y0 << std::endl;
	std::cout << "\tZ0: " << this->z0 << std::endl;
	std::cout << "\tXTf: " << this->xTf << std::endl;
	std::cout << "\tYTf: " << this->yTf << std::endl;
	std::cout << "\tZTf: " << this->zTf << std::endl;

}

void quadSYSspline::signNum(Eigen::Vector3f& x){
	for(int i=0;i<3;i++){
		double value = x(i);
		if(value < 0)x(i) = -1;
		if(value == 0)x(i) = 0;
		if(value > 1)x(i) = 1;
	}
}

/*int main(){
	quadSYSspline test;
	std::cout << "Hello World!!" << std::endl;
	test.printParameters();
	return 0;
}*/
