#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/**
    	  * RMSE Calculation.
	*/
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	//input vector length
	int inpv_ln = estimations.size();
	//check the size of input vectors and return zero RMSE if invalid input
	if( inpv_ln == 0 || inpv_ln != ground_truth.size()){
		return rmse;
	}
	//accumulate the difference squares
	for( int i=0; i < inpv_ln; i++){
		VectorXd df = estimations[i] - ground_truth[i];
		// element by element multiplication
		df = df.array() * df.array();
		rmse += df;
	}
	// mean
	rmse = rmse/inpv_ln;
	//square root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	 * Calculate a Jacobian here.
	 */
	MatrixXd Hj = MatrixXd::Zero(3, 4);
	//get state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	
	
	// precalculate common values
	double pxy_sq = (px * px) + (py * py);
	double pxy_sqrt = sqrt(pxy_sq);
    
    //check division by zero
	if(px == 0 && py == 0)
		return Hj;

	//Compute the zacobina matrix
	Hj(0,0) = px/pxy_sqrt;
	Hj(0,1) = py/pxy_sqrt;
	Hj(2,2) = Hj(0,0);
	Hj(2,3) = Hj(0,1);
	Hj(1,0) = -py/pxy_sq;
	Hj(1,1) = px/pxy_sq;
	Hj(2,0) = py*(vx*py - vy*px)/(pxy_sq * pxy_sqrt);
	Hj(2,1) = px*(vy*px - vx*py)/(pxy_sq * pxy_sqrt);

	return Hj;
}
