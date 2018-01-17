/*
 *  Copyright (c) 2018 Julian Soeren Lorenz, Carnegie Mellon University, All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *	following conditions are met:
 *
 *   	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *   	   following disclaimer.
 *   	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   	   following disclaimer in the documentation and/or other materials provided with the distribution.
 *   	3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *   	   products derived from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *	INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *	OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	END OF LICENSE
 *
 *	Author: Julian Soeren Lorenz
 *	Email:  JulianLorenz@live.de
 *
*/

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "kalmanfilter.h"

#ifdef _DEBUG
using namespace std;
#endif

using namespace Eigen;

/**
 * Extended Kalman Filter
 *  SL - Length of state s
 *  ML - Length of measurement z
 *  IL - Length of input vector u. If you don't use the input vector, ignore the value
 *
 *  To use it, you have to subclass this class and implement the Methods:
 *  	predictState - correspondes to x_p[k] = f(x[k-1], u[k])
 * 	 	setPhiGamma - sets the matrices necessary to predict the state covariance (have a look at the algorithm description)
 *  	setR - Set the measurement error covariance R
 *  	setQ - Set the model error covariance Q
 * 		predictMeasurement - insert implementation for z_p[k] = h(x_p[k])
 *  	setH - Set the observation matrix H - typically the Jacobian of h(x_p[k])
 *  	setZ - optional, in case measurement vector has to be modified
 *  For use in main loop:
 *  		1a) Call addMeasurement(z[k],u[k]) to do one quiet loop without output
 * 			1b) Call x[k] = getValue(z[k],u[k]) to do a loop with output
 * 			[ 2) use getValue() to get the estimate of the last estimation step ] 
 *		Alternative for use of Filter class (deprecated):   
 *  		1) Call setInput(u[k])
 * 			2a) Call addMeasurement(z[k]) to do one loop without direct output
 *			2b) Call x[k] = getValue(z[k]) to do one loop and receive an output
 *     		[3) use getValue() to get the estimate of the last estimation step ] 
 *
 * 
 *  The Extended Kalman Filter implements the following algorithm:
 *
 *	Variables:
 * 		x[k] -- estimation of loop k
 * 		x_p[k] -- prediction of loop k
 * 		u[k] -- input of loop k
 * 		f(x[k-1], u[k]) -- non-linear system model
 * 		h(x_p[k]) -- non-linear observation model
 * 		P[k] -- state error covariance
 * 		P_p[k] -- prediction of state error covariance
 * 		phi -- State transition matrix 
 * 		gamma -- Process noise input matrix
 * 		Q[k] -- model noise matrix of loop k
 * 		R[k] -- measurement noise matrix of loop k
 * 		K[k] -- Kalman gain of loop k
 * 		H[k] -- observation matrix of loop k
 * 		z[k] -- measurement of loop k
 *
 *  Predict next state and the error covariance
 *  	x_p[k] = f(x[k-1], u[k])
 *  	P_p[k] = phi * P[k-1] * phi^T + gamma*Q[k]*gamma^T
 *  Compute the Kalman gain
 *  	K[k] = P_p[k] * H[k]^T / (H[k] * P_p[k] * H[k]^T + R[k])
 *  Estimate state with measurement z
 *  	x[k] = x_p[k] + K * ( z[k] - h(x_p[k]))
 *  Compute the error covariance
 *		P[k] = (I - K[k] * H[k]) * P_p[k]
 *
 *
 *
 *	The following functions are called each step in this order:
 *  	1. predictState()
 *      2. predictCovariance()
 *      	2a. setQ()
 *			2b. setPhiGamma()
 * 		3. computeKalmanGain()
 *		4. estimate()
 *			4a. setZ()
 * 			4b. predictMeasurement()
 * 		5. computeErrorCovariance()
 */

template <int SL, int ML=SL, int IL=1>
class ExtendedKalmanFilter : public KalmanFilter<SL,ML,IL>
{

public:
	ExtendedKalmanFilter();

protected:
	Matrix<double,SL,SL> phi;
	Matrix<double,SL,SL> gamma;
	virtual void predictState() = 0; //Function f(x[k-1],u[k]) = x_p[k] 
	void predictCovariance();
	virtual void setPhiGamma() = 0;
	virtual Matrix<double,ML,1> predictMeasurement() = 0; //Function h(x_p[k]) = z_p[k]
	void estimate(Matrix<double,ML,1> newMeasurement);
	void setA(){return;} //A*x[k-1] is replaced with f(x[k-1],u[k])
    virtual void setR() = 0;
    virtual void setQ() = 0;
    virtual void setH() = 0;
protected:
};

#endif
