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

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "filter.h"
#include "../util/ElapsedTimer.h"

#include <Eigen/Core>
#include <Eigen/LU>


#ifdef _DEBUG
	#include <iostream>
	using namespace std;
#endif

using namespace Eigen;

/**
 * Simple Kalman Filter
 *  SL - Length of state s
 *  ML - Length of measurement z
 *  IL - Length of input vector u. If you don't use the input vector, ignore the value
 *
 *  To use it, you have to subclass this class and implement the Methods:
 *  setA - Set the system model matrix A
 *  setB - Set the input matrix B. If you dont use it, set it to zero
 *  setR - Set the measurement error covariance R
 *  setQ - Set the model error covariance Q
 *  setH - Set the measurement matrix H
 *  setZ - optional, in case measurement vector has to be modified
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
 *  The Kalman Filter implements the following algorithm:
 *
 *	Variables:
 * 		x[k] -- estimation of loop k
 * 		x_p[k] -- prediction of loop k
 * 		u[k] -- input of loop k
 * 		P[k] -- state error covariance
 * 		P_p[k] -- prediction of state error covariance
 * 		A -- model matrix 
 * 		B -- input matrix
 * 		Q -- model noise matrix
 * 		R -- measurement noise matrix
 * 		K[k] -- Kalman gain of loop k
 * 		H[k] -- observation matrix of loop k
 * 		z[k] -- measurement of loop k
 *
 *  Predict next state and the error covariance
 *  	x_p[k] = A * x[k-1] + B * u[k]
 *  	P_p[k] = A * P[k-1] * A^T + Q[k]
 *  Compute the Kalman gain
 *  	K[k] = P_p[k] * H[k]^T / (H[k] * P_p[k] * H[k]^T + R[k])
 *  Estimate state with measurement z
 *  	x[k] = x_p[k] + K * ( z[k] - H[k] * x_p[k])
 *  Compute the error covariance
 *		P[k] = (I - K[k] * H[k]) * P_p[k]
 *
 *
 *	The following functions are called each step in this order:
 *  	1. predictState()
 * 			1a. setA()
 * 			2a. setB()
 *      2. predictCovariance()
 *      	2a. setQ()
 * 		3. computeKalmanGain()
 *		4. estimate()
 *			4a. setZ()
 * 			4b. predictMeasurement()
 * 		5. computeErrorCovariance()
 */
template <int SL, int ML=SL, int IL=1>
class KalmanFilter: public Filter<Matrix<double,SL,1>,Matrix<double,ML,1>>
{

public:
	typedef Matrix<double, SL, 1> StateType;
	typedef Matrix<double, IL, 1> InputType;
	typedef Matrix<double, ML, 1> MeasurementType;
	typedef Matrix<double, SL, SL> StateCovarianceType;
    KalmanFilter();
    //virtual ~KalmanFilter();
	StateType getValue(MeasurementType newMeasurement, InputType input);
    StateType getValue(MeasurementType newMeasurement);
    StateType getValue();
    void addMeasurement(MeasurementType newMeasurement);
	void addMeasurement(MeasurementType newMeasurement, InputType input);
    virtual inline void setInput(InputType input){this->u = input;}
    inline double getFrequency(){return 1.0/delta_time_s;}
	inline double getProcessingTime(){return processingTime;}
	inline Matrix<double,SL,SL> getStateCovariance(){return P;}
	void initialize(StateType initialState);
protected:
	double processingTime;
    double delta_time_ms;
    double delta_time_s;
    double dt; ///same as delta_time_s - just easier to use in a matrix
    InputType u;		//u[k]
    StateType x_pred;	//x_p[k]
    StateType x;	//x[k]
    Matrix<double, SL, SL> A; //Model matrix
    Matrix<double, SL, IL> B; //Input matrix
    Matrix<double, ML, ML> R; //Measurement covariance
    Matrix<double, SL, SL> Q; //environmental covariance
    Matrix<double, ML, SL> H; //Measurement matrix
    Matrix<double, SL, ML> K;      //K[k]
    Matrix<double, SL, SL> P_pred; //P_p[k] predicted state error covariance
    Matrix<double, SL, SL> P; //P[k] updated, state error covariance
    virtual void predictState();
	virtual void predictCovariance();
    void computeKalmanGain();
    virtual void estimate(MeasurementType newMeasurement);
    virtual void computeErrorCovariance();
    virtual void setA() = 0;
    virtual void setB(){B = Matrix<double,SL,IL>::Zero();}
    virtual void setR() = 0;
    virtual void setQ() = 0;
    virtual void setH() = 0;
    virtual inline MeasurementType setZ(MeasurementType z){return z;} //Leave option to modify measurement vector
private:
    ElapsedTimer timer;		//Measures the elapsed time since the last call
	ElapsedTimer processingTimer; //Measures the calculation time for one step
};

#endif // KALMANFILTER_H
