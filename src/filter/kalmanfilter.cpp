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

#include "kalmanfilter.h"

template <int SL,int ML,int IL>
KalmanFilter<SL,ML,IL>::KalmanFilter()
{
	//Initialize to zero
	x = StateType::Zero(SL);
	P = Matrix<double,SL,SL>::Identity();
	P_pred = Matrix<double,SL,SL>::Identity();
	u = InputType::Zero();
	Q = Matrix<double,SL,SL>::Identity();
	R = Matrix<double,ML,ML>::Identity();

	processingTime = 0;
	delta_time_s = 0;
	dt = 0;
	delta_time_ms = 0;
    timer.reset();
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::initialize(StateType initialState){
	x = initialState;
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::addMeasurement(MeasurementType newMeasurement, InputType input){
	setInput(input);
	addMeasurement(newMeasurement);
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::addMeasurement(MeasurementType newMeasurement){
    delta_time_s = timer.elapsed();
    dt = delta_time_s;
    delta_time_ms = ((double)delta_time_s)*1000;
    timer.reset();

	processingTimer.reset();
    //Predict
    predictState();
	predictCovariance();
    computeKalmanGain();
    //Update
    estimate(newMeasurement);
    computeErrorCovariance();

	processingTime = processingTimer.elapsed();
}

template <int SL,int ML,int IL>
Matrix<double,SL,1> KalmanFilter<SL,ML,IL>::getValue(){
    return x;
}

template <int SL,int ML,int IL>
Matrix<double,SL,1> KalmanFilter<SL,ML,IL>::getValue(MeasurementType newMeasurement){
    addMeasurement(newMeasurement);
    return getValue();
}

template <int SL,int ML,int IL>
Matrix<double,SL,1> KalmanFilter<SL,ML,IL>::getValue(MeasurementType newMeasurement, InputType input){
	setInput(input);
    addMeasurement(newMeasurement);
    return getValue();
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::predictState(){
    setA();
    setB();
    //x_p[k] = A * x[k-1] + B * u[k]
    x_pred = A*x + B*u;
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::predictCovariance(){
    //P_p[k] = A * P[k-1] * A^T + Q[k]
    setQ();
    P_pred = A*P*A.transpose() + Q;
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::computeKalmanGain(){
	setR();
	setH();
	#ifdef _DEBUG
		cout << "R[k]:" << endl << R << endl;
	#endif
	//K[k] = P_p[k] * H[k]^T / (H[k] * P_p[k] * H[k]^T + R[k])
    K = P_pred*H.transpose()*(H*P_pred*H.transpose()+R).inverse();
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::estimate(MeasurementType z){
    z = setZ(z);
    //x[k] = x_p[k] + K * ( z[k] - H[k] * x_p[k])
    x = x_pred + K*(z-H*x_pred);
}

template <int SL,int ML,int IL>
void KalmanFilter<SL,ML,IL>::computeErrorCovariance(){
	//P[k] = (I - K[k] * H[k]) * P_p[k]
    P = (Matrix<double,SL,SL>::Identity() - K*H)*P_pred;
}

template class KalmanFilter<2>;
template class KalmanFilter<6>;
template class KalmanFilter<4>;
template class KalmanFilter<4,2>;
template class KalmanFilter<6,6,3>;
template class KalmanFilter<5,3,4>;
template class KalmanFilter<7,4,6>;
