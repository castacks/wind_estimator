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

#include "extendedkalmanfilter.h"

template <int SL,int ML,int IL>
ExtendedKalmanFilter<SL,ML,IL>::ExtendedKalmanFilter(){
    phi = MatrixXd::Identity(SL, SL); // State transition matrix
    gamma = MatrixXd::Identity(SL, SL); // Process noise input matrix
}

template <int SL,int ML,int IL>
void ExtendedKalmanFilter<SL,ML,IL>::predictCovariance(){
    //P_p[k] = Phi * P[k-1] * Phi^T + Gamma*Q[k]*Gamma^T
	setQ();
    setPhiGamma();
	Matrix<double,SL,SL> phi_transposed = phi.transpose();
	Matrix<double,SL,SL> gamma_transposed = gamma.transpose();

    this->P_pred = (phi*(this->P)*phi_transposed) + (gamma*(this->Q)*gamma_transposed);

	#ifdef _DEBUG
		cout << "Q[k]:" << endl << this->Q << endl;
		cout << "Phi[k]:" << endl << phi << endl;
		cout << "Gamma[k]:" << endl << gamma << endl;
		cout << "P_p[k]:" << endl << this->predErrorCovariance << endl;
	#endif
}

template <int SL,int ML,int IL>
void ExtendedKalmanFilter<SL,ML,IL>::estimate(Matrix<double,ML,1> z){
	z = this->setZ(z); //Does do nothing if not overridden
	#ifdef _DEBUG
		cout << "R: " << endl << this->R << endl;
		cout << "H: " << endl << this->H << endl;
		cout << "K[k]:" << endl << this->kalmanGain << endl;
	#endif
    //x[k] = x_p[k] + K * ( z[k] - h(x_p[k]))
    this->x = this->x_pred + this->K*(z- predictMeasurement());
}

template class ExtendedKalmanFilter<7,4,6>;
template class ExtendedKalmanFilter<5,3,4>;
