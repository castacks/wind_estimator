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

#include "cekfilter.h"

CEKFilter::CEKFilter(){
	x(3) = 0.6125;
	P(0,0) = 25;
	P(1,1) = 25;
	P(2,2) = 8;
	P(3,3) = 1e-3;
	P(4,4) = 12;
	P(5,5) = 12;
	P(6,6) = 12;
	useIMUInput = true;

	//Initialize variances
	modelVarWindH = 0.01;
	modelVarWindV = 0.01;
	modelVarEta = 0.00000001;
	modelVarAirspeedLon = 0.1;
	modelVarAirspeedSideUp = 0.1;

	measurementVarGroundspeedH = 1;
	measurementVarGroundspeedV = 1;
	measurementVarPressure = 4;
}

//=========================================================================
// Interface functions
//=========================================================================
void CEKFilter::addMeasurement(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Vector3d eulerAngles){
	setInput(u,eulerAngles);
	KalmanFilter::addMeasurement(measurement);
}

void CEKFilter::addMeasurement(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Quaternion<double> q){
	setInput(u,q);
	KalmanFilter::addMeasurement(measurement);
}

Matrix<double,7,1> CEKFilter::getValue(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Vector3d eulerAngles){
	setInput(u,eulerAngles);
	return KalmanFilter::getValue(measurement);
}

Matrix<double,7,1> CEKFilter::getValue(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Quaternion<double> q){
	setInput(input,q);
	return KalmanFilter::getValue(measurement);
}

/*
	Sets the u u for the next iteration and computes the rotation matrix from Euler angles
*/
void CEKFilter::setInput(Matrix<double,6,1> input, Vector3d eulerAngles){
	KalmanFilter::setInput(input);
	R_g2f = Util::calcRotationMatrix(eulerAngles(0), eulerAngles(1), eulerAngles(2));
	R_f2g = R_g2f.transpose();
}

/*
	Sets the u u for the next iteration and computes the rotation matrix from quaternions
*/
void CEKFilter::setInput(Matrix<double,6,1> input, Quaternion<double> q){
	KalmanFilter::setInput(input);
	R_g2f = Util::calcRotationMatrix(q);
	R_f2g = R_g2f.transpose();
}

//=========================================================================
// Model functions
//=========================================================================

Matrix<double,7,1> CEKFilter::calcRungeKutta_f(Matrix<double,7,1> x){
	Matrix<double,7,1> result = MatrixXd::Zero(7,1);

	Vector3d omega = u.head(3);
	Vector3d acc_f = u.tail(3);
	Vector3d airspeed_f = x.tail(3);


	result.tail(3) = airspeed_f.cross(omega) + acc_f;
	return result;
}

/**
 * Predicts next state x_p[k] based on our model, the last x and the u vector.
 *
 *  Model:
 * 		x_p[k] = f(x[k-1],u[k]) :
 * 			dot(Vw) = 0
 * 			dot(eta) = 0
 * 			dot(Va) = a_f - omega x Va
*/
void CEKFilter::predictState(){
	x_pred.head(4) = x.head(4); //Windspeed and correction factor eta stay the same.

	if(!useIMUInput){
		u << 0,0,0,0,0,0;
		x_pred = x;
	}else{

		Vector3d gravity(0,0,-9.8052);
		Vector3d acc_f = u.tail(3) - R_g2f*gravity;   //Acceleration in body coordinates, gravity has to be substracted
		u.tail(3) = acc_f;

		x_pred = x;

		int steps = 4;
		double h = dt/steps;

		for(int i = 0; i<steps; i++){

			Matrix<double,7,1> rk1 = calcRungeKutta_f(x_pred);
			Matrix<double,7,1> rk2 = calcRungeKutta_f(x_pred + (h/2)*rk1);
			Matrix<double,7,1> rk3 = calcRungeKutta_f(x_pred + (h/2)*rk2);
			Matrix<double,7,1> rk4 = calcRungeKutta_f(x_pred + h*rk3);

			x_pred = x_pred + (h/6)*(rk1 + 2*rk2 + 2*rk3 + rk4);

		}
		#ifdef _DEBUG
			cout << "acc_f:" << endl << acc_f << endl;
			cout << "omega:" << endl << u.head(3) << endl;
		#endif
	}
#ifdef _DEBUG
	cout << "x_p[k]:" << endl << x_pred << endl;
#endif
}

/**
	Computes matrices phi and gamma which are necessary to predict the state error covariance
*/
void CEKFilter::setPhiGamma(){
    double theta = ((u.head(3)).norm())*dt;
    if (theta<1.0e-12) // This case corresponds to (p,q,r)=0
    {
        phi=MatrixXd::Identity(7,7);
        gamma=dt*MatrixXd::Identity(7,7);
    }
    else // For every case (p,q,r)!=0
    {
        Matrix3d F_sub;
		F_sub <<    0,  u(2), -u(1),
    			-u(2), 	   0,  u(0),
    			 u(1), -u(0),     0;

        phi.bottomRightCorner(3,3) = Matrix3d::Identity(3, 3) + (sin(theta)*dt/theta)*F_sub + (1-cos(theta))*dt*dt*F_sub*F_sub/(theta*theta);

        gamma.bottomRightCorner(3,3) = dt*Matrix3d::Identity(3, 3) + (1-cos(theta))*dt*dt*F_sub/(theta*theta) + (1-sin(theta)/theta)*dt*dt*dt*F_sub*F_sub/(theta*theta);
    }
}

/**
 *	Predicts the next measurement z_p[k] = h(x_p[k]) based on the observation model
 *			Vk = Vw + R_f2g*Va
 * 			dP = eta * Vax^2 
*/
Vector4d CEKFilter::predictMeasurement(){
	Vector4d predMeasurement;
	predMeasurement.head(3) = x_pred.head(3) + R_f2g*x_pred.tail(3); //Vg = Vw + R*Va
	predMeasurement(3) = x_pred(3)*pow(x_pred(4),2); //dP = eta * Vax^2
	
	#ifdef _DEBUG
		cout << "z_p[k]" << endl << predMeasurement << endl;
	#endif
	return predMeasurement;
}

/**
	Computes the Jacobian matrix H = d*h(x_p[k]) / d*x_p[k]
*/
void CEKFilter::setH(){
	double eta = x_pred(3);
	double Vax = x_pred(4);
	H << 1, 0, 0, 0, R_f2g(0,0), R_f2g(0,1), R_f2g(0,2),
		 0, 1, 0, 0, R_f2g(1,0), R_f2g(1,1), R_f2g(1,2),
		 0, 0, 1, 0, R_f2g(2,0), R_f2g(2,1), R_f2g(2,2),
		 0, 0, 0, pow(Vax,2), 2*eta*Vax, 0, 0;
}

/**
	Sets the measurement noise matrix
*/
void CEKFilter::setR(){
	R.diagonal() << measurementVarGroundspeedH,
					measurementVarGroundspeedH,
					measurementVarGroundspeedV,
					measurementVarPressure;
	//R = dt*R;
}

/**
	Sets the model noise matrix
*/
void CEKFilter::setQ(){
	Q.diagonal() << modelVarWindH,
					modelVarWindH,
					modelVarWindV,
					modelVarEta,
					modelVarAirspeedLon,
					modelVarAirspeedSideUp,
					modelVarAirspeedSideUp;
}

void CEKFilter::estimate(MeasurementType z){
	ExtendedKalmanFilter::estimate(z);
	setWind(x.head(3));
}

void CEKFilter::computeErrorCovariance(){
	P = (MatrixXd::Identity(7, 7) - K*H)*P_pred*((MatrixXd::Identity(7, 7)-K*H).transpose()) + K*R*(K.transpose());
	setWindSigmaWithVariance(P.diagonal().head(3));
}

