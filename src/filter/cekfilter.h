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

#ifndef CEKF_FILTER_H
#define CEKF_FILTER_H

#include "extendedkalmanfilter.h"
#include "WindFilter.h"
#include "../util/MathUtil.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#define _DEBUG
#ifdef _DEBUG
using namespace std;
#endif

using namespace Eigen;

/**
 *  Calibrating EKF. Used to estimate wind, airspeed and a calibration factor eta for an uncalibrated pitot tube.
 *  For more details check:
 *  	1) 	A. Cho, J. Kim, S. Lee, and C. Kee. Wind estimation and airspeed calibration using a uav
 *  		with a single-antenna gps receiver and pitot tube. IEEE Transactions on Aerospace and
 * 			Electronic Systems, 47(1):109–117, January 2011
 * 		2)  T. A. Johansen, A. Cristofaro, K. Sørensen, J. M. Hansen, and T. I. Fossen. On estimation
 *			of wind velocity, angle-of-attack and sideslip angle of small uavs using standard sensors.
 *			In 2015 International Conference on Unmanned Aircraft Systems (ICUAS), pages 510–519, June 2015.
 *  
 *  state x[k] = [Vwx, Vwy, Vwz, eta, Vax, Vay, Vaz] = [Vw, eta, Va]
 *  measurement z[k] = [Vkx, Vky, Vkz, dP] = [Vk, dP]
 *  input u[k] = [p,q,r,a_fx,a_fy,a_fz] = [omega,a_f]
 *
 *  Variables:
 *  	Vw = [Vwx, Vwy, Vwz] -- Wind speed [m/s] in earth frame
 *  	Va = [Vax, Vay, Vaz] -- Airspeed [m/s] in body frame
 *  	Vk = [Vkx, Vky, Vkz] -- Groundspeed [m/s] in earth frame
 *  	eta -- calibration factor for pitot tube. dP = eta*Vax^2. Unit in [kg/m^3]
 *  	dP -- Differential pressure = dynamic pressure - static pressure . Unit in [N/m^2]
 *  	omega = [p,q,r] -- Angular turnrates [rad/s]
 *  	a_f = [a_fx,a_fy,a_fz] -- accelerations from IMU in body frame [m/s^2]. Gravity is not corrected.
 * 		x[k] = estimation of loop k
 * 		x_p[k] = prediction of loop k
 * 		z[k] = measurement of loop k
 * 		z_p[k] = exspected measurement based on prediction x_p[k]
 *  
 *  Model:
 * 		x_p[k] = f(x[k-1],u[k]) : 
 * 			dot(Vw) = 0
 * 			dot(eta) = 0
 * 			dot(Va) = a_f - omega x Va
 *
 *	Observation:
 * 		z_p[k] = h(x_p[k])
 *			Vk = Vw + R_f2g*Va
 * 			dP = eta * Vax^2 
 *  Jacobian matrix H = d*h(x_p[k]) / d*x_p[k]
 *  				  = 1 0 0 0     R_f2g(1,1) R_f2g(1,2) R_f2g(1,3)
 *						0 1 0 0     R_f2g(2,1) R_f2g(2,2) R_f2g(2,3)
 *						0 0 1 0     R_f2g(3,1) R_f2g(3,2) R_f2g(3,3)
 * 						0 0 0 Vax^2 2*eta*Vax  0          0
 *
 *  For use in main loop:
 * 		With Euler angles:
 * 			1a) Call void addMeasurement(z[k],u[k],attitude) - attitude = [roll,pitch,yaw] in radians or a quaternion
 * 			1b) Call x[k] = getValue(z[k],u[k],attitude) - attitude = [roll,pitch,yaw] in radians or a quaternion
 *
 *		Deprecated:
 *		  	1) Call setInput(u[k],attitude) - attitude = [roll,pitch,yaw] in radians or a quaternion
 *	 		2a) Call addMeasurement(z[k]) to do one loop without direct output
 *			2b) Call x[k] = getValue(z[k]) to do one loop and receive an output
 *		 	[3) use getValue(void) to get the estimate of the last estimation step]
*/

class CEKFilter : public ExtendedKalmanFilter<7,4,6>, public WindFilter
{
public:
	CEKFilter();

	// Filter functions
	void setInput(Matrix<double,6,1> input, Vector3d eulerAngles);
	void setInput(Matrix<double,6,1> input, Quaternion<double> q);
	void addMeasurement(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Vector3d eulerAngles);
	void addMeasurement(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Quaternion<double> q);
	Matrix<double,7,1> getValue(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Vector3d eulerAngles);
	Matrix<double,7,1> getValue(Matrix<double,4,1> measurement, Matrix<double,6,1> input, Quaternion<double> q);

	// Setter
	inline void setUseIMUInput(bool _p){ useIMUInput = _p;}
	inline void setModelVarWind(double _H, double _V){modelVarWindH = _H; modelVarWindV = _V;}
	inline void setModelVarWindH(double _p){modelVarWindH = _p;}
	inline void setModelVarWindV(double _p){modelVarWindV = _p;}
	inline void setModelVarEta(double _p){modelVarEta = _p;}
	inline void setModelVarAirspeed(double _lon, double _sideUp){modelVarAirspeedLon=_lon;modelVarAirspeedSideUp=_sideUp;}
	inline void setModelVarAirspeedLon(double _p){modelVarAirspeedLon = _p;}
	inline void setModelVarAirspeedSideUp(double _p){modelVarAirspeedSideUp = _p;}
	inline void setMeasurementVarGroundspeed(double _H,double _V){measurementVarGroundspeedH=_H;measurementVarGroundspeedV=_V;}
	inline void setMeasurementVarGroundspeedH(double _p){measurementVarGroundspeedH = _p;}
	inline void setMeasurementVarGroundspeedV(double _p){measurementVarGroundspeedV = _p;}
	inline void setMeasurementVarPressure(double _p){measurementVarPressure = _p;}

protected:
	void predictState();
	Vector4d predictMeasurement();
	void estimate(Matrix<double,4,1> newMeasurement);
	void computeErrorCovariance();
	void setPhiGamma();
	void setR();
	void setQ();
	void setH();

private:
	Matrix<double,7,1> calcRungeKutta_f(Matrix<double,7,1> x);

	// Flags
	bool useIMUInput;

	// Model variances. All values denote variance per second
	double modelVarWindH; // model variance of horizontal wind
	double modelVarWindV; // model variance of vertical wind
	double modelVarEta; // model variance of calibration factor eta
	double modelVarAirspeedLon; // model variance of x airspeed
	double modelVarAirspeedSideUp; // model variance of y,z airspeed

	// Measurement variances. All values denote variance per second
	double measurementVarGroundspeedH; // measurement variance of horizontal groundspeed
	double measurementVarGroundspeedV; // measurement variance of vertical component of groundspeed
	double measurementVarPressure; // measurement variance of differential pressure

	// Rotation matrizes
	Matrix3d R_g2f; // earth to body fixed coordinates
	Matrix3d R_f2g; // body fixed coordinates to earth frame
};

#endif
