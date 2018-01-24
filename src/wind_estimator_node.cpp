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


#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
//Messages
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/ParamGet.h>

//Standart libs
#include <tgmath.h>
#include <mutex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctime>

//Boost libs
#include <boost/bind.hpp>
#include <boost/ref.hpp>

//Eigen
#include <Eigen/Dense>

//Own headers
#include "filter/cekfilter.h"
#include "filter/simplewindkf.h"
#include "filter/WindTurbulenceIntensityFilter.h"
#include "util/EigenRosConversions.h"

using namespace std;

typedef Matrix<double,5,1> Vector5d;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,7,1> Vector7d;

typedef struct AircraftDataset{
	std::mutex mutex;
	double airspeed;//m/s
	double heading;	//degrees
	Eigen::Vector3d gps_vel_ned; //m/s
	Eigen::Vector3d gps_vel_enu; //m/s
	Eigen::Vector3d angular_velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d rpy; // Euler angles in deg
	Eigen::Quaterniond q_ned2body; //attitute quaternion
	Eigen::Quaterniond q_enu2body;
	Eigen::Vector3d Va_f;
	Eigen::Vector3d Va_g;
	Eigen::Matrix3d R_f2g;
	Eigen::Matrix3d R_g2f;
	double differentialPressure; //Pa
}AircraftDataset;

AircraftDataset data;

CEKFilter cekf;
SimpleWindKF simpleWindKF;

bool received_vfr_hud = false;
bool received_imu_data = false;
bool received_local_position = false;

/*===========================================================================================================
	C A L L B A C K S
============================================================================================================*/
void Wind_Estimation_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
	ROS_INFO("Windspeed: [%3.3f %3.3f %3.3f]", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void VFR_HUD_Callback(const mavros_msgs::VFR_HUD::ConstPtr& msg){
	//ROS_INFO("Airspeed: %f m/s", msg->airspeed);
	data.mutex.lock();
	data.airspeed = msg->airspeed;
	data.Va_f << msg->airspeed,0,0;
	data.heading = msg->heading;
	//data.gps_vel(2) = (-1)*(msg->climb);
	
	data.mutex.unlock();
	received_vfr_hud = true;
}

void IMU_Data_Callback(const sensor_msgs::Imu::ConstPtr& msg){
	data.mutex.lock();
	data.angular_velocity = Util::toVector3d(msg->angular_velocity);
	data.angular_velocity(1) *= -1;
	data.angular_velocity(2) *= -1;
	data.acceleration = Util::toVector3d(msg->linear_acceleration);
	data.acceleration(1) *= -1;
	data.acceleration(2) *= -1;

	//Convert quaternion from ENU to NED
	tf::Quaternion enu2ned = tf::createQuaternionFromRPY(0, Util::PI, -0.5*Util::PI);
	tf::Quaternion att;
	tf::quaternionMsgToTF(msg->orientation,att);
	tf::quaternionTFToEigen(att,data.q_enu2body);
	att = enu2ned*att;
	tf::quaternionTFToEigen(att,data.q_ned2body);

	data.mutex.unlock();
	received_imu_data = true;
}

void Local_Position_Velocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
	data.mutex.lock();

	data.gps_vel_enu = Util::toVector3d(msg->twist.linear); //Data comes in North, East, Up.

	data.gps_vel_ned(0) = data.gps_vel_enu(1);
	data.gps_vel_ned(1) = data.gps_vel_enu(0);
	data.gps_vel_ned(2) = (-1)*data.gps_vel_enu(2);

	data.mutex.unlock();
	received_local_position = true;
}

/*===========================================================================================================
	M A I N   F U N C T I O N
============================================================================================================*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wind_estimator");

	ros::NodeHandle node;

	double filterFreq;

	//==================
	// Setup publishers
	ros::Publisher filter_freq_pub = node.advertise<std_msgs::Float64>("wind_estimator/filter_frequency",100);

	ros::Publisher cekf_wind_pub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("wind_estimator/cekf/wind",100);
	ros::Publisher cekf_wind_sigma_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/cekf/wind/sigma",100);
	ros::Publisher cekf_wind_3sig_upper_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/cekf/wind/3sig/upper",100);
	ros::Publisher cekf_wind_3sig_lower_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/cekf/wind/3sig/lower",100);
	ros::Publisher cekf_airspeed_pub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("wind_estimator/cekf/airspeed",100);
	ros::Publisher cekf_eta_pub = node.advertise<std_msgs::Float64>("wind_estimator/cekf/calibrationFactor",100);
	ros::Publisher cekf_ti_pub = node.advertise<std_msgs::Float64>("wind_estimator/cekf/turbulenceIntensity",100);

	ros::Publisher swkf_wind_pub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("wind_estimator/swkf/wind",100);
	ros::Publisher swkf_wind_sigma_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/swkf/wind/sigma",100);
	ros::Publisher swkf_wind_3sig_upper_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/swkf/wind/3sig/upper",100);
	ros::Publisher swkf_wind_3sig_lower_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/swkf/wind/3sig/lower",100);
	ros::Publisher swkf_ti_pub = node.advertise<std_msgs::Float64>("wind_estimator/swkf/turbulenceIntensity",100);

	ros::Publisher unfiltered_wind_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/unfiltered/wind",100);
	ros::Publisher moving_average_wind_pub = node.advertise<geometry_msgs::TwistStamped>("wind_estimator/movingAverage/wind",100);

	//==================
	// Setup subscribers
	mavros_msgs::State state;
	
	ros::Subscriber imu_data_sub = node.subscribe("mavros/imu/data",10,IMU_Data_Callback);
	ros::Subscriber local_position_velocity_sub = node.subscribe("mavros/local_position/velocity",10,Local_Position_Velocity_Callback);
	ros::Subscriber vfr_hud_sub = node.subscribe("mavros/vfr_hud",10,VFR_HUD_Callback);
	//ros::Subscriber wind_estimation_sub = node.subscribe("mavros/wind_estimation", 10,Wind_Estimation_Callback);
	
	//=====================
	// Setup service server
	//ros::ServiceServer imuSetter_service = node.advertiseService("wind_estimator/useImu",setUseImu);

	//=====================
	// Setup service client
	ros::ServiceClient param_get_client = node.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");

	//=====================
	// Start spinner
	ros::AsyncSpinner spinner(4);
	spinner.start();



	//=====================
	// Declare filter
	// Calibrating EKF
	cekf.setUseIMUInput(false);
	CEKFilter::MeasurementType cekf_measurement;
	CEKFilter::InputType cekf_input;
	CEKFilter::StateType cekf_estimation;
	CEKFilter::StateCovarianceType cekf_covariance;
	bool isInitialized = false;

	// No filter
	Vector3d unfiltered_wind;

	// Moving average filter
	MovingAverageFilter<Vector3d> moving_average_filter(10*60*3); //10min average -> 10 Samples/s * 60s/min * 10min = 6000 Samples
	Vector3d moving_average_wind;

	//Simple wind kalmanfilter
	SimpleWindKF::MeasurementType swkf_measurement;
	SimpleWindKF::StateType swkf_estimation;
	SimpleWindKF::StateCovarianceType swkf_covariance;

	ElapsedTimer elapsedTime;

	//=====================
	// Prepare log file in .csv format
	ofstream logfile;
	ElapsedTimer logTimer;
	logTimer.reset();
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char logfilename[100];
    sprintf(logfilename,"wind_estimator_%04d_%02d_%02d_%02d_%02d_%02d.log",now->tm_year+1900,now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
	logfile.open(logfilename, std::ofstream::out);
	if(logfile.is_open()){
		ROS_INFO("Logfile successfully opened!");
	}else{
		ROS_WARN("Failed to open logfile!");
	}
	logfile << "# Log file of wind_estimator node" << endl;
	logfile << "time \t"

			<< "unfiltered_wind_x \t"
			<< "unfiltered_wind_y \t"
			<< "unfiltered_wind_z \t"

			<< "cekf_wind_x \t"
			<< "cekf_wind_y \t"
			<< "cekf_wind_z \t"
			<< "cekf_wind_x_sigma \t"
			<< "cekf_wind_y_sigma \t"
			<< "cekf_wind_z_sigma \t"
			<< "cekf_eta \t"
			<< "cekf_eta_sigma \t"
			<< "cekf_airspeed_x \t"
			<< "cekf_airspeed_y \t"
			<< "cekf_airspeed_z \t"
			<< "cekf_airspeed_x_sigma \t"
			<< "cekf_airspeed_y_sigma \t"
			<< "cekf_airspeed_z_sigma \t"
			<< "cekf_turbulence_intensity \t"

			<< "swkf_wind_x \t"
			<< "swkf_wind_y \t"
			<< "swkf_wind_z \t"
			<< "swkf_wind_x_sigma \t"
			<< "swkf_wind_y_sigma \t"
			<< "swkf_wind_z_sigma \t"
			<< "swkf_turbulence_intensity \t"

			<< "moving_average_wind_x \t"
			<< "moving_average_wind_y \t"
			<< "moving_average_wind_z \t"

			<< "loop_calc_time \t"
			<< "frequency"

			<< endl;


	/*==============================================
		Main loop
	================================================*/
	while (ros::ok())
	{
		while(!(received_vfr_hud && received_imu_data && received_local_position) && ros::ok());
		data.mutex.lock();
			elapsedTime.reset();
			//Request AIRSPEED_RATIO = 1/eta
			mavros_msgs::ParamGet srv;
			srv.request.param_id = "ARSPD_RATIO";
			double ARSPD_RATIO = 0;
			if(param_get_client.call(srv) && srv.response.success){
				ARSPD_RATIO = srv.response.value.real;
				data.differentialPressure = (1/ARSPD_RATIO) * pow(data.airspeed,2);
				ROS_INFO("Eta: %f", 1/ARSPD_RATIO);
			}else{
				ROS_ERROR("Failed to get parameter! %d",srv.response.success);		
			}

			received_vfr_hud = false;
			received_imu_data = false;
			received_local_position = false;

			/*==============================================
				Calculate common variables
			================================================*/
			data.R_g2f = Util::calcRotationMatrix(data.q_enu2body);
			data.R_f2g = data.R_g2f.transpose();

			data.Va_g = data.R_f2g * data.Va_f;

			/*=============================================
			    Calculate wind without any filter
			================================================*/

			unfiltered_wind = data.gps_vel_enu - data.Va_g;
			cout << "Unfiltered wind: " << endl << unfiltered_wind << endl;

			/*==============================================
				Update CEKF
			================================================*/
			if(!isInitialized){
				isInitialized = true;
				//Init CEKF
				cekf_estimation << 0.1, 0.1, 0.1, 0.6125, data.R_g2f * data.gps_vel_enu;
				cekf.initialize(cekf_estimation);
				cekf.setModelVarWind(1e-4,1e-4);
				cekf.setModelVarEta(1e-7);
				cekf.setModelVarAirspeed(1e-2,0.1);
				cekf.setMeasurementVarGroundspeedV(1);
				cekf.setMeasurementVarGroundspeedH(1);
				cekf.setMeasurementVarPressure(10);
				cout << "Initialized filter with state: " << endl << cekf_estimation << endl;
				//Init SWKF
				simpleWindKF.setModelVarWindspeed(0.0001);
				simpleWindKF.setMeasurementVarAirspeed(1);
				simpleWindKF.setMeasurementVarGroundspeed(2);
				//Init MovingAverageFilter
				moving_average_filter.initialize(unfiltered_wind);
			}
			//Fill measurement vector z
			cekf_measurement << data.gps_vel_enu, data.differentialPressure;
			//Fill input vector u
			cekf_input.head(3) = data.angular_velocity;
			cekf_input.tail(3) = data.acceleration;
			//Run filter
			cekf_estimation = cekf.getValue(cekf_measurement,
													cekf_input,
													data.q_enu2body);
			cekf_covariance = cekf.getStateCovariance();

			filterFreq = cekf.getFrequency();

			ROS_INFO("Ti: %f",cekf.getWindTurbulenceIntensity());

			cout << "CEKF State" << endl << cekf_estimation << endl;

			/*==============================================
				Update Lorenz's filter - SWKF
			================================================*/
			swkf_measurement << data.gps_vel_enu, data.Va_g;
			swkf_estimation = simpleWindKF.getValue(swkf_measurement);
			swkf_covariance = simpleWindKF.getStateCovariance();

			cout << "SWKF State:" << endl << swkf_estimation << endl;

			/*=============================================
			    Update MovingAverageFilter
			================================================*/

			moving_average_wind = moving_average_filter.getValue(unfiltered_wind);
			cout << "10min average:" << endl << moving_average_wind << endl;

			/*==============================================
				publish results
			================================================*/
			//General
			std_msgs::Float64 filter_freq_msg;
			filter_freq_msg.data = filterFreq;
			filter_freq_pub.publish(filter_freq_msg);

			//CEKF
			geometry_msgs::TwistWithCovarianceStamped cekf_wind_msg;
			geometry_msgs::TwistStamped cekf_wind_sigma_msg;
			geometry_msgs::TwistStamped cekf_wind_3sig_upper_msg;
			geometry_msgs::TwistStamped cekf_wind_3sig_lower_msg;
			geometry_msgs::TwistWithCovarianceStamped cekf_airspeed_msg;
			std_msgs::Float64 cekf_eta_msg;
			std_msgs::Float64 cekf_ti_msg;

			cekf_wind_msg.header.stamp = ros::Time::now();
			cekf_wind_sigma_msg.header.stamp = ros::Time::now();
			cekf_wind_3sig_upper_msg.header.stamp = ros::Time::now();
			cekf_wind_3sig_upper_msg.header.stamp = ros::Time::now();
			cekf_airspeed_msg.header.stamp = ros::Time::now();
			tf::vectorEigenToMsg(cekf_estimation.tail(3),cekf_airspeed_msg.twist.twist.linear);
			tf::vectorEigenToMsg(cekf.getWindSigmaVector(), cekf_wind_sigma_msg.twist.linear);
			tf::vectorEigenToMsg(cekf.getWind3SigBorderUpper(), cekf_wind_3sig_upper_msg.twist.linear);
			tf::vectorEigenToMsg(cekf.getWind3SigBorderLower(), cekf_wind_3sig_lower_msg.twist.linear);
			tf::vectorEigenToMsg(cekf_estimation.head(3),cekf_wind_msg.twist.twist.linear);

			Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero(6,6);
			//Assign airspeed covariances
			cov.topLeftCorner(3,3) = cekf_covariance.bottomRightCorner(3,3);
			cekf_airspeed_msg.twist.covariance = Util::toBoostArray(cov);
			//Assign wind covariances
			cov.topLeftCorner(3,3) = cekf_covariance.topLeftCorner(3,3);
			cekf_wind_msg.twist.covariance = Util::toBoostArray(cov);

			cekf_eta_msg.data = cekf_estimation(3);

			cekf_ti_msg.data = cekf.getWindTurbulenceIntensity();

			cekf_wind_pub.publish(cekf_wind_msg);
			cekf_wind_sigma_pub.publish(cekf_wind_sigma_msg);
			cekf_wind_3sig_upper_pub.publish(cekf_wind_3sig_upper_msg);
			cekf_wind_3sig_lower_pub.publish(cekf_wind_3sig_lower_msg);
			cekf_airspeed_pub.publish(cekf_airspeed_msg);
			cekf_eta_pub.publish(cekf_eta_msg);
			cekf_ti_pub.publish(cekf_ti_msg);

			ROS_INFO("CEKF magnitude: %f m/s | CEKF wind dir: %03.1f degrees", cekf.getWindMagnitude(), cekf.getWindDirFROM());

			//SWKF
			geometry_msgs::TwistWithCovarianceStamped swkf_wind_msg;
			geometry_msgs::TwistStamped swkf_wind_sigma_msg;
			geometry_msgs::TwistStamped swkf_wind_3sig_upper_msg;
			geometry_msgs::TwistStamped swkf_wind_3sig_lower_msg;
			std_msgs::Float64 swkf_ti_msg;

			swkf_wind_msg.header.stamp = ros::Time::now();
			tf::vectorEigenToMsg(swkf_estimation.tail(3),swkf_wind_msg.twist.twist.linear);

			cov = Eigen::Matrix<double,6,6>::Zero(6,6);
			cov.topLeftCorner(3,3) = swkf_covariance.bottomRightCorner(3,3);
			swkf_wind_msg.twist.covariance = Util::toBoostArray(cov);
			tf::vectorEigenToMsg(simpleWindKF.getWindSigmaVector(), swkf_wind_sigma_msg.twist.linear);
			tf::vectorEigenToMsg(simpleWindKF.getWind3SigBorderUpper(), swkf_wind_3sig_upper_msg.twist.linear);
			tf::vectorEigenToMsg(simpleWindKF.getWind3SigBorderLower(), swkf_wind_3sig_lower_msg.twist.linear);

			swkf_ti_msg.data = simpleWindKF.getWindTurbulenceIntensity();

			swkf_wind_pub.publish(swkf_wind_msg);
			swkf_wind_sigma_pub.publish(swkf_wind_sigma_msg);
			swkf_wind_3sig_upper_pub.publish(swkf_wind_3sig_upper_msg);
			swkf_wind_3sig_lower_pub.publish(swkf_wind_3sig_lower_msg);
			swkf_ti_pub.publish(swkf_ti_msg);

			//Unfiltered wind
			geometry_msgs::TwistStamped unfiltered_wind_msg;
			unfiltered_wind_msg.header.stamp = ros::Time::now();
			tf::vectorEigenToMsg(unfiltered_wind, unfiltered_wind_msg.twist.linear);
			unfiltered_wind_pub.publish(unfiltered_wind_msg);


			//Moving average wind
			geometry_msgs::TwistStamped moving_average_wind_msg;
			moving_average_wind_msg.header.stamp = ros::Time::now();
			tf::vectorEigenToMsg(moving_average_wind, moving_average_wind_msg.twist.linear);
			moving_average_wind_pub.publish(moving_average_wind_msg);


			ROS_INFO("Frequency: %f hz",filterFreq);
			ROS_INFO("Total calculation time: %f ms",elapsedTime.elapsed()*1000);

			/*==============================================
				log data
			================================================*/
			logfile << logTimer.elapsed() << "\t"

					<< unfiltered_wind(0) << "\t"
					<< unfiltered_wind(1) << "\t"
					<< unfiltered_wind(2) << "\t"

					<< cekf.getWindVector()(0) << "\t"
					<< cekf.getWindVector()(1) << "\t"
					<< cekf.getWindVector()(2) << "\t"
					<< cekf.getWindSigmaVector()(0) << "\t"
					<< cekf.getWindSigmaVector()(1) << "\t"
					<< cekf.getWindSigmaVector()(2) << "\t"
					<< cekf_estimation(3) << "\t"
					<< sqrt(cekf_covariance(3,3)) << "\t"
					<< cekf_estimation(4) << "\t"
					<< cekf_estimation(5) << "\t"
					<< cekf_estimation(6) << "\t"
					<< sqrt(cekf_covariance(4,4)) << "\t"
					<< sqrt(cekf_covariance(5,5)) << "\t"
					<< sqrt(cekf_covariance(6,6)) << "\t"
					<< cekf.getWindTurbulenceIntensity() << "\t"

					<< simpleWindKF.getWindVector()(0) << "\t"
					<< simpleWindKF.getWindVector()(1) << "\t"
					<< simpleWindKF.getWindVector()(2) << "\t"
					<< simpleWindKF.getWindSigmaVector()(0) << "\t"
					<< simpleWindKF.getWindSigmaVector()(1) << "\t"
					<< simpleWindKF.getWindSigmaVector()(2) << "\t"
					<< simpleWindKF.getWindTurbulenceIntensity() << "\t"

					<< moving_average_wind(0) << "\t"
					<< moving_average_wind(1) << "\t"
					<< moving_average_wind(2) << "\t"

					<< elapsedTime.elapsed() << "\t"
					<< filterFreq

					<< endl;

		data.mutex.unlock();
	}// end of while loop

	logfile.close();

	return 0;
}
// %EndTag(FULLTEXT)%
