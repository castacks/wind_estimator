# wind_estimator ROS node

## Description

This node estimates the prevaling wind condition based on the provided data by mavros.
It runs multiple wind estimation algorithms and publishs the results in seperat topics.

## Subscribed ROS topics

* mavros/imu/data : attitude information
* mavros/local_position/velocity : GPS velocity
* mavros/vfr_hud : Airspeed, Heading

## Published ROS topics

* wind_estimator/goyal/wind 	 			geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/goyal/airspeed 			geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/goyal/calibrationFactor  	std_msgs/Float64
* wind_estimator/swkf/wind  				geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/unfiltered/wind	 		geometry_msgs/TwistStamped 

## Implemented Algorithms
### Wind Filter by Pulkit Goyal

This filter employs an extended Kalman Filter with GNSS velocity and differential pressure from a pitottube as input. The output is a wind vector, an airspeed vector and a calibration factor for the pitottube. The filter's model assumes the wind and airspeed to be constant. 

Variables:

* Vw_g - 3D windspeed vector in earth frame [m/s]
* Vg_g - 3D groundspeed vector in earth frame [m/s]
* Va_g - 3D airspeed vecotr in body frame [m/s]
* eta - calibration factor for pitottube [kg/m^3]
* dP - differential pressure from pitottube [N/m^2]
* omega - angular velocity [rad/s]
* acc_f - linear acceleration in body frame [m/s^2] 


State x[k] = [Vw_g, eta, Va_f]^T

Measurement z[k] = [Vg_g, dP]^T

(Input u[k] = [omega,acc_f]^T) [Optional. Deactivated by default]


Model x_p[k] = f(x[k-1],u[k]):

* dot(Vw_g) = 0
* dot(eta) = 0
* dot(Va_f) = acc_f - omega.cross(Va_f)

Observation equation z_p[k] = h(x_p[k])

* Vg_g = Vw_g + R_f2g*Va_f
* dP = eta * Vax_f^2

### Simple Wind Kalman Filter (SWKF) by Julian Lorenz

This is a simple 2D wind Kalman Filter, which assumes the wind and groundspeed to be constant.

Variables:

* Vg_g - 2D groundspeed vector in earth frame [m/s]
* Vw_g - 2D windspeed vector in earth frame [m/s]
* Va_g - 2D airspeed vector in earth frame [m/s] 


State x[k] = [Vg_g,Vw_g]^T

Measurement z[k] = [Vg_g,Va_g]^T


Model: 

* x_p[k] = x[k-1]

Observation equation z_p[k] = h(x_p[k]): 

* Vg_g = Vg_g
* Va_g = Vg_g - Vw_g

### Simple unfiltered wind calculation 

Calculates a wind vector based on the estimated groundspeed and airspeed vector:

Vw_g = Vg_g - Va_g


 