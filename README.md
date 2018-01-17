# License
  
Copyright (c) 2018 Julian Soeren Lorenz, Carnegie Mellon University, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

   	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
   	   following disclaimer.
   	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   	   following disclaimer in the documentation and/or other materials provided with the distribution.
   	3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
   	   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

END OF LICENSE

Author: Julian Soeren Lorenz

Email:  JulianLorenz@live.de

# wind_estimator ROS node

## Description

This node estimates the prevaling wind condition based on the provided data by mavros.
It runs multiple wind estimation algorithms and publishs the results in seperat topics.

## Subscribed ROS topics

* mavros/imu/data : attitude information
* mavros/local_position/velocity : GPS velocity
* mavros/vfr_hud : Airspeed, Heading

## Published ROS topics

Topic name | Message type

--- | ---

* wind_estimator/filter_frequency			| std_msgs/Float64

* wind_estimator/cekf/wind					| geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/cekf/wind/sigma			| geometry_msgs/TwistStamped
* wind_estimator/cekf/wind/3sig/upper		| geometry_msgs/TwistStamped
* wind_estimator/cekf/wind/3sig/lower		| geometry_msgs/TwistStamped
* wind_estimator/cekf/airspeed				| geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/cekf/calibrationFactor		| std_msgs/Float64
* wind_estimator/cekf/turbulenceIntensity	| std_msgs/Float64

* wind_estimator/swkf/wind					| geometry_msgs/TwistWithCovarianceStamped
* wind_estimator/swkf/wind/sigma			| geometry_msgs/TwistStamped
* wind_estimator/swkf/wind/3sig/upper		| geometry_msgs/TwistStamped
* wind_estimator/swkf/wind/3sig/lower		| geometry_msgs/TwistStamped
* wind_estimator/swkf/turbulenceIntensity	| std_msgs/Float64

* wind_estimator/unfiltered/wind			| geometry_msgs/TwistStamped

* wind_estimator/movingAverage/wind			| geometry_msgs/TwistStamped

## Implemented Algorithms
### Simple unfiltered wind calculation 

Calculates a wind vector based on the estimated groundspeed and airspeed vector:

Vw_g = Vg_g - Va_g

### Moving Average Filter 

Takes the moving average of the unfiltered wind vector over 3min.

### Simple Wind Kalman Filter (SWKF)

This is a simple 2D wind Kalman Filter, which assumes the wind and groundspeed to be constant.

Variables:

* Vg_g - 3D groundspeed vector in earth frame [m/s]
* Vw_g - 3D windspeed vector in earth frame [m/s]
* Va_g - 3D airspeed vector in earth frame [m/s] 


State x[k] = [Vg_g,Vw_g]^T

Measurement z[k] = [Vg_g,Va_g]^T


Model: 

* x_p[k] = x[k-1]

Observation equation z_p[k] = h(x_p[k]): 

* Vg_g = Vg_g
* Va_g = Vg_g - Vw_g

### Calibrating Extended Kalman Filter (CEKF)

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

# Contact
For questions contact Julian Lorenz (JulianLorenz@live.de)



 