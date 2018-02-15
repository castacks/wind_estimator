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

#include "EigenRosConversions.h"

namespace Util{
Eigen::Vector4d toVector4d(geometry_msgs::Quaternion q){
	Eigen::Vector4d result;
	result(0) = (double)q.w;
	result(1) = (double)q.x;
	result(2) = (double)q.y;
	result(3) = (double)q.z;
	return result;
}

Eigen::Vector3d toVector3d(geometry_msgs::Vector3 rosvec){
	Eigen::Vector3d result;
	result(0) = (double)rosvec.x;
	result(1) = (double)rosvec.y;
	result(2) = (double)rosvec.z;
	return result;
}

boost::array<double,36> toBoostArray(Eigen::Matrix<double,6,6> eigenMatrix){
	boost::array<double,36> result;

	int r = 0;
	for(int i=0; i<6; i++){
		for(int j = 0; j<6; j++){
			result[r] = eigenMatrix(i,j);
			r++;
		}
	}

	return result;
}

Eigen::Matrix<double,6,6> toEigenArray(boost::array<double,36> boostMatrix){
	Eigen::Matrix<double,6,6> result;

	int r = 0;
	for(int i=0; i<6; i++){
		for(int j = 0; j<6; j++){
			result(i,j) = boostMatrix[r];
			r++;
		}
	}

	return result;
}

Eigen::Vector3d ENU2NED(Eigen::Vector3d enu){
	Eigen::Vector3d result;
	result(0) = enu(1);
	result(1) = enu(0);
	result(2) = -enu(2);
	return result;
}

}//end namespace
