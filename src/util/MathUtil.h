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

#ifndef _MATHUTIL_H
#define _MATHUTIL_H

#include <Eigen/Dense>

using namespace Eigen;

namespace Util{
	const double PI = 3.14159265359;
	
	inline double deg2rad(double deg){return (2*PI/180)*deg;}
	inline double rad2deg(double rad){return (180/2*PI)*rad;}
	Matrix3d calcRotationMatrix(Quaternion<double> q);
	Matrix3d calcRotationMatrix(Vector4d q); ///Return the rotation matrix in Z,Y,X convention from quaternion
	Matrix3d calcRotationMatrix(double q0, double q1, double q2, double q3); ///Return the rotation matrix in Z,Y,X convention from quaternion
	Matrix3d calcRotationMatrix(double phi, double theta, double psi); ///Return the rotation matrix in Z,Y,X convention from euler angles(radians!)
}

#endif
