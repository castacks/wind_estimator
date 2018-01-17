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


#include "MathUtil.h"

namespace Util{

Matrix3d calcRotationMatrix(double qw, double qx, double qy, double qz){
    // Sources: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    //          https://arxiv.org/pdf/1708.08680.pdf
    // Note that the below matrix is transposed so it can be pre-multiplied
	Matrix3d R_g2f;
    R_g2f <<
        qw*qw+qx*qx-qy*qy-qz*qz, 2*(qx*qy+qw*qz),          2*(qx*qz-qw*qy),
        2*(qx*qy-qw*qz),         qw*qw-qx*qx+qy*qy-qz*qz,  2*(qw*qx+qy*qz),
        2*(qw*qy+qx*qz),         2*(qy*qz-qw*qx),          qw*qw-qx*qx-qy*qy+qz*qz;
	return R_g2f;
}

Matrix3d calcRotationMatrix(Quaternion<double> q){
	return calcRotationMatrix(q.w(),q.x(),q.y(),q.z());
}

Matrix3d calcRotationMatrix(Vector4d q){
	return calcRotationMatrix(q(0),q(1),q(2),q(3));
}

Matrix3d calcRotationMatrix(double phi, double theta, double psi){
    // Source: https://de.wikipedia.org/wiki/Eulersche_Winkel#Gier-Nick-Roll:_z.2C_y.E2.80.B2.2C_x.E2.80.B3-Konvention
    // Convention: Z Y' X''

	Matrix3d R_g2f;	

    const double c1 = std::cos(psi); //yaw
    const double c2 = std::cos(theta); //pitch
    const double c3 = std::cos(phi);  //roll
    const double s1 = std::sin(psi);
    const double s2 = std::sin(theta);
    const double s3 = std::sin(phi);

	Matrix3d R_yaw, R_pitch, R_roll;

	R_yaw << c1, s1, 0,
			-s1, c1, 0,
 			  0,  0, 1;

	R_pitch << c2, 0, -s2,
			   	0, 1,  0,
			   s2, 0,  c2;

	R_roll << 1,   0,  0,
			  0,  c3, s3,
			  0, -s3, c3;

	R_g2f = R_roll * R_pitch * R_yaw;
	
 /* Resulting matrix:   
	R_g2f <<  c2*c1,               c2*s1,              -s2,
               s3*s2*c1-c3*s1,      s3*s2*s1+c3*c1,     s3*c2,
               c3*s2*c1+s3*s1,      c3*s2*s1-s3*c1,    c3*c2;
 */

	return R_g2f;
}

}//end namespace
