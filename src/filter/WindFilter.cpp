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

#include "WindFilter.h"

WindFilter::WindFilter() {
	windVector << 0, 0, 0;
	windMagnitude = 0;
	windDirTO = 0;
	windDirFROM = 0;
}

// Sets the wind vector and calculates wind direction and magnitude
void WindFilter::setWind(Vector3d wind){
	windVector = wind;
	windMagnitude = windVector.norm();
	windDirTO = atan2(windVector(0),windVector(1))*180/3.14;
	if(windDirTO < 0){
		windDirTO += 360;
	}
	windDirFROM = windDirTO - 180;
	while(windDirFROM < 0){
		windDirFROM += 360;
	}
}

// Sets the wind sigma vector and calculates the 3-sigma boarders
void WindFilter::setWindSigma(Vector3d sigma){
	this->sigmaVector = sigma;
	border3SigLower = windVector - 3*sigmaVector;
	border3SigUpper = windVector + 3*sigmaVector;
	varianceVector << pow(sigmaVector(0),2), pow(sigmaVector(1),2), pow(sigmaVector(2),2);

	tiFilter.doIteration(windVector, varianceVector);
}

void WindFilter::setWindSigmaWithVariance(Vector3d variance){
	Vector3d sigma(sqrt(variance(0)), sqrt(variance(1)), sqrt(variance(2)));
	setWindSigma(sigma);
}
