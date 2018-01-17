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

#include "WindTurbulenceIntensityFilter.h"

#include <iostream>

WindTurbulenceIntensityFilter::WindTurbulenceIntensityFilter()
	:wind_average_filter(100),
	 wind_variance_average_filter(100)
{
	turbulenceIntensity = 0;
}


double WindTurbulenceIntensityFilter::doIteration(Vector3d wind, Vector3d variance){
	Vector3d currentWindAverage = wind_average_filter.getValue(wind);
	Vector3d currentWindVarianceAverage = wind_variance_average_filter.getValue(variance);

	//Variance = (standard deviation)^2

	double wind_sigma = sqrt((1.0/3.0)*(currentWindVarianceAverage(0)+currentWindVarianceAverage(1)+currentWindVarianceAverage(2)));
	double wind_mean = currentWindAverage.norm();
#ifdef _DEBUG
	std::cout << "Wind_sigma: " << wind_sigma << std::endl;
	std::cout << "Wind_mean: " << wind_mean << std::endl;
#endif

	turbulenceIntensity = wind_sigma/wind_mean;

	return turbulenceIntensity;
}
