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

#ifndef SRC_FILTER_WINDFILTER_H_
#define SRC_FILTER_WINDFILTER_H_

#include <math.h>
#include <tgmath.h>

#include "Eigen/Core"

#include "WindTurbulenceIntensityFilter.h"

using namespace Eigen;

/*
 *  class WindFilter
 *
 *  This class is responsible for wind-related calculations like wind direction and magnitude.
 *  It has to be updated with the current wind vector and the corresponding variance.
 */

class WindFilter {
public:
	WindFilter();
	inline Vector3d getWindVector(){return windVector;} // Returns the current wind vector
	inline Vector3d getWindSigmaVector(){return sigmaVector;} // Returns the current sigma for each vector component
	inline Vector3d getWindVarianceVector(){return varianceVector;} // Returns the variances for each vector component
	inline Vector3d getWind3SigBorderLower(){return border3SigLower;} // Returns the current lower 3*sigma border for plotting
	inline Vector3d getWind3SigBorderUpper(){return border3SigUpper;} // Returns the current uppper 3*sigma border for plotting
	inline double getWindMagnitude(){return windMagnitude;} // Returns the magnitude of the current wind vector
	inline double getWindDirFROM(){return windDirFROM;} // Returns the direction FROM which the wind is blowing
	inline double getWindDirTO(){return windDirTO;} // Returns the direction TO which the wind is blowing
	inline double getWindTurbulenceIntensity(){return tiFilter.getTurbulenceIntensity();} // Returns the estimated turbulence intensity

protected:
	void setWind(Vector3d);
	void setWindSigma(Vector3d);
	void setWindSigmaWithVariance(Vector3d);

private:
	Vector3d windVector;
	Vector3d sigmaVector;
	Vector3d varianceVector;
	Vector3d border3SigLower;
	Vector3d border3SigUpper;
	double windMagnitude;
	double windDirFROM;
	double windDirTO;
	WindTurbulenceIntensityFilter tiFilter;
};

#endif /* SRC_FILTER_WINDFILTER_H_ */
