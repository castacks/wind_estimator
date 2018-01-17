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

#ifndef SRC_ELAPSEDTIMER_H_
#define SRC_ELAPSEDTIMER_H_

#include <iostream>
#include <ctime>

class ElapsedTimer
{
public:
    ElapsedTimer() { clock_gettime(CLOCK_REALTIME, &beginTime); }

    //returns elapsed time in seconds
    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &endTime);
        return endTime.tv_sec - beginTime.tv_sec +
            (endTime.tv_nsec - beginTime.tv_nsec) / 1000000000.;
    }

    void reset() { clock_gettime(CLOCK_REALTIME, &beginTime); }

private:
    timespec beginTime, endTime;
};

#endif /* SRC_ELAPSEDTIMER_H_ */
