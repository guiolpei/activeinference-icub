/*
 * YarpPeriodic.cpp
 *
 * Description: Dedicated class for the creation of periodical executions.
 *
 *  Created on: Feb 3, 2019
 *     Authors: Guillermo Oliver, Pablo Lanillos, Gordon Cheng
 *
 * Copyright 2019 Institute for Cognitive Systems, Technical University of Munich. All rights reserved.
 *
 * This file is part of the ActiveInferenceICub project.
 *
 * ActiveInferenceICub is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef YARPPERIODIC_H_
#define YARPPERIODIC_H_

//Includes
//Standard Library
#include <iostream> //cout
#include <string> //strings
//Yarp
#include <yarp/os/all.h> //OS

//Namespaces
using namespace std;
using namespace yarp::os;

//Definitions
typedef void (*fpointer)(void); //pointer to function

class YarpPeriodic : public RateThread
{
	public:
		YarpPeriodic(double p, string description);
		void setRun(fpointer execute);
		virtual bool threadInit();
		virtual void afterStart(bool s);
		virtual void run();
		virtual void threadRelease();
		
	private:

		//Internal variables
		string description;
		fpointer execute;
};

#endif /* YARPPERIODIC_H_ */
