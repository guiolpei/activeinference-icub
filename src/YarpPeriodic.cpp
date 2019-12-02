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

#include "YarpPeriodic.h"

YarpPeriodic::YarpPeriodic(double p, string description) : RateThread(p)
{
	//constructor
	this->description = description;
}

void YarpPeriodic::setRun(fpointer execute)
{
	//define run function
	this->execute = execute;
	
}

bool YarpPeriodic::threadInit()
{
	cout << "Starting thread: " << description << endl;
	return true;
}

void YarpPeriodic::afterStart(bool s)
{
	//called by start after threadInit, s is true iff the thread started
	//successfully
	
	if (s)
		cout << description << " started successfully" << endl;
	else
		cout << description <<  " did not start" << endl;
}

void YarpPeriodic::run() 
{
	execute();
}

void YarpPeriodic::threadRelease()
{
	cout << "Goodbye from " << description << endl;
}
