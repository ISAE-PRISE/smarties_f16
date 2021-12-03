//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// Copyright (C) 2018-2022  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email:  jean-baptiste.chaudron@isae-supaero.fr
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------

#ifndef __JOYSTICK_HH__
#define __JOYSTICK_HH__

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef __linux__
// Linux-only API
#include <linux/joystick.h> 
#endif


const int MAX_AXIS   = 7;
const int MAX_BUTTON = 24;
const int MAX_INPUTS = MAX_AXIS+MAX_BUTTON;
const double MAX_INT = 32767;

using namespace std;

class Joystick
{
	public:

		Joystick();
		virtual ~Joystick();

		// getInputs does the acquisition of inputs from both joysticks
		// returns the inputs of a joystick if compiled on Linux and JOYSTICK_ENABLED is true,
		// otherwise it returns an array of 0. */
		double* getInputs();

		void setInputs(int index,double value);
		void setFailureInputs(int index,int value);

		// Failure mode
		int _FailureInputs[2*MAX_INPUTS];

	private:

		unsigned char mJ0AxisCount;
		unsigned char mJ1AxisCount;
		unsigned char mJ0ButtonCount;
		unsigned char mJ1ButtonCount;
		
		int mJ0Fd;
		int mJ1Fd;

		int mJ0Axis[MAX_AXIS];
		int mJ1Axis[MAX_AXIS];
		int mJ0Button[MAX_BUTTON];
		int mJ1Button[MAX_BUTTON];

		// The inputs are built like this: [J0 axis,J0 buttons,J1 axis,J1 buttons]
		double mInputs[2*MAX_INPUTS];

		int mJ0Result;
		int mJ1Result;

		#ifdef __linux__
		// is part of the Linux-only joystick API
		js_event mEvent; 
		#endif

};

#endif
