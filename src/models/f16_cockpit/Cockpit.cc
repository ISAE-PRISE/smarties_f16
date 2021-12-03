//**********************************************************************
//**********************************************************************
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// Copyright (C) 2018-2021  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email: jean-baptiste.chaudron@isae.fr
//
//**********************************************************************
//**********************************************************************

#include "Cockpit.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


//----------------------------------------------------------------------
// Constructor 
Cockpit::Cockpit() 
{

	 COCKPIT_AUTOPILOT_AP_ACTIVE=false;
	 COCKPIT_AUTOPILOT_AP2_ACTIVE=false;
	 COCKPIT_AUTOPILOT_ATHR_ACTIVE=false;
	 COCKPIT_AUTOPILOT_SPD=0;
	 COCKPIT_AUTOPILOT_HDG=0;
	 COCKPIT_AUTOPILOT_ALT=0;
	 COCKPIT_AUTOPILOT_VS=0;
	 
	 AIRCRAFT_POSITION_LONGITUDE =0.0 ;
	 AIRCRAFT_POSITION_LATITUDE =0.0 ;
	 AIRCRAFT_POSITION_ALTITUDE =0.0 ;
	 
	 AIRCRAFT_ORIENTATION_THETA = 0.0 ;
	 AIRCRAFT_ORIENTATION_PHI = 0.0 ;
	 AIRCRAFT_ORIENTATION_PSI = 0.0 ;
	
	 AIRCRAFT_SPEED_IAS = 0.0 ;
	 AIRCRAFT_SPEED_TAS = 0.0 ;

	 AIRCRAFT_SPEED_MACH = 0.0 ;
	
	 ACTUATORS_RIGHT_ENGINE_THRUST = 0.0 ;
	 ACTUATORS_LEFT_ENGINE_THRUST = 0.0 ;
	
	 ENVIRONMENT_VARIABLES_PRESSURE = 0.0 ;

     AIRCRAFT_ADDITIONAL_ALPHA = 0.0 ;
     AIRCRAFT_ADDITIONAL_BETA = 0.0 ;
     AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE = 0.0 ;
	 
}

//----------------------------------------------------------------------
// Destructor 
Cockpit::~Cockpit()
{
	// Nothing to do
}
