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

// This file has been copied from FlighGear 2016.1.1 version
// Original file (which has been slightly modified here) is:
// 
// net_fdm.hxx -- defines a common net I/O interface to the flight
//                dynamics model
//
// Written by Curtis Olson - http://www.flightgear.org/~curt
// Started September 2001.
//
#ifndef __FLIGHTGEAR_NATIVE_FDM_HH_DEF__
#define __FLIGHTGEAR_NATIVE_FDM_HH_DEF__

#include <iostream>
#include <memory>
#include <string>
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// For Net FDM
#define DEG2RAD (3.14159 / 180.0)
#define FEET2METER 0.3048
#define METER2FEET 3.2808399
#define MS2KTS 1,94384
const uint32_t FG_NET_FDM_VERSION = 24;

// For Net ctrl
#define RESERVED_SPACE 25
const uint32_t FG_NET_CTRLS_VERSION = 27;

// Useful typedef for socket definition
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;

// Define a structure containing the top level flight dynamics model parameters for FlighGear
class FGNetFDM {

public:

    enum {
        FG_MAX_ENGINES = 4,
        FG_MAX_WHEELS = 3,
        FG_MAX_TANKS = 4
    };

    uint32_t version;		// increment when data values change
    uint32_t padding;		// padding

    // Positions
    double longitude;		// geodetic (radians)
    double latitude;		// geodetic (radians)
    double altitude;		// above sea level (meters)
    float agl;			// above ground level (meters)
    float phi;			// roll (radians)
    float theta;		// pitch (radians)
    float psi;			// yaw or true heading (radians)
    float alpha;                // angle of attack (radians)
    float beta;                 // side slip angle (radians)

    // Velocities
    float phidot;		// roll rate (radians/sec)
    float thetadot;		// pitch rate (radians/sec)
    float psidot;		// yaw rate (radians/sec)
    float vcas;		        // calibrated airspeed
    float climb_rate;		// feet per second
    float v_north;              // north velocity in local/body frame, fps
    float v_east;               // east velocity in local/body frame, fps
    float v_down;               // down/vertical velocity in local/body frame, fps
    float v_body_u;    // ECEF velocity in body frame
    float v_body_v;    // ECEF velocity in body frame 
    float v_body_w;    // ECEF velocity in body frame
    
    // Accelerations
    float A_X_pilot;		// X accel in body frame ft/sec^2
    float A_Y_pilot;		// Y accel in body frame ft/sec^2
    float A_Z_pilot;		// Z accel in body frame ft/sec^2

    // Stall
    float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    float slip_deg;		// slip ball deflection

    // Pressure
    
    // Engine status
    uint32_t num_engines;	     // Number of valid engines
    uint32_t eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
    float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    uint32_t num_tanks;		// Max number of fuel tanks
    float fuel_quantity[FG_MAX_TANKS];

    // Gear status
    uint32_t num_wheels;
    uint32_t wow[FG_MAX_WHEELS];
    float gear_pos[FG_MAX_WHEELS];
    float gear_steer[FG_MAX_WHEELS];
    float gear_compression[FG_MAX_WHEELS];

    // Environment
    uint32_t cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    int32_t warp;                // offset in seconds to unix time
    float visibility;            // visibility in meters (for env. effects)

    // Control surface positions (normalized values)
    float elevator;
    float elevator_trim_tab;
    float left_flap;
    float right_flap;
    float left_aileron;
    float right_aileron;
    float rudder;
    float nose_wheel;
    float speedbrake;
    float spoilers;
};

class FlightGearNativeFdm
{
	private:
	
		FGNetFDM _FlighGearFDM;
		
		SOCKET _FlightGearTxSocket;
		SOCKADDR_IN _FlightGearTxAddr;
		
	public:
	
		FlightGearNativeFdm();
		~FlightGearNativeFdm();	
		void FlighGearSocketSend();
		void setLongitude(double val);
		void setLatitude(double val);
		void setAltitude(double val);
		void setPhi(double val);
		void setTheta(double val);
		void setPsi(double val);
		void setUspeed(double val);
		void setVspeed(double val);
		void setWspeed(double val);
		void setXacc(double val);
		void setYacc(double val);
		void setZacc(double val);
		void setVcas(double val);
		void setAlpha(double val);
		void setBeta(double val);
		void setRightAileron(double val);
		void setLeftAileron(double val);
		void setElevator(double val);
		void setRudder(double val);
		void setFlaps(double val);
		void setSpoilers(double val);
		void setGears(double val);
		double fabsFGFS(double x);
		void setEngineThrottle(double val);
		double normalizeValue(double value, double val_min, double val_max);
};

// Network to Host and Host to Network conversion
// for double and float values
double htond (double x) ;
float htonf (float x);

#endif // __FLIGHTGEAR_NATIVE_FDM_HH_DEF__
