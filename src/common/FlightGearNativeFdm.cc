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

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "FlightGearNativeFdm.hh"

//----------------------------------------------------------------------
// Constructor 
FlightGearNativeFdm::FlightGearNativeFdm() 
{	
	// Flighgear IP address and UDP port used
	// TO DO: The name might change when using interface to Xplane
	char * FGIPAddr = NULL;
	int FGUdpPort;
	
	FGUdpPort = 5500;
	std::string FGIPAddrStr = "127.0.0.1";
	FGIPAddr = (char * ) FGIPAddrStr.c_str();

    
	memset(&_FlightGearTxAddr,0,sizeof(_FlightGearTxAddr));
	_FlightGearTxAddr.sin_family = AF_INET;
	_FlightGearTxAddr.sin_port = htons(FGUdpPort);
	_FlightGearTxAddr.sin_addr.s_addr = inet_addr(FGIPAddr);
	
	_FlightGearTxSocket = socket(AF_INET,SOCK_DGRAM,0);
	if (_FlightGearTxSocket == -1)
	{
		std::cout << "Visualization.cc: FlighGear Socket creation ERROR." << std::endl;
	}

	    // Init FG model 
    // TO DO: We might need to change these hardcoded values
	memset(&_FlighGearFDM,0,sizeof(_FlighGearFDM));
	_FlighGearFDM.version = htonl(FG_NET_FDM_VERSION);
	_FlighGearFDM.latitude = htond((37.2) * DEG2RAD);
	_FlighGearFDM.longitude = htond((-122.385) * DEG2RAD);
	_FlighGearFDM.altitude = htond(10000.0);
	_FlighGearFDM.num_engines = htonl(4);
	_FlighGearFDM.eng_state[0] = htonl(2);
	_FlighGearFDM.eng_state[1] = htonl(2);
	_FlighGearFDM.eng_state[2] = htonl(2);
	_FlighGearFDM.eng_state[3] = htonl(2);
	_FlighGearFDM.num_tanks = htonl(1);
	_FlighGearFDM.fuel_quantity[0] = htonf(100.0);
	_FlighGearFDM.num_wheels = htonl(3);
	_FlighGearFDM.cur_time = htonl(time(0));
	_FlighGearFDM.warp = htonl(1);
	_FlighGearFDM.visibility = htonf(5000.0);
}

//----------------------------------------------------------------------
// Constructor 
FlightGearNativeFdm::~FlightGearNativeFdm() 
{	
	//
}

//----------------------------------------------------------------------
// Send on dedicated Udp socket the native FDM structure 
void FlightGearNativeFdm::FlighGearSocketSend() 
{
	int return_val_1;
	return_val_1 = sendto( _FlightGearTxSocket
		   				,(char *)&_FlighGearFDM
		  				 ,sizeof(_FlighGearFDM)
		  				 ,0
		  				 ,(struct sockaddr *)&_FlightGearTxAddr
		  				 ,sizeof(_FlightGearTxAddr)
		  				);
		  				

	#ifdef DEBUG_VISUALIZATION
	if ( (return_val_1 == -1) 
	{
		std::cout << "Visualization.cc : FlighGear_Socket_Send function ==> ERROR in sendto." << std::endl;
	}
	else
	{
		std::cout << "Visualization_Fed.cc : FlighGear_Socket_Send function ==> sendto is OK." << std::endl;
	}
	#endif
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setLongitude(double val)
{
	_FlighGearFDM.longitude = htond(DEG2RAD * val);
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setLatitude(double val)
{
	_FlighGearFDM.latitude = htond(DEG2RAD * val); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setAltitude(double val)
{
	_FlighGearFDM.altitude = htond(val);
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setPhi(double val)
{
	_FlighGearFDM.phi = htonf((float) (val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setTheta(double val)
{
	_FlighGearFDM.theta = htonf((float) (val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setPsi(double val)
{
	_FlighGearFDM.psi = htonf((float) (val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setUspeed(double val)
{
	_FlighGearFDM.v_body_u = htonf((float) (METER2FEET * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setVspeed(double val)
{
	_FlighGearFDM.v_body_v = htonf((float) (METER2FEET * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setWspeed(double val)
{
	_FlighGearFDM.v_body_w = htonf((float) (METER2FEET * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setXacc(double val)
{
	_FlighGearFDM.A_X_pilot = htonf((float) (METER2FEET * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setYacc(double val)
{
	_FlighGearFDM.A_Y_pilot = htonf((float) (METER2FEET * val));  
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setZacc(double val)
{
	_FlighGearFDM.A_Z_pilot = htonf((float) (METER2FEET * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setVcas(double val)
{
	_FlighGearFDM.vcas = htonf((float) (val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setAlpha(double val)
{
	_FlighGearFDM.alpha = htonf((float) (DEG2RAD * val)); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setBeta(double val)
{
	_FlighGearFDM.beta = htonf((float) (DEG2RAD * val));
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setRightAileron(double val)
{
	/*double normalized_value = -(val<=0)*((val - ra_minpos*degtorad) / (ra_maxpos*degtorad - ra_minpos*degtorad)) 
	                          + (val>0)*((val - ra_minpos*degtorad) / (ra_maxpos*degtorad - ra_minpos*degtorad));*/
	//double normalized_value = (2* ((val - ra_minpos) / (ra_maxpos - ra_minpos)) -1); 
	//_FlighGearFDM.right_aileron = htonf((float) (-(val<=0)*val/(ra_minpos*degtorad) + (val>0)*val/(ra_maxpos*degtorad)));  
	//_FlighGearFDM.right_aileron = htonf((float) normalized_value); 
	_FlighGearFDM.right_aileron = htonf((float) val);
	

}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setLeftAileron(double val)
{
	/*double normalized_value = -(val<=0)*((val - la_minpos*degtorad) / (la_maxpos*degtorad - la_minpos*degtorad)) 
	                          + (val>0)*((val - la_minpos*degtorad) / (la_maxpos*degtorad - la_minpos*degtorad)); */
 // double normalized_value = (2* ((val - la_minpos) / (la_maxpos - la_minpos)) -1);
	//_FlighGearFDM.left_aileron = htonf((float) (-(val<=0)*val/(la_minpos*degtorad) + (val>0)*val/(la_maxpos*degtorad)));
	_FlighGearFDM.left_aileron = htonf((float) val);  
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setElevator(double val)
{
	 //double normalized_value = (2 * ((val - el_minpos) /(el_maxpos - el_minpos)) -1);
	 _FlighGearFDM.elevator = htonf((float) val); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setRudder(double val)
{
	//double normalized_value = (2 * ((val - ru_minpos)/(ru_maxpos - ru_minpos)) - 1); 
	_FlighGearFDM.rudder = htonf((float) val); 
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setFlaps(double val)
{
	//_FlighGearFDM.left_flap  = htonf((float) (val/(fl_maxpos*degtorad)));
	_FlighGearFDM.left_flap  = htonf((float) val);
	_FlighGearFDM.right_flap = _FlighGearFDM.left_flap;
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setSpoilers(double val)
{
	//_FlighGearFDM.spoilers = htonf((float) (val/(sp_maxpos*degtorad)));
	_FlighGearFDM.spoilers = htonf((float) val);
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setGears(double val)
{
	//_FlighGearFDM.gear_pos[0] = htonf((float) (val/(ge_maxpos*degtorad)));
	_FlighGearFDM.gear_pos[0] = htonf((float) val);
	_FlighGearFDM.gear_pos[0] =  htonf((float) 0.0);
	_FlighGearFDM.gear_pos[1] = _FlighGearFDM.gear_pos[0];
	_FlighGearFDM.gear_pos[2] = _FlighGearFDM.gear_pos[0];
}

//----------------------------------------------------------------------
void FlightGearNativeFdm::setEngineThrottle(double val)
{
	_FlighGearFDM.rpm[0] = htonf((float) val);
	_FlighGearFDM.rpm[1] = _FlighGearFDM.rpm[0];
	_FlighGearFDM.rpm[2] = _FlighGearFDM.rpm[1];
	_FlighGearFDM.rpm[3] = _FlighGearFDM.rpm[2];
}

//----------------------------------------------------------------------
// Function: Calculate fabs value (avoiding dependency to lib math)
double FlightGearNativeFdm::fabsFGFS(double x)
{
    return x >= 0 ? x : -x;
}

double FlightGearNativeFdm::normalizeValue(double value, double val_min, double val_max)
{
        double nm_value = (2 * ((value - val_min) / (val_max - val_min))) -1;
        return nm_value;
}

//----------------------------------------------------------------------
double htond (double x)	
{
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = htonl(p[1]);
    p[1] = htonl(tmp);

    return x;
}

//----------------------------------------------------------------------
float htonf (float x)	
{
    int * p = (int *)&x;
    *p = htonl(*p);
    return x;
}






