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

#include "f16_fdm_f.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include <unistd.h>

int main()
{
	int i = 1;
	f16_fdm_f f16_fdm_entity;
	
	float alt_trim = 5000.0; // m
	float vt_trim = 250.0 ;	 // m/s
	float psi_trim = 0.09;	 // rad
	float p_trim = 0.0;	 // rad/s
	float q_trim = 0.0;	 // rad/s
	float r_trim = 0.0;	 // rad/s
	f16_fdm_entity.load_tables_aero_f();
	f16_fdm_entity.set_trim_conditions(vt_trim, 0.0, 0.0, alt_trim, p_trim, q_trim, r_trim, psi_trim);
	f16_fdm_entity.set_trim_init(20000.0, 0.3, -0.03 * 57.29577951, 0.0, 0.0, 0.6);
	f16_fdm_entity.trim_aircraft();

	usleep(2000000);
	
	while (i < 1000000000)
	{
		f16_fdm_entity.compute_xdot(0.02);
		usleep(200000);
		i++;
	}
	
	return 1;
}
