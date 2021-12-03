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

#include "f16_engine_f.hh"

//---------------------------------------------------------------------
f16_engine_f::f16_engine_f()
{

	// ------------- Engines -------------------
	memset(&_thrust_idle_mach_alt_d1_f, 0, sizeof(_thrust_idle_mach_alt_d1_f));
	memset(&_thrust_mil_mach_alt_d1_f, 0, sizeof(_thrust_mil_mach_alt_d1_f));
	memset(&_thrust_max_mach_alt_d1_f, 0, sizeof(_thrust_max_mach_alt_d1_f));
	_pa = _pc = _thrust =  0.0;
	
}

//---------------------------------------------------------------------
f16_engine_f::~f16_engine_f()
{

}

//---------------------------------------------------------------------
//
void f16_engine_f::load_tables_engine_f()
{
	int i,j;
	// ------------- Engines -------------------
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_idle_mach_alt_d1_f[i][j] = thrust_idle_mach_alt_d1_f[i][j];
		}
	}
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_mil_mach_alt_d1_f[i][j] = thrust_mil_mach_alt_d1_f[i][j];
		}
	}
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_max_mach_alt_d1_f[i][j] = thrust_max_mach_alt_d1_f[i][j];
		}
	}
}

//---------------------------------------------------------------------
// this need to be checked http://www.ida.upmc.fr/~zaleski/markers_doc/interpolation_8h-source.html
float f16_engine_f::interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val)
{
	float lambda, value;
	int r = 0;

	while(r < row_nb && table[r][0] < row_val) {r++;}

	if (r == 0) {r = 1;}
	if (r == row_nb) {r--;}

	lambda = (row_val - table[r-1][0])/(table[r][0] - table[r-1][0]);
	value  = table[r-1][1] + lambda*(table[r][1] - table[r-1][1]);

	return value;
}

//---------------------------------------------------------------------
float f16_engine_f::interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val)
{
	float lambda1, lambda2, value;
	int r = 1;
	int c = 1;

	while(r < row_nb && table[r][0] < row_val) {r++;}
	while(c < col_nb && table[0][c] < col_val) {c++;}

	if (r == 1) {r = 2;}
	if (r == row_nb) {r--;}
	if (c == 1) {c = 2;}
	if (c == col_nb) {c--;}

	lambda1 = (row_val - table[r-1][0])/(table[r][0] - table[r-1][0]);
	lambda2 = (col_val - table[0][c-1])/(table[0][c] - table[0][c-1]);

	value  = (1.0 - lambda1) * (1.0 - lambda2) * table[r-1][c-1];
	value +=       (lambda1) * (1.0 - lambda2) * table[r][c-1]; 
	value += (1.0 - lambda1) *       (lambda2) * table[r-1][c]; 
	value +=       (lambda1) *       (lambda2) * table[r][c];

	return value;
}

//---------------------------------------------------------------------
// 
float f16_engine_f::tgear (float throt)
{
    float tgear_value; 
    if (throt <= 0.77)
    {
		tgear_value = 64.94 * throt;
    }
    else
    {
		tgear_value = 217.38 * throt - 117.38;
    }
    return tgear_value;    
}

//---------------------------------------------------------------------
// 
float f16_engine_f::rtau (float dp )
{
    float tau_r;
    if ( dp <= 25.0 )
    {
        tau_r = 1.0;  
    }
    else if ( dp >= 50.0 )
    {
        tau_r = 0.1;
    }
    else
    {
        tau_r = 1.9 - 0.036 * dp;
    }
    return tau_r;
}

//---------------------------------------------------------------------
// 
float f16_engine_f::power_dot( float pw, float cpw )
{

    float tpw, ta;

    if ( cpw >= 50.0 )
    {
        if ( pw >= 50.0)
        {
                tpw = cpw;
                ta = 5.0;
        }
        else
        {
            tpw = 60.0;
            ta = rtau ( tpw - pw );
        }
    }
    else
    {
        if ( pw >= 50.0)
        {
            tpw = 40.0;
            ta = 5.0;
        }
        else
        {
            tpw = cpw;
            ta = rtau ( tpw - pw );
        }
    }
    return ta * ( tpw - pw );
}

//---------------------------------------------------------------------
// 
float f16_engine_f::thrust(float power, float mach, float alt_m)
{
	float thrust_idle = 0.0, thrust_mil = 0.0, thrust_max = 0.0;
	thrust_idle = interp_2d_f(_thrust_idle_mach_alt_d1_f,6,7,mach,alt_m);
	thrust_mil = interp_2d_f(_thrust_mil_mach_alt_d1_f,6,7,mach,alt_m);
	thrust_max = interp_2d_f(_thrust_max_mach_alt_d1_f,6,7,mach,alt_m);

	if (power < 50)
	{
		_thrust = thrust_idle +(thrust_mil - thrust_idle)*power/50;
	}
	else if (power>=50)
	{
		_thrust = thrust_mil +(thrust_max - thrust_mil)*(power-50)/50;
	}
	
	//_thrust = _thrust*4.4482216;
	//_thrust = _thrust*4.4482216;
	if (_thrust > 19000*4.4482216) _thrust = 19000*4.4482216;
	if (_thrust < 1000*4.4482216) _thrust = 1000*4.4482216;
	return _thrust;
}

