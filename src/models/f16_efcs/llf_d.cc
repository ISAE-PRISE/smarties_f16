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

// Adapted from Marek Cel outstanding work in:
// https://github.com/marek-cel/mscsim

#include <algorithm>
#include <cmath>
#include <llf_d.hh>

//---------------------------------------------------------------------
llf_d::llf_d() 
{
	_c1 = 0.0;
    _c2 = 0.0;
    _c3 = 0.0;
    _c4 = 0.0;
    _u_prev = 0.0;
    _y_prev = 0.0;
    _y = 0.0;
}

//---------------------------------------------------------------------
llf_d::~llf_d() 
{
	// Nothing to do
}

//---------------------------------------------------------------------
void llf_d::set_y( double y )
{
    _y = y;
}

//---------------------------------------------------------------------
double llf_d::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
void llf_d::set_c1to4(double c1, double c2, double c3, double c4)
{
	_c1 = c1;
    _c2 = c2;
    _c3 = c3;
    _c4 = c4;
}


//---------------------------------------------------------------------
double llf_d::compute( double dt, double u )
{
    if ( dt > 0.0 )
    {
        double den = 2.0 * _c3 + dt * _c4;

        double ca = ( 2.0 * _c1 + dt  * _c2 ) / den;
        double cb = ( dt  * _c2 - 2.0 * _c1 ) / den;
        double cc = ( 2.0 * _c3 - dt  * _c4 ) / den;

        _y = u * ca + _u_prev * cb + _y_prev * cc;

        _u_prev = u;
        _y_prev = _y;
    }
    return _y;
}
