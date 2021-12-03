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
#include <sof_d.hh>

//---------------------------------------------------------------------
sof_d::sof_d() 
{
	_c1 = 0.0;
    _c2 = 0.0;
    _c3 = 0.0;
    _c4 = 0.0;
    _c5 = 0.0;
    _c6 = 0.0;
    _u_prev_1 = 0.0;
    _u_prev_2 = 0.0;
    _y_prev_1 = 0.0;
    _y_prev_2 = 0.0;
    _y = 0.0;
}

//---------------------------------------------------------------------
sof_d::~sof_d() 
{
	// Nothing to do
}

//---------------------------------------------------------------------
double sof_d::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
void sof_d::set_c1to6(double c1, double c2, double c3, double c4, double c5, double c6)
{
	_c1 = c1;
    _c2 = c2;
    _c3 = c3;
    _c4 = c4;
    _c5 = c5;
    _c6 = c6;
}

//---------------------------------------------------------------------
double sof_d::compute(double dt, double u)
{
    if ( dt > 0.0 )
    {
        double den = 4.0*_c4 + 2.0*_c5*dt + _c6*dt*dt;
        double dt2 = dt * dt;
        double ca = ( 4.0 * _c1       + 2.0 * _c2 * dt + _c3 * dt2 ) / den;
        double cb = ( 2.0 * _c3 * dt2 - 8.0 * _c1                  ) / den;
        double cc = ( 4.0 * _c1       - 2.0 * _c2 * dt + _c3 * dt2 ) / den;
        double cd = ( 2.0 * _c6 * dt2 - 8.0 * _c4                  ) / den;
        double ce = ( 4.0 * _c4       - 2.0 * _c5 * dt + _c6 * dt2 ) / den;
        _y = u * ca + _u_prev_1 * cb + _u_prev_2 * cc
                    - _y_prev_1 * cd - _y_prev_2 * ce;
        _u_prev_2 = _u_prev_1;
        _u_prev_1 = u;
        _y_prev_2 = _y_prev_1;
        _y_prev_1 = _y;
    }
    return _y;
}
