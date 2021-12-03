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
#include <sof_f.hh>

//---------------------------------------------------------------------
sof_f::sof_f() 
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
sof_f::~sof_f() 
{
	// Nothing to do
}

//---------------------------------------------------------------------
float sof_f::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
void sof_f::set_c1to6(float c1, float c2, float c3, float c4, float c5, float c6)
{
	_c1 = c1;
    _c2 = c2;
    _c3 = c3;
    _c4 = c4;
    _c5 = c5;
    _c6 = c6;
}

//---------------------------------------------------------------------
float sof_f::compute(float dt, float u)
{
    if ( dt > 0.0 )
    {
        float den = 4.0*_c4 + 2.0*_c5*dt + _c6*dt*dt;
        float dt2 = dt * dt;
        float ca = ( 4.0 * _c1       + 2.0 * _c2 * dt + _c3 * dt2 ) / den;
        float cb = ( 2.0 * _c3 * dt2 - 8.0 * _c1                  ) / den;
        float cc = ( 4.0 * _c1       - 2.0 * _c2 * dt + _c3 * dt2 ) / den;
        float cd = ( 2.0 * _c6 * dt2 - 8.0 * _c4                  ) / den;
        float ce = ( 4.0 * _c4       - 2.0 * _c5 * dt + _c6 * dt2 ) / den;
        _y = u * ca + _u_prev_1 * cb + _u_prev_2 * cc
                    - _y_prev_1 * cd - _y_prev_2 * ce;
        _u_prev_2 = _u_prev_1;
        _u_prev_1 = u;
        _y_prev_2 = _y_prev_1;
        _y_prev_1 = _y;
    }
    return _y;
}
