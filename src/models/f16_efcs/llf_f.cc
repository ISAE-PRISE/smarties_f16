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
#include <llf_f.hh>

//---------------------------------------------------------------------
llf_f::llf_f() 
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
llf_f::~llf_f() 
{
	// Nothing to do
}

//---------------------------------------------------------------------
void llf_f::set_y( float y )
{
    _y = y;
}

//---------------------------------------------------------------------
float llf_f::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
void llf_f::set_c1to4(float c1, float c2, float c3, float c4)
{
	_c1 = c1;
    _c2 = c2;
    _c3 = c3;
    _c4 = c4;
}


//---------------------------------------------------------------------
float llf_f::compute( float dt, float u )
{
    if ( dt > 0.0 )
    {
        float den = 2.0 * _c3 + dt * _c4;

        float ca = ( 2.0 * _c1 + dt  * _c2 ) / den;
        float cb = ( dt  * _c2 - 2.0 * _c1 ) / den;
        float cc = ( 2.0 * _c3 - dt  * _c4 ) / den;

        _y = u * ca + _u_prev * cb + _y_prev * cc;

        _u_prev = u;
        _y_prev = _y;
    }
    return _y;
}
