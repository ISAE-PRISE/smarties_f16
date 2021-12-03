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

#include <cmath>
#include <fof_d.hh>

//---------------------------------------------------------------------
fof_d::fof_d()
{
	_tc = 1.0; // init value
	_y = 0.0; // init value
}

//---------------------------------------------------------------------
fof_d::~fof_d()
{
	// Nothing to do
}

//---------------------------------------------------------------------
void fof_d::set_tc( double tc )
{
    if ( tc > 0.0 )
    {
        _tc = tc;
    }
    else
    {
		_tc = 1;
	}
}

//---------------------------------------------------------------------
void fof_d::set_y( double y )
{
    _y = y;
}

//---------------------------------------------------------------------
double fof_d::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
double fof_d::compute( double dt, double u )
{
	_y = _y + ( 1.0 - exp( -dt / _tc ) ) * ( u - _y );
    return _y;
}
