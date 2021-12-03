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
#include <fof_f.hh>

//---------------------------------------------------------------------
fof_f::fof_f()
{
	_tc = 1.0; // init value
	_y = 0.0; // init value
}

//---------------------------------------------------------------------
fof_f::~fof_f()
{
	// Nothing to do
}

//---------------------------------------------------------------------
void fof_f::set_tc( float tc )
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
void fof_f::set_y( float y )
{
    _y = y;
}

//---------------------------------------------------------------------
float fof_f::get_y()
{
    return _y ;
}

//---------------------------------------------------------------------
float fof_f::compute( float dt, float u )
{
	_y = _y + ( 1.0 - exp( -dt / _tc ) ) * ( u - _y );
    return _y;
}
