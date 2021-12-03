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

#ifndef __FIRST_ORDER_FILTER_F_HH__
#define __FIRST_ORDER_FILTER_F_HH__

// FOF: First-Order Filter with transfer function
// G(s)  =  1 / ( Tc*s + 1 )

class fof_f
{
	public:
	
		fof_f();
		~fof_f();
		void set_tc( float tc );
		void set_y( float y );
		float get_y();
		float compute( float dt, float u );

	private:
	
		float _tc; // filter time constant
		float _y;  // current value for the filter that is updated each dt cycle
};

#endif 
