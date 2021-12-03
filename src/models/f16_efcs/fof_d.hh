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

#ifndef __FIRST_ORDER_FILTER_D_HH__
#define __FIRST_ORDER_FILTER_D_HH__

// FOF: First-Order Filter with transfer function
// G(s)  =  1 / ( Tc*s + 1 )

class fof_d
{
	public:
	
		fof_d();
		~fof_d();
		void set_tc( double tc );
		void set_y( double y );
		double get_y();
		double compute( double dt, double u );

	private:
	
		double _tc; // filter time constant
		double _y;  // current value for the filter that is updated each dt cycle
};

#endif 
