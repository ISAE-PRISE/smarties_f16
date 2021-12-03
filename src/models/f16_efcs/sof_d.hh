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

#ifndef __SECOND_ORDER_FILTER_D_HH__
#define __SECOND_ORDER_FILTER_D_HH__

// SOF: Second-Order Filter with transfer function
// G(s)  =  ( c1*s^2 + c2*s + c3 ) / ( c4*s^2 + c5*s + c6 )

class sof_d
{
	public:

		sof_d();
		~sof_d();
		void set_c1to6(double c1, double c2, double c3, double c4, double c5, double c6);
		void set_y(double y);
		double get_y();
		double compute(double dt, double u);

	private:

		double _c1;
		double _c2;
		double _c3;
		double _c4;
		double _c5;
		double _c6;
		double _u_prev_1;
		double _u_prev_2;
		double _y_prev_1;
		double _y_prev_2;
		double _y;
};

#endif 
