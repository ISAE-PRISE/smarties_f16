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

#ifndef __sof_f_HH__
#define __sof_f_HH__

// SOF: Second-Order Filter with transfer function
// G(s)  =  ( c1*s^2 + c2*s + c3 ) / ( c4*s^2 + c5*s + c6 )

class sof_f
{
	public:

		sof_f();
		~sof_f();
		void set_c1to6(float c1, float c2, float c3, float c4, float c5, float c6);
		void set_y(float y);
		float get_y();
		float compute(float dt, float u);

	private:

		float _c1;
		float _c2;
		float _c3;
		float _c4;
		float _c5;
		float _c6;
		float _u_prev_1;
		float _u_prev_2;
		float _y_prev_1;
		float _y_prev_2;
		float _y;
};

#endif 
