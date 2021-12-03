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

#ifndef __llf_f_HH__
#define __llf_f_HH__

// LLF: Lead-Lag Filter with transfer function
// G(s)  =  ( c1*s + c2 ) / ( c3*s + c4 )

class llf_f
{
	public:
	
		llf_f();
		~llf_f();
		void set_c1to4(float c1, float c2, float c3, float c4);
		void set_y(float y);
		float get_y();
		float compute(float dt, float u);

	private:
	
		float _c1;           
		float _c2;             
		float _c3;             
		float _c4;            
		float _u_prev;         
		float _y_prev;         
		float _y;      
};

#endif 
