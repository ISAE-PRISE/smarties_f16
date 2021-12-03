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

#ifndef __LEAD_LAG_D_HH__
#define __LEAD_LAG_D_HH__

// LLF: Lead-Lag Filter with transfer function
// G(s)  =  ( c1*s + c2 ) / ( c3*s + c4 )

class llf_d
{
	public:
	
		llf_d();
		~llf_d();
		void set_c1to4(double c1, double c2, double c3, double c4);
		void set_y(double y);
		double get_y();
		double compute(double dt, double u);

	private:
	
		double _c1;           
		double _c2;             
		double _c3;             
		double _c4;            
		double _u_prev;         
		double _y_prev;         
		double _y;      
};

#endif 
