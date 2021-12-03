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

#ifndef __PID_D_HH__
#define __PID_D_HH__

class pid_d
{
	public:
		pid_d();

		~pid_d();
		void set_parameters( double kp
		                  , double Ts
		                  , double Ti
		                  , double Td
		                  , double Alpha
		                  , double Beta
		                  , double Gamma
		                  , double Min
		                  , double Max
		                  );
		double compute(double input, double ref);

	private: 
		
		double _kp; 
		double _ep, _ep_l; 
		double _ed, _edL; 
		double _en;
		double _edf, _edfL, _edfLL;
		double _delta_u;
		double _output; 
		double _outputL; 
		double _beta, _alpha, _gamma;
		double _ts, _ti, _td, _tf;
		double _max; // Max Saturation
		double _min; // Min Saturation		
};

#endif
