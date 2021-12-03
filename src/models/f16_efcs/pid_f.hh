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

#ifndef __PID_F_HH__
#define __PID_F_HH__

class pid_f
{
	public:
		pid_f();

		~pid_f();
		void set_parameters( float kp
		                  , float Ts
		                  , float Ti
		                  , float Td
		                  , float Alpha
		                  , float Beta
		                  , float Gamma
		                  , float Min
		                  , float Max
		                  );
		float compute(float input, float ref);

	private: 
		
		float _kp; 
		float _ep, _ep_l; 
		float _ed, _edL; 
		float _en;
		float _edf, _edfL, _edfLL;
		float _delta_u;
		float _output; 
		float _outputL; 
		float _beta, _alpha, _gamma;
		float _ts, _ti, _td, _tf;
		float _max; // Max Saturation
		float _min; // Min Saturation		
};

#endif
