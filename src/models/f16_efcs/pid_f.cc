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

#include <pid_f.hh>

//----------------------------------------------------------------------
// Constructor
pid_f::pid_f()
{
	_kp = 0.0; 		
	_ep = 0.0;
	_ep_l= 0.0;
	_ed = 0.0;
	_edL= 0.0;
	_en = 0.0;
	_edf = 0.0; 
	_edfL = 0.0;
	_edfLL = 0.0;	
	_output = 0.0; 
	_outputL = 0.0;	
	_beta = 1.0; 
	_alpha = 0.1; 
	_gamma = 0.0;
	_ts = 0.0; 
	_ti = 0.0; 
	_td = 0.0;
	_tf = 0.0;
	_delta_u = 0.0;
	_max = 0.0; 
	_min = 0.0; 
}

//----------------------------------------------------------------------
// Destructor
pid_f::~pid_f()
{
	// Nothing to do here
}

//----------------------------------------------------------------------
// function to set correct value for Controller
void pid_f::set_parameters( float kp
		                         , float Ts
		                         , float Ti
		                         , float Td
		                         , float Alpha
		                         , float Beta
		                         , float Gamma
		                         , float Min
		                         , float Max
		                         )
{
	_kp = kp; 		
	_ts = Ts; 
	_ti = Ti; 
	_td = Td;
	_alpha = Alpha; 
	_beta = Beta; 
	_gamma = Gamma;
	_tf = _alpha * _ts;
	_min = Min;
	_max = Max;
}

//----------------------------------------------------------------------
// Function: Calculate the PID controller output value for a given entry
float pid_f::compute(float input, float ref)
{
	_ep = _beta * ref - input;
	_en = ref - input;
	_ed = _gamma * ref - input;
	_edf = _edfL / ((_ts/_tf) + 1) + _ed * ((_ts/_tf) / ((_ts/_tf) + 1));
	_delta_u = _kp * ( (_ep - _ep_l) + ((_ts/_ti) * _en) + ((_td/_ts) * (_edf - 2.0*_edfL + _edfLL)));
	_output = _outputL + _delta_u;
	
	// Saturation
	if     (_output > _max)
	{
		_output = _max;
	}
    else if(_output < _min) 
    {
		_output = _min;
	}
	
	_ep_l = _ep;
	_edL = _ed;
	_edfL = _edf;
	_edfLL = _edfL;
	_outputL = _output;
	
	return _output;
}
