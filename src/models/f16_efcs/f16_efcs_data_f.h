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

// The Tables are all taken from the following documents:
// DOCUMENT 1:
// https://ntrs.nasa.gov/api/citations/19800005879/downloads/19800005879.pdf
// 
// Title: SIMULATOR STUDY OF STALL/POST-STALL CHARACTERISTICS OF A FIGHTER
// AIRPLANE WITH RELAXED LONGITUDINAL STABILITY
// By: Luat T.Nguyen, Marilyn E.Ogburn, William P.Gilbert, Kemper S.Kibler, 
//     Phillip W.Brown, and Perry L.Deal
// Ref: Nasa Technical Paper 1538 (Noted TP-1538 in this header then)
// Date: December 1979
// 
// --

#ifndef __F16_AUTOPILOT_DATA_F_HH__
#define __F16_AUTOPILOT_DATA_F_HH__

#ifdef __cplusplus
extern "C" 
{
#endif


// Document 1 (TP-1538) page 212
// F-long in Newton has been normalized between -1 and 1 to match Joystick input  
const float pitch_cmd_d1_f[8][2] = 
{
	{-1.000,   10.86},
	{-0.181,    0.44},
	{-0.044,    0.00},
	{ 0.000,    0.00},
	{ 0.044,    0.00},
	{ 0.181,   -0.44},
	{ 0.441,   -4.00},
	{ 1.000,    -4.0}
};

// Table alpha_deg for lines and beta_deg for columns
// Document 1 (TP-1538) page 211
const float pitch_trim_d1_f[3][2] = 
{
	{-1.0,      2.4},
	{ 0.0,      0.0},
	{ 1.0,     -2.4}
};

// Document 1 (TP-1538) page 215
// F-lat in Newton has been normalized between -1 and 1 to match Joystick input  
const float roll_cmd_d1_f[9][2] = 
{
	{-1.000,  308.0},
	{-0.647,   80.0},
	{-0.353,   20.0},
	{-0.059,    0.0},
	{ 0.000,    0.0},
	{ 0.059,    0.0},
	{ 0.353,  -20.0},
	{ 0.647,  -80.0},
	{ 1.000, -308.0}
};

// Document 1 (TP-1538) page 214
const float roll_trim_d1_f[3][2] = 
{
	{-1.0,     40.0},
	{ 0.0,      0.0},
	{ 1.0,    -40.0}
};

// Document 1 (TP-1538) page 215
// F-lat in Newton has been normalized between -1 and 1 to match Joystick input  
const float yaw_cmd_d1_f[5][2] = 
{
	{-1.000,  -30.0},
	{-0.125,    0.0},
	{ 0.000,    0.0},
	{ 0.125,    0.0},
	{ 1.000,   30.0}
};

// Document 1 (TP-1538) page 214
const float yaw_trim_d1_f[3][2] = 
{
	{-1.0,      8.0},
	{ 0.0,      0.0},
	{ 1.0,     -8.0}
};




#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __F16_AUTOPILOT_DATA_F_HH__
