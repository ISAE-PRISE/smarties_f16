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

#ifndef __F16_ENGINE_DATA_F_HH__
#define __F16_ENGINE_DATA_F_HH__

#ifdef __cplusplus
extern "C" 
{
#endif


// Table mach for lines and altitude_meters for columns
// Document 1 (TP-1538) page 93
const float thrust_idle_mach_alt_d1_f[6][7] = 
{
	{0.0,	   0.0,	 3048.0,	6096.0,	  9144.0,  12192.0,	15240.0},
	{0.2,	2824.0,	 1890.0,	3069.0,	  4492.0,	5916.0,	 7562.0},
	{0.4,    267.0,	  111.0,	1535.0,	  3358.0,	5026.0,	 6783.0},
	{0.6,  -4537.0,	-3158.0,   -1334.0,	  1557.0,	4048.0,	 6049.0},
	{0.8, -12010.0,	-8451.0,   -5782.0,	 -1099.0,	2669.0,	 4893.0},
	{1.0, -16013.0,	-6227.0,   -2647.0,	 -1521.0,	-890.0,	 3114.0}
};

// Table mach for lines and altitude_meters for columns
// Document 1 (TP-1538) page 93
const float thrust_mil_mach_alt_d1_f[6][7] = 
{
	{0.0,	   0.0,	 3048.0,	6096.0,	  9144.0,  12192.0,	15240.0},
	{0.2,  56401.0,	40699.0,   28080.0,	 17970.0,  10987.0,	 6227.0},
	{0.4,  56089.0,	41420.0,   29401.0,	 19082.0,  11585.0,	 6939.0},
	{0.6,  56223.0,	43764.0,   31536.0,	 20738.0,  12632.0,	 7384.0},
	{0.8,  55111.0,	45263.0,   34472.0,	 23663.0,  14456.0,	 8585.0},
	{1.0,  51953.0,	43804.0,   35806.0,	 27133.0,  16902.0,	10275.0}
};

// Table mach for lines and altitude_meters for columns
// Document 1 (TP-1538) page 93
const float thrust_max_mach_alt_d1_f[6][7] = 
{
	{0.0,	   0.0,	 3048.0,	6096.0,	  9144.0,  12192.0,	15240.0},
	{0.2,  95276.0,	69834.0,   49929.0,	 32573.0,  19727.0,	11565.0},
	{0.4, 100970.0,	74993.0,   54488.0,	 36269.0,  22240.0,	12610.0},
	{0.6, 107820.0,	84112.0,   61204.0,	 41300.0,  25354.0,	14300.0},
	{0.8, 115959.0,	93742.0,   71057.0,	 49440.0,  30513.0,	17570.0},
	{1.0, 128485.0,103723.0,   81398.0,	 59977.0,  38440.0,	22494.0}
};



#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __F16_ENGINE_DATA_F_HH__
