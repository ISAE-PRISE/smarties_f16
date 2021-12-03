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

#ifndef __F16_DATA_TYPES_F_HH__
#define __F16_DATA_TYPES_F_HH__

#ifdef __cplusplus
extern "C" 
{
#endif

typedef struct f16_fcs_pitch_data_f 
{ 
	float v1; 
	float v2; 
	float v3; 
	float v4;
	float v5; 
	float v6;
	float v7;
} 
f16_fcs_pitch_data_f_t;

typedef struct f16_fcs_cmd_f 
{ 
	float der; 
	float del; 
	float dar; 
	float dal;
	float dlefr; 
	float dlefl;
	float dtefr; 
	float dtefl;
	float dr;
	float dth;
} 
f16_fcs_cmd_f_t;

typedef struct f16_cs_f 
{ 
	float der; 
	float del; 
	float dar; 
	float dal;
	float dlefr; 
	float dlefl;
	float dtefr; 
	float dtefl;
	float dr;
} 
f16_cs_f_t;

typedef struct f16_fcu_f 
{ 
	bool ap_en;
	bool athr_en;  
	bool fd_en; 
	bool fbw_en;
	bool man_en; 
	float alt_ref; 
	float psi_ref;
	float vs_ref; 
	float vt_ref;
	float mach_ref;
} 
f16_fcu_f_t;

typedef struct f16_sst_f 
{ 
	float pitch_cmd; 
	float roll_cmd;
	float yaw_cmd; 
	float thr_cmd;
} 
f16_sst_f_t;


typedef struct f16_trim_c_f 
{ 
	float vt ;		
	float npos ;
	float epos ;
	float alt ;
	float p ;
	float q ;
	float r ;
	float psi ;
} 
f16_trim_c_f_t;

typedef struct f16_trim_v_f 
{ 
	float thrust ;		
	float alpha ;
	float de ;
	float da ;
	float dr ;
	float dth ;
} 
f16_trim_v_f_t;

typedef struct f16_mi_f 
{ 
	float mass ;		
	float ixx ;
	float iyy ;
	float izz ;
	float ixz ;
	float sref;
	float bref ;
	float cref ;
	float xcg ;
	float xcgr;
	float heng;
} 
f16_mi_f_t;

typedef struct f16_u_f 
{ 				
	float dth;			
	float de;
	float de_norm;			
	float da;
	float da_norm;			
	float dr;
	float dr_norm;			
	float dlef;	
	float dlef_norm;
	float dtef;
	float dtef_norm;	
} 
f16_u_f_t;

typedef struct f16_xdot_f 
{ 
	// NASA TP-1538 page 36
	float u_dot;
	float v_dot;
	float w_dot;
	float p_dot;
	float q_dot;
	float r_dot;
	float npos_dot;
	float epos_dot;
	double lon_dot;
	double lat_dot;
	float alt_dot;
	float phi_dot;
	float theta_dot;
	float psi_dot;
	float q0_dot;
	float q1_dot;
	float q2_dot;
	float q3_dot;
	float vt_dot;
	float alpha_dot;
	float beta_dot;
	// engine
	float power_dot;
} 
f16_xdot_f_t;

typedef struct f16_x_f 
{ 
	float time_sec;
	// NASA TP-1538 page 36
	float u;
	float v;
	float w;
	float p;
	float q;
	float r;
	float npos;
	float epos;
	double lon;
	double lat;
	float alt; // sea level
	float phi;
	float theta;
	float psi;
	float q0;
	float q1;
	float q2;
	float q3;
	float vt;
	float vs;
	float alpha;
	float beta;
	float nx;
	float ny;
	float nz;
	// Atmos
	float mach;
	float qbar;
	float ps;
	// Engine
	float thrust;
	float power;
	// Aero coef/moments
	float cx;
	float cy;
	float cz;
	float cl;
	float cm;
	float cn;
	
} 
f16_x_f_t;


typedef struct f16_cockpit_data_f
{ 
	f16_sst_f_t sst; 
	f16_fcu_f_t fcu;
} 
f16_cockpit_data_f_t;

typedef struct f16_engine_data_f
{ 
	float thrust; 
	float fuelflow;
} 
f16_engine_data_f_t;


typedef struct f16_fdm_data_f 
{ 
	f16_x_f_t x;
	f16_xdot_f_t xdot;
	f16_u_f_t u;
	f16_mi_f_t mi;
	f16_fcs_cmd_f_t fcs;
	f16_cs_f_t cs;
} 
f16_fdm_data_f_t;


#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __F16_DATA_TYPES_F_HH__
