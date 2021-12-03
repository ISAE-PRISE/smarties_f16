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

#include "f16_fdm_f.hh"

//---------------------------------------------------------------------
f16_fdm_f::f16_fdm_f()
{
	// Global struc variables
	memset(&_xdot_old1, 0, sizeof(_xdot_old1));
	memset(&_xdot_old2, 0, sizeof(_xdot_old2));
	memset(&_trim_cond, 0, sizeof(_trim_cond));
	memset(&_cockpit_in, 0, sizeof(_cockpit_in));
	memset(&_fdm.fcs, 0, sizeof(_fdm.fcs));
	
	memset(&_fdm, 0, sizeof(_fdm));
	memset(&_fdm_trap, 0, sizeof(_fdm_trap));
	memset(&_fdm_adba2, 0, sizeof(_fdm_adba2));
	memset(&_fdm_adba3, 0, sizeof(_fdm_adba3));
	
	_fdm.mi.mass = 9295.44; // kg                                 
	_fdm.mi.ixx = 12874.8;     //   [kg/m2]	 
	_fdm.mi.iyy = 75673.6;  //   [kg/m2]
	_fdm.mi.izz = 85552.1;    //   [kg/m2]
	_fdm.mi.ixz = 1331.4 ;    //   [kg/m2]
	_fdm.mi.sref = 27.87;      //  [m2]
	_fdm.mi.bref = 9.144;      //  [m]	
	_fdm.mi.cref =  3.45;     //   [m]	 
	_fdm.mi.xcg =    0.3;  // [%]	 
	_fdm.mi.xcgr =  0.35; //    [%]	 */
	_fdm.mi.heng = 0.0; /* engine angular momentum, assumed fixed */ //160 slug-ft^2/s
	
	
	// Set all tables to zeros
	// ------------- CX -------------------
	memset(&_CX_alpha1_beta_dh_n25_d1_f, 0, sizeof(_CX_alpha1_beta_dh_n25_d1_f));
	memset(&_CX_alpha1_beta_dh_n10_d1_f, 0, sizeof(_CX_alpha1_beta_dh_n10_d1_f));
	memset(&_CX_alpha1_beta_dh_0_d1_f, 0, sizeof(_CX_alpha1_beta_dh_0_d1_f));
	memset(&_CX_alpha1_beta_dh_p10_d1_f, 0, sizeof(_CX_alpha1_beta_dh_p10_d1_f));
	memset(&_CX_alpha1_beta_dh_p25_d1_f, 0, sizeof(_CX_alpha1_beta_dh_p25_d1_f));
	memset(&_CX_lef_alpha2_beta_d1_f, 0, sizeof(_CX_lef_alpha2_beta_d1_f));
	memset(&_CX_q_alpha1_d1_f, 0, sizeof(_CX_q_alpha1_d1_f));
	memset(&_dCX_sb_alpha1_d1_f, 0, sizeof(_dCX_sb_alpha1_d1_f)); 
	memset(&_dCX_q_lef_alpha2_d1_f, 0, sizeof(_dCX_q_lef_alpha2_d1_f)); 
	memset(&_dCX_tef_alpha1, 0, sizeof(_dCX_tef_alpha1)); 
	
	// ------------- CZ -------------------
	memset(&_CZ_alpha1_beta_dh_n25_d1_f, 0, sizeof(_CZ_alpha1_beta_dh_n25_d1_f));
	memset(&_CZ_alpha1_beta_dh_n10_d1_f, 0, sizeof(_CZ_alpha1_beta_dh_n10_d1_f));
	memset(&_CZ_alpha1_beta_dh_0_d1_f, 0, sizeof(_CZ_alpha1_beta_dh_0_d1_f));
	memset(&_CZ_alpha1_beta_dh_p10_d1_f, 0, sizeof(_CZ_alpha1_beta_dh_p10_d1_f));
	memset(&_CZ_alpha1_beta_dh_p25_d1_f, 0, sizeof(_CZ_alpha1_beta_dh_p25_d1_f));
	memset(&_CZ_lef_alpha2_beta_d1_f, 0, sizeof(_CZ_lef_alpha2_beta_d1_f));
	memset(&_CZ_q_alpha1_d1_f, 0, sizeof(_CZ_q_alpha1_d1_f));
	memset(&_dCZ_sb_alpha1_d1_f, 0, sizeof(_dCZ_sb_alpha1_d1_f)); 
	memset(&_dCZ_q_lef_alpha2_d1_f, 0, sizeof(_dCZ_q_lef_alpha2_d1_f));
	memset(&_dCZ_tef_alpha1, 0, sizeof(_dCZ_tef_alpha1)); 
	
	// ------------- CY -------------------
	memset(&_CY_alpha1_beta_d1_f, 0, sizeof(_CY_alpha1_beta_d1_f));
	memset(&_CY_lef_alpha2_beta_d1_f, 0, sizeof(_CY_lef_alpha2_beta_d1_f));
	memset(&_CY_alpha1_beta_da_20_d1_f, 0, sizeof(_CY_alpha1_beta_da_20_d1_f));
	memset(&_CY_lef_alpha2_beta_da_20_d1_f, 0, sizeof(_CY_lef_alpha2_beta_da_20_d1_f));
	memset(&_CY_alpha1_beta_dr_30_d1_f, 0, sizeof(_CY_alpha1_beta_dr_30_d1_f));
	// WARNING: Correct use of _CY_XX instead of proper _dCY_XX
	memset(&_CY_r_alpha1_d1_f, 0, sizeof(_CY_r_alpha1_d1_f));
	memset(&_dCY_r_lef_alpha2_d1_f, 0, sizeof(_dCY_r_lef_alpha2_d1_f));
	memset(&_CY_p_alpha1_d1_f, 0, sizeof(_CY_p_alpha1_d1_f));
	memset(&_dCY_p_lef_alpha2_d1_f, 0, sizeof(_dCY_p_lef_alpha2_d1_f));
	// ------------- Cn -------------------
	memset(&_Cn_alpha1_beta_dh_n25_d1_f, 0, sizeof(_Cn_alpha1_beta_dh_n25_d1_f));
	memset(&_Cn_alpha1_beta_dh_0_d1_f, 0, sizeof(_Cn_alpha1_beta_dh_0_d1_f));
	memset(&_Cn_alpha1_beta_dh_p25_d1_f, 0, sizeof(_Cn_alpha1_beta_dh_p25_d1_f));
	memset(&_Cn_lef_alpha2_beta_d1_f, 0, sizeof(_Cn_lef_alpha2_beta_d1_f));
	memset(&_Cn_alpha1_beta_da_20_d1_f, 0, sizeof(_Cn_alpha1_beta_da_20_d1_f));
	memset(&_Cn_lef_alpha2_beta_da_20_d1_f, 0, sizeof(_Cn_lef_alpha2_beta_da_20_d1_f));
	memset(&_Cn_r_alpha1_d1_f, 0, sizeof(_Cn_r_alpha1_d1_f));
	memset(&_dCn_beta_alpha1_d1_f, 0, sizeof(_dCn_beta_alpha1_d1_f));
	memset(&_dCn_da_alpha1_d1_f, 0, sizeof(_dCn_da_alpha1_d1_f));
	memset(&_dCn_r_lef_alpha2_d1_f, 0, sizeof(_dCn_r_lef_alpha2_d1_f));
	memset(&_Cn_p_alpha1_d1_f, 0, sizeof(_Cn_p_alpha1_d1_f));
	memset(&_dCn_p_lef_alpha2_d1_f, 0, sizeof(_dCn_p_lef_alpha2_d1_f));
	
	// ------------- Cl -------------------
	memset(&_Cl_alpha1_beta_dh_n25_d1_f, 0, sizeof(_Cl_alpha1_beta_dh_n25_d1_f));
	memset(&_Cl_alpha1_beta_dh_0_d1_f, 0, sizeof(_Cl_alpha1_beta_dh_0_d1_f));
	memset(&_Cl_alpha1_beta_dh_p25_d1_f, 0, sizeof(_Cl_alpha1_beta_dh_p25_d1_f));
	memset(&_Cl_lef_alpha2_beta_d1_f, 0, sizeof(_Cl_lef_alpha2_beta_d1_f));
	memset(&_Cl_alpha1_beta_da_20_d1_f, 0, sizeof(_Cl_alpha1_beta_da_20_d1_f));
	memset(&_Cl_lef_alpha2_beta_da_20_d1_f, 0, sizeof(_Cl_lef_alpha2_beta_da_20_d1_f));
	memset(&_Cl_alpha1_beta_dr_30_d1_f, 0, sizeof(_Cl_alpha1_beta_dr_30_d1_f));
	memset(&_Cl_r_alpha1_d1_f, 0, sizeof(_CY_r_alpha1_d1_f));
	memset(&_dCl_beta_alpha1_d1_f, 0, sizeof(_dCl_beta_alpha1_d1_f));
	memset(&_dCl_r_lef_alpha2_d1_f, 0, sizeof(_dCl_r_lef_alpha2_d1_f));
	memset(&_Cl_p_alpha1_d1_f, 0, sizeof(_Cl_p_alpha1_d1_f));
	memset(&_dCl_p_lef_alpha2_d1_f, 0, sizeof(_dCl_p_lef_alpha2_d1_f));
	
	// ------------- Cm -------------------
	memset(&_Cm_alpha1_beta_dh_n25_d1_f, 0, sizeof(_Cm_alpha1_beta_dh_n25_d1_f));
	memset(&_Cm_alpha1_beta_dh_n10_d1_f, 0, sizeof(_Cm_alpha1_beta_dh_n10_d1_f));
	memset(&_Cm_alpha1_beta_dh_0_d1_f, 0, sizeof(_Cm_alpha1_beta_dh_0_d1_f));
	memset(&_Cm_alpha1_beta_dh_p10_d1_f, 0, sizeof(_Cm_alpha1_beta_dh_p10_d1_f));
	memset(&_Cm_alpha1_beta_dh_p25_d1_f, 0, sizeof(_Cm_alpha1_beta_dh_p25_d1_f));
	memset(&_Cm_lef_alpha2_beta_d1_f, 0, sizeof(_Cm_lef_alpha2_beta_d1_f));
	memset(&_dCm_sb_alpha1_d1_f, 0, sizeof(_dCm_sb_alpha1_d1_f));
	memset(&_dCm_alpha1_d1_f, 0, sizeof(_dCm_alpha1_d1_f));
	memset(&_eta_dh_d1_f, 0, sizeof(_eta_dh_d1_f));
	memset(&_Cm_q_alpha1_d1_f, 0, sizeof(_Cm_q_alpha1_d1_f));
	memset(&_dCm_q_lef_alpha2_d1_f, 0, sizeof(_dCm_q_lef_alpha2_d1_f));
	memset(&_dCm_alpha1_ds_d1_f, 0, sizeof(_dCm_alpha1_ds_d1_f));
	memset(&_dCm_tef_alpha1, 0, sizeof(_dCm_tef_alpha1)); 
	
	// ------------- Engines -------------------
	memset(&_thrust_idle_mach_alt_d1_f, 0, sizeof(_thrust_idle_mach_alt_d1_f));
	memset(&_thrust_mil_mach_alt_d1_f, 0, sizeof(_thrust_mil_mach_alt_d1_f));
	memset(&_thrust_max_mach_alt_d1_f, 0, sizeof(_thrust_max_mach_alt_d1_f));
	_pa = _pc = _thrust =  0.0;
	
	// ------------- Ctrl surfaces -------------------
	memset(&_fdm.fcs, 0, sizeof(_fdm.fcs));
	
	
	//control surfaces, first order lag filter
	_tc_ele_r = 0.0495; // doc pdf page 40
	_pos_ele_r = 0.0; 
	_min_ele_r = -25.0;
	_max_ele_r = 25.0; // deg
	_max_rate_ele_r = 60.0; //deg/s
	_tc_ele_l = 0.0495; // doc pdf page 40
	_pos_ele_l = 0.0; 
	_min_ele_l = -25.0;
	_max_ele_l = 25.0; // deg
	_max_rate_ele_l = 60.0; //deg/s
	
	_tc_ail_r = 0.0495; // doc pdf page 40
	_pos_ail_r = 0.0; 
	_min_ail_r = -21.5;
	_max_ail_r = 21.5; // deg
	_max_rate_ail_r = 80.0; //deg/s
	_tc_ail_l = 0.0495; // doc pdf page 40
	_pos_ail_l = 0.0; 
	_min_ail_l = -21.5;
	_max_ail_l = 21.5; // deg
	_max_rate_ail_l = 80.0; //deg/s
	
	_tc_lef_r = 0.136; // doc pdf page 40
	_pos_lef_r = 0.0; 
	_min_lef_r = 0.0;
	_max_lef_r = 25.0; // deg
	_max_rate_lef_r = 25.0; //deg/s
	_tc_lef_l = 0.136; // doc pdf page 40
	_pos_lef_l = 0.0; 
	_min_lef_l = 0.0;
	_max_lef_l = 25.0; // deg
	_max_rate_lef_l = 25.0; //deg/s
	
	_tc_tef_r = 0.136; // doc pdf page 40
	_pos_tef_r = 0.0; 
	_min_tef_r = -21.5;
	_max_tef_r = 21.5; // deg
	_max_rate_tef_r = 25.0; //deg/s
	_tc_tef_l = 0.136; // doc pdf page 40
	_pos_tef_l = 0.0; 
	_min_tef_l = -21.5;
	_max_tef_l = 21.5; // deg
	_max_rate_tef_l = 25.0; //deg/s
	
	_tc_rud = 0.0495; // doc pdf page 40
	_pos_rud = 0.0; 
	_min_rud = -30.0;
	_max_rud = 30.0; // deg
	_max_rate_rud = 120.0; //deg/s
	
	_r2d = 57.29577951;
	_d2r = 0.017453293;
	
	// Lat/long init (TODO:Move this!!!)
	_fdm.x.lat = 37.6262 * _d2r;
	_fdm.x.lon = -122.3919 * _d2r;
}

//---------------------------------------------------------------------
f16_fdm_f::~f16_fdm_f()
{

}

//---------------------------------------------------------------------
//
f16_x_f_t f16_fdm_f::get_x()
{
	return _fdm.x;
}

//---------------------------------------------------------------------
//
f16_fdm_data_f_t f16_fdm_f::get_fdm()
{
	return _fdm;
}

//---------------------------------------------------------------------
//
f16_fdm_data_f_t f16_fdm_f::get_fdm_euler()
{
	return _fdm_euler;
}

//---------------------------------------------------------------------
//
f16_fdm_data_f_t f16_fdm_f::get_fdm_trap()
{
	return _fdm_trap;
}

//---------------------------------------------------------------------
//
f16_fdm_data_f_t f16_fdm_f::get_fdm_adba2()
{
	return _fdm_adba2;
}

//---------------------------------------------------------------------
//
f16_fdm_data_f_t f16_fdm_f::get_fdm_adba3()
{
	return _fdm_adba3;
}

//---------------------------------------------------------------------
//
void f16_fdm_f::set_trim_conditions( float vt_trim
                                   , float npos_trim
                                   , float epos_trim
                                   , float alt_trim
                                   , float p_trim
                                   , float q_trim
                                   , float r_trim
                                   , float psi_trim)
{
	_trim_cond.vt = vt_trim;		
	_trim_cond.npos = npos_trim;
	_trim_cond.epos = epos_trim;
	_trim_cond.alt = alt_trim;
	_trim_cond.p = p_trim;
	_trim_cond.q = q_trim;
	_trim_cond.r = r_trim;
	_trim_cond.psi =  psi_trim;
}

//---------------------------------------------------------------------
//
void f16_fdm_f::set_trim_init( float thrust_init
						  , float alpha_init
                          , float de_init
                          , float da_init
                          , float dr_init
                          , float dth_init)
{

	_trim_init.thrust = thrust_init;		
	_trim_init.alpha = alpha_init;
	_trim_init.de = de_init;
	_trim_init.da = da_init;
	_trim_init.dr = dr_init;
	_trim_init.dth = dth_init;
}

//---------------------------------------------------------------------
//
void f16_fdm_f::load_tables_aero_f()
{
	int i,j;
	// ------------- CX -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_alpha1_beta_dh_n25_d1_f[i][j] = CX_alpha1_beta_dh_n25_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_alpha1_beta_dh_n10_d1_f[i][j] = CX_alpha1_beta_dh_n10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_alpha1_beta_dh_0_d1_f[i][j] = CX_alpha1_beta_dh_0_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_alpha1_beta_dh_p10_d1_f[i][j] = CX_alpha1_beta_dh_p10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_alpha1_beta_dh_p25_d1_f[i][j] = CX_alpha1_beta_dh_p25_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CX_lef_alpha2_beta_d1_f[i][j] = CX_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_CX_q_alpha1_d1_f[i][j] = CX_q_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCX_sb_alpha1_d1_f[i][j] = dCX_sb_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCX_q_lef_alpha2_d1_f[i][j] = dCX_q_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCX_tef_alpha1[i][j] = dCX_tef_alpha1[i][j];
		}
	}
	
	// ------------- CZ -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_alpha1_beta_dh_n25_d1_f[i][j] = CZ_alpha1_beta_dh_n25_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_alpha1_beta_dh_n10_d1_f[i][j] = CZ_alpha1_beta_dh_n10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_alpha1_beta_dh_0_d1_f[i][j] = CZ_alpha1_beta_dh_0_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_alpha1_beta_dh_p10_d1_f[i][j] = CZ_alpha1_beta_dh_p10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_alpha1_beta_dh_p25_d1_f[i][j] = CZ_alpha1_beta_dh_p25_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CZ_lef_alpha2_beta_d1_f[i][j] = CZ_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_CZ_q_alpha1_d1_f[i][j] = CZ_q_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCZ_sb_alpha1_d1_f[i][j] = dCZ_sb_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCZ_q_lef_alpha2_d1_f[i][j] = dCZ_q_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCZ_tef_alpha1[i][j] = dCZ_tef_alpha1[i][j];
		}
	}
	
	// ------------- CY -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CY_alpha1_beta_d1_f[i][j] = CY_alpha1_beta_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CY_lef_alpha2_beta_d1_f[i][j] = CY_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CY_alpha1_beta_da_20_d1_f[i][j] = CY_alpha1_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CY_lef_alpha2_beta_da_20_d1_f[i][j] = CY_lef_alpha2_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_CY_alpha1_beta_dr_30_d1_f[i][j] = CY_alpha1_beta_dr_30_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_CY_r_alpha1_d1_f[i][j] = CY_r_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCY_r_lef_alpha2_d1_f[i][j] = dCY_r_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_CY_p_alpha1_d1_f[i][j] = CY_p_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCY_p_lef_alpha2_d1_f[i][j] = dCY_p_lef_alpha2_d1_f[i][j];
		}
	}
	
	// ------------- Cl -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_alpha1_beta_dh_n25_d1_f[i][j] = Cl_alpha1_beta_dh_n25_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_alpha1_beta_dh_0_d1_f[i][j] = Cl_alpha1_beta_dh_0_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_alpha1_beta_dh_p25_d1_f[i][j] = Cl_alpha1_beta_dh_p25_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_lef_alpha2_beta_d1_f[i][j] = Cl_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_alpha1_beta_da_20_d1_f[i][j] = Cl_alpha1_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_lef_alpha2_beta_da_20_d1_f[i][j] = Cl_lef_alpha2_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cl_alpha1_beta_dr_30_d1_f[i][j] = Cl_alpha1_beta_dr_30_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_Cl_r_alpha1_d1_f[i][j] = Cl_r_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_Cl_p_alpha1_d1_f[i][j] = Cl_p_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCl_beta_alpha1_d1_f[i][j] = dCl_beta_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCl_r_lef_alpha2_d1_f[i][j] = dCl_r_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCl_p_lef_alpha2_d1_f[i][j] = dCl_p_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCl_p_lef_alpha2_d1_f[i][j] = dCl_p_lef_alpha2_d1_f[i][j];
		}
	}
	
	// ------------- Cn -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_alpha1_beta_dh_n25_d1_f[i][j] = Cn_alpha1_beta_dh_n25_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_alpha1_beta_dh_0_d1_f[i][j] = Cn_alpha1_beta_dh_0_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_alpha1_beta_dh_p25_d1_f[i][j] = Cn_alpha1_beta_dh_p25_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_lef_alpha2_beta_d1_f[i][j] = Cn_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_alpha1_beta_da_20_d1_f[i][j] = Cn_alpha1_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_lef_alpha2_beta_da_20_d1_f[i][j] = Cn_lef_alpha2_beta_da_20_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cn_alpha1_beta_dr_30_d1_f[i][j] = Cn_alpha1_beta_dr_30_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_Cn_r_alpha1_d1_f[i][j] = Cn_r_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCn_beta_alpha1_d1_f[i][j] = dCn_beta_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCn_da_alpha1_d1_f[i][j] = dCn_da_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCn_r_lef_alpha2_d1_f[i][j] = dCn_r_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_Cn_p_alpha1_d1_f[i][j] = Cn_p_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCn_p_lef_alpha2_d1_f[i][j] = dCn_p_lef_alpha2_d1_f[i][j];
		}
	}
	
	// ------------- Cm -------------------
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_alpha1_beta_dh_n25_d1_f[i][j] = Cm_alpha1_beta_dh_n25_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_alpha1_beta_dh_n10_d1_f[i][j] = Cm_alpha1_beta_dh_n10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_alpha1_beta_dh_0_d1_f[i][j] = Cm_alpha1_beta_dh_0_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_alpha1_beta_dh_p10_d1_f[i][j] = Cm_alpha1_beta_dh_p10_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_alpha1_beta_dh_p25_d1_f[i][j] = Cm_alpha1_beta_dh_p25_d1_f[i][j];
		}
	}
	for (i=0; i<15; i++) 
	{
		for (j=0; j<20; j++) 
		{
			_Cm_lef_alpha2_beta_d1_f[i][j] = Cm_lef_alpha2_beta_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCm_sb_alpha1_d1_f[i][j] = dCm_sb_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCm_alpha1_d1_f[i][j] = dCm_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<5; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_eta_dh_d1_f[i][j] = eta_dh_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_Cm_q_alpha1_d1_f[i][j] = Cm_q_alpha1_d1_f[i][j];
		}
	}
	for (i=0; i<14; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCm_q_lef_alpha2_d1_f[i][j] = dCm_q_lef_alpha2_d1_f[i][j];
		}
	}
	for (i=0; i<21; i++) 
	{
		for (j=0; j<8; j++) 
		{
			_dCm_alpha1_ds_d1_f[i][j] = dCm_alpha1_ds_d1_f[i][j];
		}
	}
	for (i=0; i<20; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_dCm_tef_alpha1[i][j] = dCm_tef_alpha1[i][j];
		}
	}
	
	// ------------- Engines -------------------
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_idle_mach_alt_d1_f[i][j] = thrust_idle_mach_alt_d1_f[i][j];
		}
	}
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_mil_mach_alt_d1_f[i][j] = thrust_mil_mach_alt_d1_f[i][j];
		}
	}
	for (i=0; i<6; i++) 
	{
		for (j=0; j<7; j++) 
		{
			_thrust_max_mach_alt_d1_f[i][j] = thrust_max_mach_alt_d1_f[i][j];
		}
	}
	
	
	
}

//---------------------------------------------------------------------
// this need to be checked http://www.ida.upmc.fr/~zaleski/markers_doc/interpolation_8h-source.html
float f16_fdm_f::interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val)
{
	float lambda, value;
	int r = 0;

	while(r < row_nb && table[r][0] < row_val) {r++;}

	if (r == 0) {r = 1;}
	if (r == row_nb) {r--;}

	lambda = (row_val - table[r-1][0])/(table[r][0] - table[r-1][0]);
	value  = table[r-1][1] + lambda*(table[r][1] - table[r-1][1]);

	return value;
}

//---------------------------------------------------------------------
float f16_fdm_f::interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val)
{
	float lambda1, lambda2, value;
	int r = 1;
	int c = 1;

	while(r < row_nb && table[r][0] < row_val) {r++;}
	while(c < col_nb && table[0][c] < col_val) {c++;}

	if (r == 1) {r = 2;}
	if (r == row_nb) {r--;}
	if (c == 1) {c = 2;}
	if (c == col_nb) {c--;}

	lambda1 = (row_val - table[r-1][0])/(table[r][0] - table[r-1][0]);
	lambda2 = (col_val - table[0][c-1])/(table[0][c] - table[0][c-1]);

	value  = (1.0 - lambda1) * (1.0 - lambda2) * table[r-1][c-1];
	value +=       (lambda1) * (1.0 - lambda2) * table[r][c-1]; 
	value += (1.0 - lambda1) *       (lambda2) * table[r-1][c]; 
	value +=       (lambda1) *       (lambda2) * table[r][c];

	return value;
}

//---------------------------------------------------------------------
float f16_fdm_f::getCx(float alpha_deg, float beta_deg, float dh_deg)
{
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;

    if ( dh_deg < 0.0 )
    {
        if ( dh_deg < -10.0 )
        {
            coef = ( dh_deg + 25.0 ) / 15.0;

            l = interp_2d_f(_CX_alpha1_beta_dh_n25_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CX_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg + 10.0 ) / 10.0;
            l = interp_2d_f(_CX_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CX_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    else
    {
        if ( dh_deg < 10.0 )
        {
            coef = dh_deg / 10.0;
            l = interp_2d_f(_CX_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CX_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg - 10.0) / 15.0;
            l = interp_2d_f(_CX_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CX_alpha1_beta_dh_p25_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    // Saturation
	if (coef < 0.0)
	{
		coef = 0.0;
	}
	else if (coef > 1.0)
	{
		coef = 1.0;
	}
    return l + coef * ( h - l );
}

//---------------------------------------------------------------------
float f16_fdm_f::getCx_dh_0(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_CX_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCy(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_CY_alpha1_beta_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCz(float alpha_deg, float beta_deg, float dh_deg)
{
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;

    if ( dh_deg < 0.0 )
    {
        if ( dh_deg < -10.0 )
        {
            coef = ( dh_deg + 25.0 ) / 15.0;
            l = interp_2d_f(_CZ_alpha1_beta_dh_n25_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CZ_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg + 10.0 ) / 10.0;
            l = interp_2d_f(_CZ_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CZ_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    else
    {
        if ( dh_deg < 10.0 )
        {
            coef = (dh_deg) / 10.0;
            l = interp_2d_f(_CZ_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CZ_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg - 10.0) / 15.0;
            l = interp_2d_f(_CZ_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_CZ_alpha1_beta_dh_p25_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    // Saturation
	if (coef < 0.0)
	{
		coef = 0.0;
	}
	else if (coef > 1.0)
	{
		coef = 1.0;
	}
    return l + coef * ( h - l );
}

//---------------------------------------------------------------------
float f16_fdm_f::getCz_dh_0(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_CZ_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
}


//---------------------------------------------------------------------
float f16_fdm_f::getCl(float alpha_deg, float beta_deg, float dh_deg)
{
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;
    
	if ( dh_deg < 0.0 )
    {
        coef = ( dh_deg + 25.0 ) / 25.0;
        l = interp_2d_f(_Cl_alpha1_beta_dh_n25_d1_f,21,20,alpha_deg,beta_deg);
        h = interp_2d_f(_Cl_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
    }
    else
    {
        coef = dh_deg / 25.0;
        l = interp_2d_f(_Cl_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
        h = interp_2d_f(_Cl_alpha1_beta_dh_p25_d1_f,21,20,alpha_deg,beta_deg);
    }
    // Saturation
	if (coef < 0.0)
	{
		coef = 0.0;
	}
	else if (coef > 1.0)
	{
		coef = 1.0;
	}
    return l + coef * ( h - l );
}

//---------------------------------------------------------------------
float f16_fdm_f::getCl_dh_0(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cl_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCm(float alpha_deg, float beta_deg, float dh_deg)
{
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;

    if ( dh_deg < 0.0 )
    {
        if ( dh_deg < -10.0 )
        {
            coef = ( dh_deg + 25.0 ) / 15.0;
            l = interp_2d_f(_Cm_alpha1_beta_dh_n25_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_Cm_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg + 10.0 ) / 10.0;
            l = interp_2d_f(_Cm_alpha1_beta_dh_n10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_Cm_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    else
    {
        if ( dh_deg < 10.0 )
        {
            coef = dh_deg / 10.0;
            l = interp_2d_f(_Cm_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_Cm_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
        }
        else
        {
            coef = ( dh_deg - 10.0) / 15.0;
            l = interp_2d_f(_Cm_alpha1_beta_dh_p10_d1_f,21,20,alpha_deg,beta_deg);
            h = interp_2d_f(_Cm_alpha1_beta_dh_p25_d1_f,21,20,alpha_deg,beta_deg);
        }
    }
    // Saturation
	if (coef < 0.0)
	{
		coef = 0.0;
	}
	else if (coef > 1.0)
	{
		coef = 1.0;
	}
    return l + coef * ( h - l );
}

//---------------------------------------------------------------------
float f16_fdm_f::getCm_dh_0(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cm_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCn(float alpha_deg, float beta_deg, float dh_deg)
{
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;
    
	if ( dh_deg < 0.0 )
    {
        coef = ( dh_deg + 25.0 ) / 25.0;
        l = interp_2d_f(_Cn_alpha1_beta_dh_n25_d1_f,21,20,alpha_deg,beta_deg);
        h = interp_2d_f(_Cn_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
    }
    else
    {
        coef = dh_deg / 25.0;
        l = interp_2d_f(_Cn_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
        h = interp_2d_f(_Cn_alpha1_beta_dh_p25_d1_f,21,20,alpha_deg,beta_deg);
    }
    // Saturation
	if (coef < 0.0)
	{
		coef = 0.0;
	}
	else if (coef > 1.0)
	{
		coef = 1.0;
	}
    return l + coef * ( h - l );
}

//---------------------------------------------------------------------
float f16_fdm_f::getCn_dh_0(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cn_alpha1_beta_dh_0_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCXq(float alpha_deg)
{
    return interp_1d_f(_CX_q_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCYp(float alpha_deg)
{
    return interp_1d_f(_CY_p_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCYr(float alpha_deg)
{
    return interp_1d_f(_CY_r_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCZq(float alpha_deg)
{
    return interp_1d_f(_CZ_q_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCLp(float alpha_deg)
{
    return interp_1d_f(_Cl_p_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCLr(float alpha_deg)
{
    return interp_1d_f(_Cl_r_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCMq(float alpha_deg)
{
    return interp_1d_f(_Cm_q_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCNp(float alpha_deg)
{
    return interp_1d_f(_Cn_p_alpha1_d1_f,20,alpha_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCNr(float alpha_deg)
{
    return interp_1d_f(_Cn_r_alpha1_d1_f,20,alpha_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCx_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_CX_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCy_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_CY_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCz_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_CZ_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCl_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_Cl_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCm_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_Cm_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getCn_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_Cn_lef_alpha2_beta_d1_f,15,20,alpha_lef_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCXq_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCX_q_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCYp_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCY_p_lef_alpha2_d1_f,14,alpha_lef_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getdCYr_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCY_r_lef_alpha2_d1_f,14,alpha_lef_deg);
}
//---------------------------------------------------------------------
float f16_fdm_f::getdCZq_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCZ_q_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCLp_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCl_p_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCLr_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCl_r_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCMq_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCm_q_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCNp_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCn_p_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCNr_lef(float alpha_lef_deg)
{
    return interp_1d_f(_dCn_r_lef_alpha2_d1_f,14,alpha_lef_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCy_r30(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_CY_alpha1_beta_dr_30_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCl_r30(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cl_alpha1_beta_dr_30_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCn_r30(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cn_alpha1_beta_dr_30_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCy_a20(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_CY_alpha1_beta_da_20_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCy_a20_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_CY_lef_alpha2_beta_da_20_d1_f,15,20,alpha_lef_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCn_a20(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cn_alpha1_beta_da_20_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCn_a20_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_Cn_lef_alpha2_beta_da_20_d1_f,15,20,alpha_lef_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCl_a20(float alpha_deg, float beta_deg)
{
    return interp_2d_f(_Cl_alpha1_beta_da_20_d1_f,21,20,alpha_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getCl_a20_lef(float alpha_lef_deg, float beta_deg)
{
    return interp_2d_f(_Cl_lef_alpha2_beta_da_20_d1_f,15,20,alpha_lef_deg,beta_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCn_beta(float alpha_deg)
{
    return interp_1d_f(_dCn_beta_alpha1_d1_f,20,alpha_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCl_beta(float alpha_deg)
{
    return interp_1d_f(_dCl_beta_alpha1_d1_f,20,alpha_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCm(float alpha_deg)
{
    return interp_1d_f(_dCm_alpha1_d1_f,20,alpha_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getEta_el(float dh_deg)
{
    return interp_1d_f(_eta_dh_d1_f,5,dh_deg);
}

//---------------------------------------------------------------------
float f16_fdm_f::getdCm_ds(float alpha_deg, float dh_deg)
{
    return interp_2d_f(_dCm_alpha1_ds_d1_f,21,8,alpha_deg,dh_deg);
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::tgear (float throt)
{
    float tgear_value; 
    if (throt <= 0.77)
    {
		tgear_value = 64.94 * throt;
    }
    else
    {
		tgear_value = 217.38 * throt - 117.38;
    }
    return tgear_value;    
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::rtau (float dp )
{
    float tau_r;
    if ( dp <= 25.0 )
    {
        tau_r = 1.0;  
    }
    else if ( dp >= 50.0 )
    {
        tau_r = 0.1;
    }
    else
    {
        tau_r = 1.9 - 0.036 * dp;
    }
    return tau_r;
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::power_dot( float pw, float cpw )
{

    float tpw, ta;

    if ( cpw >= 50.0 )
    {
        if ( pw >= 50.0)
        {
                tpw = cpw;
                ta = 5.0;
        }
        else
        {
            tpw = 60.0;
            ta = rtau ( tpw - pw );
        }
    }
    else
    {
        if ( pw >= 50.0)
        {
            tpw = 40.0;
            ta = 5.0;
        }
        else
        {
            tpw = cpw;
            ta = rtau ( tpw - pw );
        }
    }
    return ta * ( tpw - pw );
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::thrust(float power, float mach, float alt_m)
{
	float thrust_idle = 0.0, thrust_mil = 0.0, thrust_max = 0.0;
	thrust_idle = interp_2d_f(_thrust_idle_mach_alt_d1_f,6,7,mach,alt_m);
	thrust_mil = interp_2d_f(_thrust_mil_mach_alt_d1_f,6,7,mach,alt_m);
	thrust_max = interp_2d_f(_thrust_max_mach_alt_d1_f,6,7,mach,alt_m);

	if (power < 50)
	{
		_thrust = thrust_idle +(thrust_mil - thrust_idle)*power/50;
	}
	else if (power>=50)
	{
		_thrust = thrust_mil +(thrust_max - thrust_mil)*(power-50)/50;
	}
	
	//_thrust = _thrust*4.4482216;
	//_thrust = _thrust*4.4482216;
	if (_thrust > 19000*4.4482216) _thrust = 19000*4.4482216;
	if (_thrust < 1000*4.4482216) _thrust = 1000*4.4482216;
	return _thrust;
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::saturation_pos(float input, float min, float max)
{
	float value = input;
	if (value > max) value = max;
	else if (value < min) value = min;
	return value;
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::saturation_rate(float input_old, float input_new, float max_rate)
{
	if ( fabs( input_new - input_old ) < max_rate )
    {
        return input_new;
    }
    else
    {
        return input_old + sign( input_new - input_old ) * max_rate;
    }
}

//---------------------------------------------------------------------
// 
float f16_fdm_f::sign(float value)
{
	if      ( value < 0.0 ) return -1.0;
	else if ( value > 0.0 ) return  1.0;

	return 0.0;
}

//---------------------------------------------------------------------
// 
void f16_fdm_f::u2fcs()
{
	_fdm.fcs.der = _fdm.u.de; 
	_fdm.fcs.del = _fdm.u.de; 
	_fdm.fcs.dar = _fdm.u.da; 
	_fdm.fcs.dal = -_fdm.u.da; // sign = (-1)
	_fdm.fcs.dlefr = _fdm.u.dlef; 
	_fdm.fcs.dlefl = _fdm.u.dlef;
	_fdm.fcs.dtefr = _fdm.u.dtef; 
	_fdm.fcs.dtefl = _fdm.u.dtef;
	_fdm.fcs.dr = _fdm.u.dr;
	_fdm.fcs.dth  = _fdm.u.dth;
}

//---------------------------------------------------------------------
// 
void f16_fdm_f::u2cs()
{
	_pos_ele_r = _fdm.u.de;
	_pos_ele_l = _fdm.u.de;
	_pos_ail_r = _fdm.u.da;
	_pos_ail_l = -_fdm.u.da;
	_pos_lef_r = _fdm.u.dlef;
	_pos_lef_l = _fdm.u.dlef;
	_pos_tef_r = _fdm.u.dtef;
	_pos_tef_l = _fdm.u.dtef;
	_pos_rud = _fdm.u.dr;
}

//---------------------------------------------------------------------
// 
f16_fcs_cmd_f_t f16_fdm_f::get_fcs_cmd()
{
	return _fdm.fcs;
}

//---------------------------------------------------------------------
// 
void f16_fdm_f::set_fcs_cmd(f16_fcs_cmd_f_t fcs_cmd)
{
	_fdm.fcs = fcs_cmd;
}


//---------------------------------------------------------------------
// 
void f16_fdm_f::compute_u()
{
	// Engine
	_fdm.u.dth = _fdm.fcs.dth;
	// Control surfaces
	_fdm.u.dlef = 0.5 * (_pos_lef_r + _pos_lef_l);
	_fdm.u.dlef_norm = 1 - (_fdm.u.dlef / _max_lef_r); // may change in case max value is diff for each
	_fdm.u.dtef = 0.5 * (_pos_tef_r + _pos_tef_l);
	_fdm.u.dtef_norm = _fdm.u.dtef / _max_tef_r; // may change in case max value is diff for each
	_fdm.u.da = 0.5 * (_pos_ail_r - _pos_ail_l);
	_fdm.u.da_norm = _fdm.u.da / _max_ail_r;// may change in case max value is diff for each
	_fdm.u.de = 0.5 * (_pos_ele_r + _pos_ele_l);
	_fdm.u.de_norm = _fdm.u.de / _max_ele_r; // may change in case max value is diff for each
	_fdm.u.dr = _pos_rud;
	_fdm.u.dr_norm = _fdm.u.dr / _max_rud; // may change in case max value is diff for each	
}

//---------------------------------------------------------------------
// 
void f16_fdm_f::compute_cs(float dt)
{
	// First order filders
	_pos_ele_r = _pos_ele_r + ( 1.0 - exp( -dt / _tc_ele_r ) ) * ( _fdm.fcs.der - _pos_ele_r );
	_pos_ele_l = _pos_ele_l + ( 1.0 - exp( -dt / _tc_ele_l ) ) * ( _fdm.fcs.del - _pos_ele_l );
	_pos_ail_r = _pos_ail_r + ( 1.0 - exp( -dt / _tc_ail_r ) ) * ( _fdm.fcs.dar - _pos_ail_r );
	_pos_ail_l = _pos_ail_l + ( 1.0 - exp( -dt / _tc_ail_l ) ) * ( _fdm.fcs.dal - _pos_ail_l );
	_pos_lef_r = _pos_lef_r + ( 1.0 - exp( -dt / _tc_lef_r ) ) * ( _fdm.fcs.dlefr - _pos_lef_r );
	_pos_lef_l = _pos_lef_l + ( 1.0 - exp( -dt / _tc_lef_l ) ) * ( _fdm.fcs.dlefl - _pos_lef_l );
	_pos_tef_r = _pos_tef_r + ( 1.0 - exp( -dt / _tc_tef_r ) ) * ( _fdm.fcs.dtefr - _pos_tef_r );
	_pos_tef_l = _pos_tef_l + ( 1.0 - exp( -dt / _tc_tef_l ) ) * ( _fdm.fcs.dtefl - _pos_tef_l );
	_pos_rud = _pos_rud + ( 1.0 - exp( -dt / _tc_rud ) ) * ( _fdm.fcs.dr - _pos_rud );
	
	// save old position for rate calculation
	_pos_ele_r_old = _pos_ele_r;
	_pos_ele_l_old = _pos_ele_r;
	_pos_ail_r_old = _pos_ail_r;
	_pos_ail_l_old = _pos_ail_l;
	_pos_lef_r_old = _pos_lef_r;
	_pos_lef_l_old = _pos_lef_r;
	_pos_tef_r_old = _pos_tef_r;
	_pos_tef_l_old = _pos_tef_r;
	_pos_rud_old = _pos_rud;
	
	// Saturation position
	_pos_ele_r = saturation_pos(_pos_ele_r, _min_ele_r, _max_ele_r);
	_pos_ele_l = saturation_pos(_pos_ele_l, _min_ele_l, _max_ele_l);
	_pos_ail_r = saturation_pos(_pos_ail_r, _min_ail_r, _max_ail_r);
	_pos_ail_l = saturation_pos(_pos_ail_l, _min_ail_l, _max_ail_l);
	_pos_lef_r = saturation_pos(_pos_lef_r, _min_lef_r, _max_lef_r);
	_pos_lef_l = saturation_pos(_pos_lef_l, _min_lef_l, _max_lef_l);
	_pos_tef_r = saturation_pos(_pos_tef_r, _min_tef_r, _max_tef_r);
	_pos_tef_l = saturation_pos(_pos_tef_l, _min_tef_l, _max_tef_l);
	_pos_rud = saturation_pos(_pos_rud, _min_rud, _max_rud);
	
	// Saturation rate
	_pos_ele_r = saturation_rate(_pos_ele_r_old, _pos_ele_r, _max_rate_ele_r);
	_pos_ele_l = saturation_rate(_pos_ele_l_old, _pos_ele_l, _max_rate_ele_l);
	_pos_ail_r = saturation_rate(_pos_ail_r_old, _pos_ail_r, _max_rate_ail_l);
	_pos_ail_l = saturation_rate(_pos_ail_l_old, _pos_ail_l, _max_rate_ail_l);
	_pos_lef_r = saturation_rate(_pos_lef_r_old, _pos_lef_r, _max_rate_lef_r);
	_pos_lef_l = saturation_rate(_pos_lef_l_old, _pos_lef_l, _max_rate_lef_l);
	_pos_tef_r = saturation_rate(_pos_tef_r_old, _pos_tef_r, _max_rate_tef_r);
	_pos_tef_l = saturation_rate(_pos_tef_l_old, _pos_tef_l, _max_rate_tef_l);
	_pos_rud = saturation_rate(_pos_rud_old, _pos_rud, _max_rate_rud);

	_fdm.cs.der   = _pos_ele_r; 
	_fdm.cs.del   = _pos_ele_l; 
	_fdm.cs.dar   = _pos_ail_r; 
	_fdm.cs.dal   = _pos_ail_l;
	_fdm.cs.dlefr = _pos_lef_r; 
	_fdm.cs.dlefl = _pos_lef_l;
	_fdm.cs.dtefr = _pos_tef_r; 
	_fdm.cs.dtefl = _pos_tef_l;
	_fdm.cs.dr    = _pos_rud;
		
}


//---------------------------------------------------------------------
void f16_fdm_f::compute_xdot(bool is_thrust_external, float thrust_in)
{
	float sa, ca, sb, cb, tb, st, ct, tt, sphi, cphi, spsi, cpsi;
	float r2d, d2r;
	float g;
	
	float CX_tot, CY_tot, CZ_tot, Cl_tot, Cm_tot, Cn_tot;
	float Cx, Cz, Cm, Cy, Cn, Cl;
    float Cxq, Cyr, Cyp, Czq, Clr, Clp, Cmq, Cnr, Cnp;
    float delta_Cx_lef, delta_Cz_lef,delta_Cm_lef;
    float delta_Cy_lef,delta_Cn_lef,delta_Cl_lef;
    float delta_Cxq_lef, delta_Cyr_lef, delta_Cyp_lef, delta_Czq_lef;
    float delta_Clr_lef, delta_Clp_lef, delta_Cmq_lef, delta_Cnr_lef;
    float delta_Cnp_lef;
    float delta_Cy_r30, delta_Cn_r30, delta_Cl_r30;
    float delta_Cy_a20, delta_Cy_a20_lef, delta_Cn_a20, delta_Cn_a20_lef;
    float delta_Cl_a20, delta_Cl_a20_lef;
    float delta_Cnbeta, delta_Clbeta, delta_Cm, eta_el, delta_Cm_ds;
    float dq;
    
    float alpha_deg, beta_deg, alpha_lef_deg;
	
	// deg to rad
	r2d = 57.29577951;
	d2r = 0.017453293;
	
	// atmospheric - to be moved
	static float g0     =  9.80665;
	static float T0     =  288.15;
	static float rho0   =  1.225;
	static float p0     =  101325;
	static float lambda = -0.0065;
	static float GASCON =  287.05;
	static float gammah =  1.4;
	static float RADIUS =  6371020;
	static float Hs     =  11000;
	float he, Hgeo, mu, M, pa;
	float ptot, prel, qc, qrel, qdyn; 
	float rho, Reynl, T, Ttot; 
	float VCAS, VEAS, VIAS, Vsound, VTAS; 
	
	VTAS = _fdm.x.vt;
	he   = _fdm.x.alt;
	
	/* atmospheric data */
	Hgeo   = RADIUS*he/(RADIUS+he);

	if (Hgeo<=Hs) 
	{
		T   = T0+lambda*Hgeo;
		rho = rho0*pow((T/T0),-(g0/(GASCON*lambda)+1));
	}
	else 
	{
		T   = T0+lambda*Hs;
		rho = rho0*pow((T/T0),-(g0/(GASCON*lambda)+1));
		rho = rho*exp(-g0/(GASCON*T)*(Hgeo-Hs));
	}

	pa     = rho*GASCON*T;
	g      = g0*(RADIUS*RADIUS)/((RADIUS+he)*(RADIUS+he));
	Vsound = sqrt(gammah*GASCON*T);

	/* airdata 1 */
	M      = VTAS/Vsound;
	qdyn   = 0.5*rho*VTAS*VTAS;

	/* airdata 2 */
	mu    = 1.458e-6*pow(T,1.5)/(T+110.4);
	Reynl = rho*VTAS/mu;

	qrel = pow((1+0.2*M*M),3.5)-1;
	qc   = pa*qrel;
	ptot = pa+qc;
	Ttot = T*(1+0.2*M*M);

	VEAS = sqrt(2*qdyn/rho0);
	VCAS = sqrt(2*3.5*p0/rho0*(pow((1+qc/p0),(1/3.5))-1));
	VIAS = VCAS;
	
	_fdm.x.qbar = qdyn;
	_fdm.x.ps = pa;
	_fdm.x.mach = M;
	
	float cpow = tgear(_fdm.u.dth);
    _fdm.xdot.power_dot = power_dot(_fdm.x.power, cpow);
    if (is_thrust_external)
    {
		_fdm.x.thrust = thrust_in;
	}
	else
	{
		_fdm.x.thrust = thrust(_fdm.x.power, _fdm.x.mach, _fdm.x.alt);
	}
	// Compute Control surface and engine from FCS
	
	
	// limit alpha et beta
	if (_fdm.x.alpha > 90.0*d2r) _fdm.x.alpha = 90.0*d2r;
	else if (_fdm.x.alpha < -20.0*d2r) _fdm.x.alpha = -20.0*d2r;
	if (_fdm.x.beta > 30.0*d2r) _fdm.x.beta = 30.0*d2r;
	else if (_fdm.x.beta < -30.0*d2r) _fdm.x.beta = -30.0*d2r;
	alpha_deg = _fdm.x.alpha * r2d;
	alpha_lef_deg = alpha_deg;
	beta_deg = _fdm.x.beta * r2d;
	if (alpha_lef_deg > 45.0) alpha_lef_deg = 45.0;

	compute_u();

	sa    = sin(_fdm.x.alpha); // sin(alpha) 
	ca    = cos(_fdm.x.alpha); // cos(alpha) 
	sb    = sin(_fdm.x.beta); // sin(beta)  
	cb    = cos(_fdm.x.beta); // cos(beta)  
	tb    = tan(_fdm.x.beta); 

	st    = sin(_fdm.x.theta);
	ct    = cos(_fdm.x.theta);
	tt    = tan(_fdm.x.theta);
	sphi  = sin(_fdm.x.phi);
	cphi  = cos(_fdm.x.phi);
	spsi  = sin(_fdm.x.psi);
	cpsi  = cos(_fdm.x.psi);
	
	// Assign old xdot
	_xdot_old2 = _xdot_old1;
	_xdot_old1 = _fdm.xdot;
	
	// Navigation equations
	_fdm.xdot.npos_dot = _fdm.x.u*(ct*cpsi) + 
						_fdm.x.v*(sphi*cpsi*st - cphi*spsi) + 
						_fdm.x.w*(cphi*st*cpsi + sphi*spsi);
	_fdm.xdot.epos_dot = _fdm.x.u*(ct*spsi) + 
						_fdm.x.v*(sphi*spsi*st + cphi*cpsi) + 
						_fdm.x.w*(cphi*st*spsi - sphi*cpsi);
	_fdm.xdot.alt_dot = _fdm.x.u*st - _fdm.x.v*(sphi*ct) - _fdm.x.w*(cphi*ct);
	
	// lat long
	float a = 6378137;           // Semi-major axis (m)
	float b = 6356752;           // Semi-minor axis (m)
	float  radius = sqrt(a*cos(_fdm.x.lat)*a*cos(_fdm.x.lat) + b*sin(_fdm.x.lat)*b*sin(_fdm.x.lat));
	float  flight_radius = radius + _fdm.x.alt;
	_fdm.xdot.lat_dot = _fdm.xdot.npos_dot/flight_radius;
	_fdm.xdot.lon_dot = _fdm.xdot.epos_dot/(cos(_fdm.x.lat)*flight_radius);
  
	// Kinematic equations - Euler angles
	_fdm.xdot.phi_dot = _fdm.x.p + tt*(_fdm.x.q*sphi + _fdm.x.r*cphi);
	_fdm.xdot.theta_dot = _fdm.x.q*cphi - _fdm.x.r*sphi;
	_fdm.xdot.psi_dot = (_fdm.x.q*sphi + _fdm.x.r*cphi)/ct;
	// Kinematic equations - Quaternions
	_fdm.xdot.q0_dot      = 0.5 * (-_fdm.x.p * _fdm.x.q1 - _fdm.x.q * _fdm.x.q2 - _fdm.x.r * _fdm.x.q3);
    _fdm.xdot.q1_dot      = 0.5 * ( _fdm.x.p * _fdm.x.q0 + _fdm.x.r * _fdm.x.q2 - _fdm.x.q * _fdm.x.q3);
    _fdm.xdot.q2_dot      = 0.5 * ( _fdm.x.q * _fdm.x.q0 - _fdm.x.r * _fdm.x.q1 + _fdm.x.p * _fdm.x.q3);
    _fdm.xdot.q3_dot      = 0.5 * ( _fdm.x.r * _fdm.x.q0 + _fdm.x.q * _fdm.x.q1 - _fdm.x.p * _fdm.x.q2);
    
    /* correction term from User Manual by K. Refson */
    dq = _fdm.x.q0 * _fdm.xdot.q0_dot + _fdm.x.q1 * _fdm.xdot.q1_dot + _fdm.x.q2 * _fdm.xdot.q2_dot + _fdm.x.q3 * _fdm.xdot.q3_dot;
    
    _fdm.xdot.q0_dot -= dq * _fdm.x.q0;
    _fdm.xdot.q1_dot -= dq * _fdm.x.q1;
    _fdm.xdot.q2_dot -= dq * _fdm.x.q2;
    _fdm.xdot.q3_dot -= dq * _fdm.x.q3;
	
	// Lookup table
	Cx = getCx(alpha_deg, beta_deg, _fdm.u.de);
	Cy = getCy(alpha_deg, beta_deg);
	Cz = getCz(alpha_deg, beta_deg, _fdm.u.de);
	Cl = getCl(alpha_deg, beta_deg, _fdm.u.de);
	Cm = getCm(alpha_deg, beta_deg, _fdm.u.de);
	Cn = getCn(alpha_deg, beta_deg, _fdm.u.de);
	
	Cxq = getCXq(alpha_deg);
	Cyp = getCYp(alpha_deg);
	Cyr = getCYr(alpha_deg);
	Czq = getCZq(alpha_deg);
	Clp = getCLp(alpha_deg);
	Clr = getCLr(alpha_deg);
	Cmq = getCMq(alpha_deg);
	Cnp = getCNp(alpha_deg);
	Cnr = getCNr(alpha_deg);
	
	delta_Cx_lef = getCx_lef(alpha_lef_deg, beta_deg) - getCx_dh_0(alpha_deg, beta_deg); 
	delta_Cy_lef = getCy_lef(alpha_lef_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cz_lef = getCz_lef(alpha_lef_deg, beta_deg) - getCz_dh_0(alpha_deg, beta_deg);
	delta_Cl_lef = getCl_lef(alpha_lef_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cm_lef = getCm_lef(alpha_lef_deg, beta_deg) - getCm_dh_0(alpha_deg, beta_deg);
	delta_Cn_lef = getCn_lef(alpha_lef_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	
	delta_Cxq_lef = getdCXq_lef(alpha_lef_deg);
	delta_Cyp_lef = getdCYp_lef(alpha_lef_deg);
	delta_Cyr_lef = getdCYr_lef(alpha_lef_deg);
	delta_Czq_lef = getdCZq_lef(alpha_lef_deg);
	delta_Clp_lef = getdCLp_lef(alpha_lef_deg);
	delta_Clr_lef = getdCLr_lef(alpha_lef_deg);
	delta_Cmq_lef = getdCMq_lef(alpha_lef_deg);
	delta_Cnp_lef = getdCNp_lef(alpha_lef_deg);
	delta_Cnr_lef = getdCNr_lef(alpha_lef_deg);
	
	delta_Cy_r30 = getCy_r30(alpha_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cl_r30 = getCl_r30(alpha_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cn_r30 = getCn_r30(alpha_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	
	delta_Cy_a20     = getCy_a20(alpha_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cy_a20_lef = getCy_a20_lef(alpha_lef_deg, beta_deg) - getCy_lef(alpha_lef_deg, beta_deg) - delta_Cy_a20;
	delta_Cn_a20     = getCn_a20(alpha_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	delta_Cn_a20_lef = getCn_a20_lef(alpha_lef_deg, beta_deg) - getCn_lef(alpha_lef_deg, beta_deg) - delta_Cn_a20;
	delta_Cl_a20     = getCl_a20(alpha_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cl_a20_lef = getCl_a20_lef(alpha_lef_deg, beta_deg) - getCl_lef(alpha_lef_deg, beta_deg) - delta_Cl_a20;
	
	delta_Cnbeta = getdCn_beta(alpha_deg);
	delta_Clbeta = getdCl_beta(alpha_deg);
	delta_Cm     = getdCm(alpha_deg);
	eta_el       = getEta_el(_fdm.u.de);
	delta_Cm_ds  = getdCm_ds(alpha_deg,_fdm.u.de);
	delta_Cm_ds =0.0;
	
	/* Total force coefficients */
    /* Cx_tot */
    CX_tot = Cx + delta_Cx_lef * _fdm.u.dlef_norm
            + (_fdm.mi.cref/(2*_fdm.x.vt))*(Cxq + delta_Cxq_lef * _fdm.u.dlef_norm) * _fdm.x.q;
    /* Cy_tot */
    CY_tot = Cy + delta_Cy_lef * _fdm.u.dlef_norm
            + (delta_Cy_a20 + delta_Cy_a20_lef * _fdm.u.dlef_norm) * _fdm.u.da_norm
            + delta_Cy_r30 * _fdm.u.dr_norm
            + (_fdm.mi.bref / (2*_fdm.x.vt))*(Cyr + delta_Cyr_lef * _fdm.u.dlef_norm) * _fdm.x.r
            + (_fdm.mi.bref/(2*_fdm.x.vt))*(Cyp + delta_Cyp_lef * _fdm.u.dlef_norm) * _fdm.x.p;
    /* Cz_tot */
    CZ_tot = Cz + delta_Cz_lef * _fdm.u.dlef_norm
            + (_fdm.mi.cref/(2*_fdm.x.vt))*(Czq + delta_Czq_lef * _fdm.u.dlef_norm) * _fdm.x.q;
            
	/* Store data in X state */
	_fdm.x.cx = CX_tot;
	_fdm.x.cy = CY_tot;
	_fdm.x.cz = CZ_tot;
    
    /* Total moment coefficients */
    /* Cl_tot */
    Cl_tot = Cl + delta_Cl_lef * _fdm.u.dlef_norm
            + (delta_Cl_a20 + delta_Cl_a20_lef * _fdm.u.dlef_norm) * _fdm.u.da_norm
            + delta_Cl_r30 * _fdm.u.dr_norm
            + (_fdm.mi.bref / ((2*_fdm.x.vt))*(Clr + delta_Clr_lef * _fdm.u.dlef_norm)) * _fdm.x.r
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Clp + delta_Clp_lef * _fdm.u.dlef_norm)) * _fdm.x.p
            + delta_Clbeta * _fdm.x.beta * r2d;
    /* Cm_tot */
    Cm_tot = Cm * eta_el + CZ_tot * (_fdm.mi.xcgr - _fdm.mi.xcg) + delta_Cm_lef * _fdm.u.dlef_norm
            + (_fdm.mi.cref / (2*_fdm.x.vt))*(Cmq + delta_Cmq_lef * _fdm.u.dlef_norm) * _fdm.x.q
            + delta_Cm + delta_Cm_ds;
    /* Cn_tot */
    Cn_tot = Cn + delta_Cn_lef * _fdm.u.dlef_norm
            - CY_tot * (_fdm.mi.xcgr - _fdm.mi.xcg)*(_fdm.mi.cref/_fdm.mi.bref)
            + (delta_Cn_a20 + delta_Cn_a20_lef * _fdm.u.dlef_norm) * _fdm.u.da_norm
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Cnr + delta_Cnr_lef * _fdm.u.dlef_norm))* _fdm.x.r
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Cnp + delta_Cnp_lef * _fdm.u.dlef_norm)) * _fdm.x.p
            + delta_Cn_r30 * _fdm.u.dr_norm + delta_Cnbeta * _fdm.x.beta * r2d;
    
    /* Store data in X state */    
	_fdm.x.cl = Cl_tot;
	_fdm.x.cm = Cm_tot;
	_fdm.x.cn = Cn_tot;
            
            
    // Udot,Vdot, Wdot,(as on NASA report TP-1538 p36) 
	_fdm.xdot.u_dot = _fdm.x.r*_fdm.x.v - _fdm.x.q * _fdm.x.w - g*st + _fdm.x.qbar*_fdm.mi.sref*CX_tot/_fdm.mi.mass + _fdm.x.thrust/_fdm.mi.mass;
	_fdm.xdot.v_dot = _fdm.x.p*_fdm.x.w - _fdm.x.r * _fdm.x.u + g*ct*sphi + _fdm.x.qbar*_fdm.mi.sref*CY_tot/_fdm.mi.mass;
	_fdm.xdot.w_dot = _fdm.x.q*_fdm.x.u - _fdm.x.p * _fdm.x.v + g*ct*cphi + _fdm.x.qbar*_fdm.mi.sref*CZ_tot/_fdm.mi.mass;
	
	// vt_dot equation (from S&L, p82)
	_fdm.xdot.vt_dot = (_fdm.x.u*_fdm.xdot.u_dot + _fdm.x.v*_fdm.xdot.v_dot + _fdm.x.w*_fdm.xdot.w_dot)/_fdm.x.vt;
	
	//alpha_dot et beta_dot
	_fdm.xdot.alpha_dot = (_fdm.x.u*_fdm.xdot.w_dot - _fdm.x.w*_fdm.xdot.u_dot)/(_fdm.x.u*_fdm.x.u + _fdm.x.w*_fdm.x.w);
	_fdm.xdot.beta_dot =  (_fdm.x.vt*_fdm.xdot.v_dot - _fdm.x.v*_fdm.xdot.vt_dot)/(_fdm.x.vt*_fdm.x.vt*cb);
	
	// get moments from coefficients 
	float L_tot, M_tot, N_tot, denom;
	L_tot = Cl_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.bref;      
	M_tot = Cm_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.cref;
	N_tot = Cn_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.bref;
	denom = _fdm.mi.ixx*_fdm.mi.izz - _fdm.mi.ixz*_fdm.mi.ixz;
	
	_fdm.xdot.p_dot = (_fdm.mi.izz*L_tot + _fdm.mi.ixz*N_tot - (_fdm.mi.izz*(_fdm.mi.izz-_fdm.mi.iyy)+_fdm.mi.ixz*_fdm.mi.ixz)*_fdm.x.q*_fdm.x.r + _fdm.mi.ixz*(_fdm.mi.ixx-_fdm.mi.iyy+_fdm.mi.izz)*_fdm.x.p*_fdm.x.q + _fdm.mi.ixz*_fdm.x.q*_fdm.mi.heng)/denom;
	_fdm.xdot.q_dot = (M_tot + (_fdm.mi.izz-_fdm.mi.ixx)*_fdm.x.p*_fdm.x.r - _fdm.mi.ixz*(_fdm.x.p*_fdm.x.p - _fdm.x.r*_fdm.x.r) - _fdm.x.r*_fdm.mi.heng)/_fdm.mi.iyy;
	_fdm.xdot.r_dot = (_fdm.mi.ixx*N_tot + _fdm.mi.ixz*L_tot + (_fdm.mi.ixx*(_fdm.mi.ixx-_fdm.mi.iyy)+_fdm.mi.ixz*_fdm.mi.ixz)*_fdm.x.p*_fdm.x.q - _fdm.mi.ixz*(_fdm.mi.ixx-_fdm.mi.iyy+_fdm.mi.izz)*_fdm.x.q*_fdm.x.r + _fdm.mi.ixx*_fdm.x.q*_fdm.mi.heng)/denom;
	

	
	// copy xdot
	_fdm_euler.xdot = _fdm.xdot;
	_fdm_trap.xdot = _fdm.xdot;
	_fdm_adba2.xdot = _fdm.xdot;
	_fdm_adba3.xdot = _fdm.xdot;
	
	/* Store data in X state */
	_fdm_euler.x.cx = _fdm.x.cx;
	_fdm_euler.x.cy = _fdm.x.cy;
	_fdm_euler.x.cz = _fdm.x.cz;
	_fdm_euler.x.cl = _fdm.x.cl;
	_fdm_euler.x.cm = _fdm.x.cm;
	_fdm_euler.x.cn = _fdm.x.cn;
	_fdm_trap.x.cx = _fdm.x.cx;
	_fdm_trap.x.cy = _fdm.x.cy;
	_fdm_trap.x.cz = _fdm.x.cz;
	_fdm_trap.x.cl = _fdm.x.cl;
	_fdm_trap.x.cm = _fdm.x.cm;
	_fdm_trap.x.cn = _fdm.x.cn;
	_fdm_adba2.x.cx = _fdm.x.cx;
	_fdm_adba2.x.cy = _fdm.x.cy;
	_fdm_adba2.x.cz = _fdm.x.cz;
	_fdm_adba2.x.cl = _fdm.x.cl;
	_fdm_adba2.x.cm = _fdm.x.cm;
	_fdm_adba2.x.cn = _fdm.x.cn;
	_fdm_adba3.x.cx = _fdm.x.cx;
	_fdm_adba3.x.cy = _fdm.x.cy;
	_fdm_adba3.x.cz = _fdm.x.cz;
	_fdm_adba3.x.cl = _fdm.x.cl;
	_fdm_adba3.x.cm = _fdm.x.cm;
	_fdm_adba3.x.cn = _fdm.x.cn;
	
	// accelerations
	_fdm.x.nx =  1.0/g*(_fdm.xdot.u_dot + _fdm.x.q*_fdm.x.w - _fdm.x.r*_fdm.x.v) + st;
	_fdm.x.ny =  1.0/g*(_fdm.xdot.v_dot + _fdm.x.r*_fdm.x.u - _fdm.x.p*_fdm.x.w) - ct*sphi;
	_fdm.x.nz = -1.0/g*(_fdm.xdot.w_dot + _fdm.x.p*_fdm.x.v - _fdm.x.q*_fdm.x.u) + ct*cphi;
	_fdm_euler.x.nx = _fdm.x.nx;
	_fdm_euler.x.ny = _fdm.x.ny;
	_fdm_euler.x.nz = _fdm.x.nz;
	_fdm_trap.x.nx = _fdm.x.nx;
	_fdm_trap.x.ny = _fdm.x.ny;
	_fdm_trap.x.nz = _fdm.x.nz;
	_fdm_adba2.x.nx = _fdm.x.nx;
	_fdm_adba2.x.ny = _fdm.x.ny;
	_fdm_adba2.x.nz = _fdm.x.nz;
	_fdm_adba3.x.nx = _fdm.x.nx;
	_fdm_adba3.x.ny = _fdm.x.ny;
	_fdm_adba3.x.nz = _fdm.x.nz;
	
}

//---------------------------------------------------------------------
void f16_fdm_f::compute_x(float dt)
{
	_fdm.x.npos = _fdm.x.npos +_fdm.xdot.npos_dot *dt;
	_fdm.x.epos = _fdm.x.epos +_fdm.xdot.epos_dot *dt;
	_fdm.x.lat = _fdm.x.lat +_fdm.xdot.lat_dot *dt;
	_fdm.x.lon = _fdm.x.lon +_fdm.xdot.lon_dot *dt;
	_fdm.x.alt = _fdm.x.alt +_fdm.xdot.alt_dot *dt;
	_fdm.x.phi = _fdm.x.phi +_fdm.xdot.phi_dot *dt;
	_fdm.x.theta = _fdm.x.theta +_fdm.xdot.theta_dot *dt;
	_fdm.x.psi = _fdm.x.psi +_fdm.xdot.psi_dot *dt;
	_fdm.x.q0 = _fdm.x.q0 +_fdm.xdot.q0_dot*dt;
	_fdm.x.q1 = _fdm.x.q1 +_fdm.xdot.q1_dot*dt;
	_fdm.x.q2 = _fdm.x.q2 +_fdm.xdot.q2_dot*dt;
	_fdm.x.q3 = _fdm.x.q3 +_fdm.xdot.q3_dot*dt;
	_fdm.x.p = _fdm.x.p+_fdm.xdot.p_dot*dt;
	_fdm.x.q = _fdm.x.q+_fdm.xdot.q_dot*dt;
	_fdm.x.r = _fdm.x.r+_fdm.xdot.r_dot*dt;
	_fdm.x.u = _fdm.x.u+_fdm.xdot.u_dot*dt;
	_fdm.x.v = _fdm.x.v+_fdm.xdot.v_dot*dt;
	_fdm.x.w = _fdm.x.w+_fdm.xdot.w_dot*dt;
	_fdm.x.vt = _fdm.x.vt + _fdm.xdot.vt_dot *dt;
	_fdm.x.alpha = _fdm.x.alpha + _fdm.xdot.alpha_dot*dt;
	_fdm.x.beta = _fdm.x.beta+_fdm.xdot.beta_dot*dt;
	_fdm.x.power = _fdm.x.power + _fdm.xdot.power_dot*dt;
	_fdm.x.vs = _fdm.xdot.alt_dot;
	_fdm.x.time_sec+=dt;
	
	// Integrate Euler
	_fdm_euler.x.npos = _fdm_euler.x.npos +_fdm_euler.xdot.npos_dot *dt;
	_fdm_euler.x.epos = _fdm_euler.x.epos +_fdm_euler.xdot.epos_dot *dt;
	_fdm_euler.x.lat = _fdm_euler.x.lat +_fdm_euler.xdot.lat_dot *dt;
	_fdm_euler.x.lon = _fdm_euler.x.lon +_fdm_euler.xdot.lon_dot *dt;
	_fdm_euler.x.alt = _fdm_euler.x.alt +_fdm_euler.xdot.alt_dot *dt;
	_fdm_euler.x.phi = _fdm_euler.x.phi +_fdm_euler.xdot.phi_dot *dt;
	_fdm_euler.x.theta = _fdm_euler.x.theta +_fdm_euler.xdot.theta_dot *dt;
	_fdm_euler.x.psi = _fdm_euler.x.psi +_fdm_euler.xdot.psi_dot *dt;
	_fdm_euler.x.q0 = _fdm_euler.x.q0 +_fdm_euler.xdot.q0_dot*dt;
	_fdm_euler.x.q1 = _fdm_euler.x.q1 +_fdm_euler.xdot.q1_dot*dt;
	_fdm_euler.x.q2 = _fdm_euler.x.q2 +_fdm_euler.xdot.q2_dot*dt;
	_fdm_euler.x.q3 = _fdm_euler.x.q3 +_fdm_euler.xdot.q3_dot*dt;
	_fdm_euler.x.p = _fdm_euler.x.p+_fdm_euler.xdot.p_dot*dt;
	_fdm_euler.x.q = _fdm_euler.x.q+_fdm_euler.xdot.q_dot*dt;
	_fdm_euler.x.r = _fdm_euler.x.r+_fdm_euler.xdot.r_dot*dt;
	_fdm_euler.x.u = _fdm_euler.x.u+_fdm_euler.xdot.u_dot*dt;
	_fdm_euler.x.v = _fdm_euler.x.v+_fdm_euler.xdot.v_dot*dt;
	_fdm_euler.x.w = _fdm_euler.x.w+_fdm_euler.xdot.w_dot*dt;
	_fdm_euler.x.vt = _fdm_euler.x.vt + _fdm_euler.xdot.vt_dot *dt;
	_fdm_euler.x.alpha = _fdm_euler.x.alpha + _fdm_euler.xdot.alpha_dot*dt;
	_fdm_euler.x.beta = _fdm_euler.x.beta+_fdm_euler.xdot.beta_dot*dt;
	_fdm_euler.x.power = _fdm_euler.x.power + _fdm_euler.xdot.power_dot*dt;
	_fdm_euler.x.vs = _fdm_euler.xdot.alt_dot;
	_fdm_euler.x.time_sec+=dt;
	
	// Integrate Trapèze
	_fdm_trap.x.npos = _fdm_trap.x.npos + 0.5*dt*(_fdm_trap.xdot.npos_dot + _xdot_old1.npos_dot);
	_fdm_trap.x.epos = _fdm_trap.x.epos + 0.5*dt*(_fdm_trap.xdot.epos_dot + _xdot_old1.epos_dot);
	_fdm_trap.x.lat = _fdm_trap.x.lat + 0.5*dt*(_fdm_trap.xdot.lat_dot + _xdot_old1.lat_dot);
	_fdm_trap.x.lon = _fdm_trap.x.lon + 0.5*dt*(_fdm_trap.xdot.lon_dot + _xdot_old1.lon_dot);
	_fdm_trap.x.alt = _fdm_trap.x.alt + 0.5*dt*(_fdm_trap.xdot.alt_dot + _xdot_old1.alt_dot);
	_fdm_trap.x.phi = _fdm_trap.x.phi + 0.5*dt*(_fdm_trap.xdot.phi_dot + _xdot_old1.phi_dot);
	_fdm_trap.x.theta = _fdm_trap.x.theta + 0.5*dt*(_fdm_trap.xdot.theta_dot + _xdot_old1.theta_dot);
	_fdm_trap.x.psi = _fdm_trap.x.psi + 0.5*dt*(_fdm_trap.xdot.psi_dot + _xdot_old1.psi_dot);
	_fdm_trap.x.q0 = _fdm_trap.x.q0 + 0.5*dt*(_fdm_trap.xdot.q0_dot + _xdot_old1.q0_dot);
	_fdm_trap.x.q1 = _fdm_trap.x.q1 + 0.5*dt*(_fdm_trap.xdot.q1_dot + _xdot_old1.q1_dot);
	_fdm_trap.x.q2 = _fdm_trap.x.q2 + 0.5*dt*(_fdm_trap.xdot.q2_dot + _xdot_old1.q2_dot);
	_fdm_trap.x.q3 = _fdm_trap.x.q3 + 0.5*dt*(_fdm_trap.xdot.q3_dot + _xdot_old1.q3_dot);
	_fdm_trap.x.p = _fdm_trap.x.p + 0.5*dt*(_fdm_trap.xdot.p_dot + _xdot_old1.p_dot);
	_fdm_trap.x.q = _fdm_trap.x.q + 0.5*dt*(_fdm_trap.xdot.q_dot + _xdot_old1.q_dot);
	_fdm_trap.x.r = _fdm_trap.x.r + 0.5*dt*(_fdm_trap.xdot.r_dot + _xdot_old1.r_dot);
	_fdm_trap.x.u = _fdm_trap.x.u + 0.5*dt*(_fdm_trap.xdot.u_dot + _xdot_old1.u_dot);
	_fdm_trap.x.v = _fdm_trap.x.v + 0.5*dt*(_fdm_trap.xdot.v_dot + _xdot_old1.v_dot);
	_fdm_trap.x.w = _fdm_trap.x.w + 0.5*dt*(_fdm_trap.xdot.w_dot + _xdot_old1.w_dot);
	_fdm_trap.x.vt = _fdm_trap.x.vt + 0.5*dt*(_fdm_trap.xdot.vt_dot  + _xdot_old1.vt_dot);
	_fdm_trap.x.alpha = _fdm_trap.x.alpha + 0.5*dt*(_fdm_trap.xdot.alpha_dot + _xdot_old1.alpha_dot);
	_fdm_trap.x.beta = _fdm_trap.x.beta + 0.5*dt*(_fdm_trap.xdot.beta_dot + _xdot_old1.beta_dot);
	_fdm_trap.x.power = _fdm_trap.x.power + 0.5*dt*(_fdm_trap.xdot.power_dot + _xdot_old1.power_dot);
	// vs is special here because it is equal to alt derivative
	_fdm_trap.x.vs =  0.5*(_fdm_trap.xdot.alt_dot + _xdot_old1.alt_dot);
	_fdm_trap.x.time_sec+=dt;
	
	// Integrate Adam Bashworth 2
	_fdm_adba2.x.npos = _fdm_adba2.x.npos + dt*(1.5*_fdm_adba2.xdot.npos_dot - 0.5*_xdot_old1.npos_dot);
	_fdm_adba2.x.epos = _fdm_adba2.x.epos + dt*(1.5*_fdm_adba2.xdot.epos_dot - 0.5*_xdot_old1.epos_dot);
	_fdm_adba2.x.lat = _fdm_adba2.x.lat + dt*(1.5*_fdm_adba2.xdot.lat_dot - 0.5*_xdot_old1.lat_dot);
	_fdm_adba2.x.lon = _fdm_adba2.x.lon + dt*(1.5*_fdm_adba2.xdot.lon_dot - 0.5*_xdot_old1.lon_dot);
	_fdm_adba2.x.alt = _fdm_adba2.x.alt + dt*(1.5*_fdm_adba2.xdot.alt_dot - 0.5*_xdot_old1.alt_dot);
	_fdm_adba2.x.phi = _fdm_adba2.x.phi + dt*(1.5*_fdm_adba2.xdot.phi_dot - 0.5*_xdot_old1.phi_dot);
	_fdm_adba2.x.theta = _fdm_adba2.x.theta + dt*(1.5*_fdm_adba2.xdot.theta_dot - 0.5*_xdot_old1.theta_dot);
	_fdm_adba2.x.psi = _fdm_adba2.x.psi + dt*(1.5*_fdm_adba2.xdot.psi_dot - 0.5*_xdot_old1.psi_dot);
	_fdm_adba2.x.q0 = _fdm_adba2.x.q0 + dt*(1.5*_fdm_adba2.xdot.q0_dot - 0.5*_xdot_old1.q0_dot);
	_fdm_adba2.x.q1 = _fdm_adba2.x.q1 + dt*(1.5*_fdm_adba2.xdot.q1_dot - 0.5*_xdot_old1.q1_dot);
	_fdm_adba2.x.q2 = _fdm_adba2.x.q2 + dt*(1.5*_fdm_adba2.xdot.q2_dot - 0.5*_xdot_old1.q2_dot);
	_fdm_adba2.x.q3 = _fdm_adba2.x.q3 + dt*(1.5*_fdm_adba2.xdot.q3_dot - 0.5*_xdot_old1.q3_dot);
	_fdm_adba2.x.p = _fdm_adba2.x.p + dt*(1.5*_fdm_adba2.xdot.p_dot - 0.5*_xdot_old1.p_dot);
	_fdm_adba2.x.q = _fdm_adba2.x.q + dt*(1.5*_fdm_adba2.xdot.q_dot - 0.5*_xdot_old1.q_dot);
	_fdm_adba2.x.r = _fdm_adba2.x.r + dt*(1.5*_fdm_adba2.xdot.r_dot - 0.5*_xdot_old1.r_dot);
	_fdm_adba2.x.u = _fdm_adba2.x.u + dt*(1.5*_fdm_adba2.xdot.u_dot - 0.5*_xdot_old1.u_dot);
	_fdm_adba2.x.v = _fdm_adba2.x.v + dt*(1.5*_fdm_adba2.xdot.v_dot - 0.5*_xdot_old1.v_dot);
	_fdm_adba2.x.w = _fdm_adba2.x.w + dt*(1.5*_fdm_adba2.xdot.w_dot - 0.5*_xdot_old1.w_dot);
	_fdm_adba2.x.vt = _fdm_adba2.x.vt + dt*(1.5*_fdm_adba2.xdot.vt_dot  - 0.5*_xdot_old1.vt_dot);
	_fdm_adba2.x.alpha = _fdm_adba2.x.alpha + dt*(1.5*_fdm_adba2.xdot.alpha_dot - 0.5*_xdot_old1.alpha_dot);
	_fdm_adba2.x.beta = _fdm_adba2.x.beta + dt*(1.5*_fdm_adba2.xdot.beta_dot - 0.5*_xdot_old1.beta_dot);
	_fdm_adba2.x.power = _fdm_adba2.x.power + dt*(1.5*_fdm_adba2.xdot.power_dot - 0.5*_xdot_old1.power_dot);
	// vs is special here because it is equal to alt derivative
	_fdm_adba2.x.vs =  (1.5*_fdm_adba2.xdot.alt_dot - 0.5*_xdot_old1.alt_dot);
	_fdm_adba2.x.time_sec+=dt;
	
	// Integrate Adam Bashworth 3
	_fdm_adba3.x.npos = _fdm_adba3.x.npos + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.npos_dot - 16.0*_xdot_old1.npos_dot + 5.0*_xdot_old2.npos_dot);
	_fdm_adba3.x.epos = _fdm_adba3.x.epos + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.epos_dot - 16.0*_xdot_old1.epos_dot + 5.0*_xdot_old2.epos_dot);
	_fdm_adba3.x.lat = _fdm_adba3.x.lat + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.lat_dot - 16.0*_xdot_old1.lat_dot + 5.0*_xdot_old2.lat_dot);
	_fdm_adba3.x.lon = _fdm_adba3.x.lon + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.lon_dot - 16.0*_xdot_old1.lon_dot + 5.0*_xdot_old2.lon_dot);
	_fdm_adba3.x.alt = _fdm_adba3.x.alt + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.alt_dot - 16.0*_xdot_old1.alt_dot + 5.0*_xdot_old2.alt_dot);
	_fdm_adba3.x.phi = _fdm_adba3.x.phi + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.phi_dot - 16.0*_xdot_old1.phi_dot + 5.0*_xdot_old2.phi_dot);
	_fdm_adba3.x.theta = _fdm_adba3.x.theta + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.theta_dot - 16.0*_xdot_old1.theta_dot + 5.0*_xdot_old2.theta_dot);
	_fdm_adba3.x.psi = _fdm_adba3.x.psi + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.psi_dot - 16.0*_xdot_old1.psi_dot + 5.0*_xdot_old2.psi_dot);
	_fdm_adba3.x.q0 = _fdm_adba3.x.q0 + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.q0_dot - 16.0*_xdot_old1.q0_dot + 5.0*_xdot_old2.q0_dot);
	_fdm_adba3.x.q1 = _fdm_adba3.x.q1 + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.q1_dot - 16.0*_xdot_old1.q1_dot + 5.0*_xdot_old2.q1_dot);
	_fdm_adba3.x.q2 = _fdm_adba3.x.q2 + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.q2_dot - 16.0*_xdot_old1.q2_dot + 5.0*_xdot_old2.q2_dot);
	_fdm_adba3.x.q3 = _fdm_adba3.x.q3 + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.q3_dot - 16.0*_xdot_old1.q3_dot + 5.0*_xdot_old2.q3_dot);
	_fdm_adba3.x.p = _fdm_adba3.x.p + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.p_dot - 16.0*_xdot_old1.p_dot + 5.0*_xdot_old2.p_dot);
	_fdm_adba3.x.q = _fdm_adba3.x.q + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.q_dot - 16.0*_xdot_old1.q_dot + 5.0*_xdot_old2.q_dot);
	_fdm_adba3.x.r = _fdm_adba3.x.r + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.r_dot - 16.0*_xdot_old1.r_dot + 5.0*_xdot_old2.r_dot);
	_fdm_adba3.x.u = _fdm_adba3.x.u + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.u_dot - 16.0*_xdot_old1.u_dot + 5.0*_xdot_old2.u_dot);
	_fdm_adba3.x.v = _fdm_adba3.x.v + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.v_dot - 16.0*_xdot_old1.v_dot + 5.0*_xdot_old2.v_dot);
	_fdm_adba3.x.w = _fdm_adba3.x.w + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.w_dot - 16.0*_xdot_old1.w_dot + 5.0*_xdot_old2.w_dot);
	_fdm_adba3.x.vt = _fdm_adba3.x.vt + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.vt_dot  - 16.0*_xdot_old1.vt_dot + 5.0*_xdot_old2.vt_dot);
	_fdm_adba3.x.alpha = _fdm_adba3.x.alpha + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.alpha_dot - 16.0*_xdot_old1.alpha_dot + 5.0*_xdot_old2.alpha_dot);
	_fdm_adba3.x.beta = _fdm_adba3.x.beta + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.beta_dot - 16.0*_xdot_old1.beta_dot + 5.0*_xdot_old2.beta_dot);
	_fdm_adba3.x.power = _fdm_adba3.x.power + (1.0/12.0)*dt*(23.0*_fdm_adba3.xdot.power_dot - 16.0*_xdot_old1.power_dot + 5.0*_xdot_old2.power_dot);
	// vs is special here because it is equal to alt derivative
	_fdm_adba3.x.vs =  (1.0/12.0)*(23.0*_fdm_adba3.xdot.alt_dot - 16.0*_xdot_old1.alt_dot + 5.0*_xdot_old2.alt_dot);
	_fdm_adba3.x.time_sec+=dt;
	
	// Control surfaces
	compute_cs(dt);
	
	// select fdm
	//_fdm = _fdm_euler;
	
	std::wcout << L" _fdm.x.vt = " << _fdm.x.vt << std::endl;
	std::wcout << L" _fdm.x.beta = " << _fdm.x.beta << std::endl;
	std::wcout << L" _fdm.x.alpha = " << _fdm.x.alpha << std::endl;
	std::wcout << L" _fdm.x.beta_deg = " << _fdm.x.beta * _r2d << std::endl;
	std::wcout << L" _fdm.x.alpha_deg = " << _fdm.x.alpha * _r2d << std::endl;
	std::wcout << L" _fdm.x.phi = " << _fdm.x.phi << std::endl;
	std::wcout << L" _fdm.x.theta= " << _fdm.x.theta << std::endl;
	std::wcout << L" _fdm.x.psi = " << _fdm.x.psi << std::endl;
	std::wcout << L" _fdm.x.npos = " << _fdm.x.npos << std::endl;
	std::wcout << L" _fdm.x.epos = " << _fdm.x.epos << std::endl;
	std::wcout << L" _fdm.x.alt = " << _fdm.x.alt << std::endl;
	std::wcout << L" _fdm.x.thrust = " << _fdm.x.thrust << std::endl;
	std::wcout << L" _fdm.u.de = " << _fdm.u.de  << std::endl;
	std::wcout << L" _fdm.u.da = " << _fdm.u.da  << std::endl;
	std::wcout << L" _fdm.u.dlef = " << _fdm.u.dlef  << std::endl;
	std::wcout << L" _fdm.x.qbar = " << _fdm.x.qbar  << std::endl;
	std::wcout << L" _fdm.x.ps = " << _fdm.x.ps  << std::endl;
    std::wcout << L"" <<  std::endl; 

}
//---------------------------------------------------------------------
// x[6] = [alpha,da,de, dr,dth];
float f16_fdm_f::trim_funct(std::vector<float> inputs)
{
	float sa, ca, sb, cb, tb, st, ct, tt, sphi, cphi, spsi, cpsi;
	float r2d, d2r;
	float g;
	
	float CX_tot, CY_tot, CZ_tot, Cl_tot, Cm_tot, Cn_tot;
	float Cx, Cz, Cm, Cy, Cn, Cl;
    float Cxq, Cyr, Cyp, Czq, Clr, Clp, Cmq, Cnr, Cnp;
    float delta_Cx_lef, delta_Cz_lef,delta_Cm_lef;
    float delta_Cy_lef,delta_Cn_lef,delta_Cl_lef;
    float delta_Cxq_lef, delta_Cyr_lef, delta_Cyp_lef, delta_Czq_lef;
    float delta_Clr_lef, delta_Clp_lef, delta_Cmq_lef, delta_Cnr_lef;
    float delta_Cnp_lef;
    float delta_Cy_r30, delta_Cn_r30, delta_Cl_r30;
    float delta_Cy_a20, delta_Cy_a20_lef, delta_Cn_a20, delta_Cn_a20_lef;
    float delta_Cl_a20, delta_Cl_a20_lef;
    float delta_Cnbeta, delta_Clbeta, delta_Cm, eta_el, delta_Cm_ds;
    float dq;
    
    float da_norm, dr_norm, dlef_norm;
    float alpha_deg, beta_deg, alpha_lef_deg;
    
	float cost = 0.0;
	float lbf_to_N     = 4.448222;
	r2d = 57.29577951;
	
	// initial conditions for trimming
	_fdm.x.vt = _trim_cond.vt;		
	_fdm.x.npos = _trim_cond.npos;
	_fdm.x.epos = _trim_cond.epos;
	_fdm.x.alt = _trim_cond.alt;
	_fdm.x.p = _trim_cond.p;
	_fdm.x.q = _trim_cond.q;
	_fdm.x.r = _trim_cond.r;
	_fdm.x.psi =  _trim_cond.psi;
	//Free parameters from trimming nelder meld tests
	//_fdm.x.thrust  = inputs[0];
	_fdm.x.alpha  = inputs[0];
	_fdm.u.de    =  inputs[1];
	_fdm.u.da    =  inputs[2];
	_fdm.u.dr    =  inputs[3];
	_fdm.u.dth  = inputs[4];
	float expected_thrust = inputs[5];
	
	// limit alpha et beta
	//if (_fdm.x.alpha > 90.0*d2r) _fdm.x.alpha = 90.0*d2r;
	//else if (_fdm.x.alpha < -20.0*d2r) _fdm.x.alpha = -20.0*d2r;
	if (_fdm.x.alpha > 0.3) _fdm.x.alpha = 0.3;
	else if (_fdm.x.alpha < 0.0) _fdm.x.alpha = 0.0;
	
	if (_fdm.x.beta > 30.0*d2r) _fdm.x.beta = 30.0*d2r;
	else if (_fdm.x.beta < -30.0*d2r) _fdm.x.beta = -30.0*d2r;
	alpha_deg = _fdm.x.alpha * r2d;
	alpha_lef_deg = alpha_deg;
	beta_deg = _fdm.x.beta * r2d;
	if (alpha_lef_deg > 45.0) alpha_lef_deg = 45.0;
	
		// derivated/stated from initial conditions
	_fdm.x.phi = 0.0;
	_fdm.x.theta = _fdm.x.alpha;
	_fdm.x.beta = 0.0;
	_fdm.x.u = _fdm.x.vt*cos(_fdm.x.alpha);
	_fdm.x.v = 0.0;
	_fdm.x.w = _fdm.x.vt*sin(_fdm.x.alpha);
	
	// atmospheric - to be moved
	static float g0     =  9.80665;
	static float T0     =  288.15;
	static float rho0   =  1.225;
	static float p0     =  101325;
	static float lambda = -0.0065;
	static float GASCON =  287.05;
	static float gammah =  1.4;
	static float RADIUS =  6371020;
	static float Hs     =  11000;
	float he, Hgeo, mu, M, pa;
	float ptot, prel, qc, qrel, qdyn; 
	float rho, Reynl, T, Ttot; 
	float VCAS, VEAS, VIAS, Vsound, VTAS;
	
	VTAS = _fdm.x.vt;
	he   = _fdm.x.alt;
	
	/* atmospheric data */
	Hgeo   = RADIUS*he/(RADIUS+he);

	if (Hgeo<=Hs) 
	{
		T   = T0+lambda*Hgeo;
		rho = rho0*pow((T/T0),-(g0/(GASCON*lambda)+1));
	}
	else 
	{
		T   = T0+lambda*Hs;
		rho = rho0*pow((T/T0),-(g0/(GASCON*lambda)+1));
		rho = rho*exp(-g0/(GASCON*T)*(Hgeo-Hs));
	}

	pa     = rho*GASCON*T;
	g      = g0*(RADIUS*RADIUS)/((RADIUS+he)*(RADIUS+he));
	Vsound = sqrt(gammah*GASCON*T);

	/* airdata 1 */
	M      = VTAS/Vsound;
	qdyn   = 0.5*rho*VTAS*VTAS;

	/* airdata 2 */
	mu    = 1.458e-6*pow(T,1.5)/(T+110.4);
	Reynl = rho*VTAS/mu;

	qrel = pow((1+0.2*M*M),3.5)-1;
	qc   = pa*qrel;
	ptot = pa+qc;
	Ttot = T*(1+0.2*M*M);

	VEAS = sqrt(2*qdyn/rho0);
	VCAS = sqrt(2*3.5*p0/rho0*(pow((1+qc/p0),(1/3.5))-1));
	VIAS = VCAS;
	
	_fdm.x.qbar = qdyn;
	_fdm.x.ps = pa;
	_fdm.x.mach = M;
	
	
	_fdm.u.dlef = 1.38*alpha_lef_deg - 9.05*_fdm.x.qbar/_fdm.x.ps + 1.45;
	
	_fdm.x.power = tgear(_fdm.u.dth);
	_fdm.xdot.power_dot = power_dot(_fdm.u.dth * 100.0, _fdm.x.power);
	_fdm.x.thrust = thrust(_fdm.x.power, _fdm.x.mach, _fdm.x.alt );

      
	
	// limit inputs - controls
	if (_fdm.u.de > 25.0) _fdm.u.de = 25.0;
	else if (_fdm.u.de < -25.0) _fdm.u.de = -25.0;
	if (_fdm.u.da > 21.5) _fdm.u.da = 21.5;
	else if (_fdm.u.da < -21.5) _fdm.u.da = -21.5;
	if (_fdm.u.dr > 30.0) _fdm.u.dr = 30.0;
	else if (_fdm.u.dr < -30.0) _fdm.u.dr = -30.0;
	if (_fdm.u.dlef > 25.0) _fdm.u.dlef = 25.0;
	else if (_fdm.u.dlef < 0.0) _fdm.u.dlef = 0.0;
	if (_fdm.u.dth > 1.0) _fdm.u.dth = 1.0;
	else if (_fdm.u.dth < 0.0) _fdm.u.dth = 0.0;
	if (_fdm.x.thrust > 19000 * lbf_to_N) _fdm.x.thrust = 19000 * lbf_to_N;
	else if (_fdm.x.thrust < 1000.0 * lbf_to_N) _fdm.x.thrust = 1000 * lbf_to_N;
	
	_fdm.u.da_norm = _fdm.u.da/21.5;
    _fdm.u.dr_norm = _fdm.u.dr/30.0;
    _fdm.u.dlef_norm = 1 - (_fdm.u.dlef/25.0);
    
    // assign values to u and control surfaces
    _pos_ele_r = _fdm.u.de;
	_pos_ele_l = _fdm.u.de;
	_pos_ail_r = _fdm.u.da;
	_pos_ail_l = -_fdm.u.da;
	_pos_lef_r = _fdm.u.dlef;
	_pos_lef_l = _fdm.u.dlef;
	_pos_tef_r = 0.0;
	_pos_tef_l = 0.0;
	_pos_rud = _fdm.u.dr;
	
	sa    = sin(_fdm.x.alpha); // sin(alpha) 
	ca    = cos(_fdm.x.alpha); // cos(alpha) 
	sb    = sin(_fdm.x.beta); // sin(beta)  
	cb    = cos(_fdm.x.beta); // cos(beta)  
	tb    = tan(_fdm.x.beta); 

	st    = sin(_fdm.x.theta);
	ct    = cos(_fdm.x.theta);
	tt    = tan(_fdm.x.theta);
	sphi  = sin(_fdm.x.phi);
	cphi  = cos(_fdm.x.phi);
	spsi  = sin(_fdm.x.psi);
	cpsi  = cos(_fdm.x.psi);
	
	// Navigation equations
	_fdm.xdot.npos_dot = _fdm.x.u*(ct*cpsi) + 
						_fdm.x.v*(sphi*cpsi*st - cphi*spsi) + 
						_fdm.x.w*(cphi*st*cpsi + sphi*spsi);
	_fdm.xdot.epos_dot = _fdm.x.u*(ct*spsi) + 
						_fdm.x.v*(sphi*spsi*st + cphi*cpsi) + 
						_fdm.x.w*(cphi*st*spsi - sphi*cpsi);
	_fdm.xdot.alt_dot = _fdm.x.u*st - _fdm.x.v*(sphi*ct) - _fdm.x.w*(cphi*ct);
	// Kinematic equations - Euler angles
	_fdm.xdot.phi_dot = _fdm.x.p + tt*(_fdm.x.q*sphi + _fdm.x.r*cphi);
	_fdm.xdot.theta_dot = _fdm.x.q*cphi - _fdm.x.r*sphi;
	_fdm.xdot.psi_dot = (_fdm.x.q*sphi + _fdm.x.r*cphi)/ct;
	// Kinematic equations - Quaternions
	_fdm.xdot.q0_dot      = 0.5 * (-_fdm.x.p * _fdm.x.q1 - _fdm.x.q * _fdm.x.q2 - _fdm.x.r * _fdm.x.q3);
    _fdm.xdot.q1_dot      = 0.5 * ( _fdm.x.p * _fdm.x.q0 + _fdm.x.r * _fdm.x.q2 - _fdm.x.q * _fdm.x.q3);
    _fdm.xdot.q2_dot      = 0.5 * ( _fdm.x.q * _fdm.x.q0 - _fdm.x.r * _fdm.x.q1 + _fdm.x.p * _fdm.x.q3);
    _fdm.xdot.q3_dot      = 0.5 * ( _fdm.x.r * _fdm.x.q0 + _fdm.x.q * _fdm.x.q1 - _fdm.x.p * _fdm.x.q2);
    
    /* correction term from User Manual by K. Refson */
    dq = _fdm.x.q0 * _fdm.xdot.q0_dot + _fdm.x.q1 * _fdm.xdot.q1_dot + _fdm.x.q2 * _fdm.xdot.q2_dot + _fdm.x.q3 * _fdm.xdot.q3_dot;
    
    _fdm.xdot.q0_dot -= dq * _fdm.x.q0;
    _fdm.xdot.q1_dot -= dq * _fdm.x.q1;
    _fdm.xdot.q2_dot -= dq * _fdm.x.q2;
    _fdm.xdot.q3_dot -= dq * _fdm.x.q3;
	
	// Lookup table
	Cx = getCx(alpha_deg, beta_deg, _fdm.u.de);
	Cy = getCy(alpha_deg, beta_deg);
	Cz = getCz(alpha_deg, beta_deg, _fdm.u.de);
	Cl = getCl(alpha_deg, beta_deg, _fdm.u.de);
	Cm = getCm(alpha_deg, beta_deg, _fdm.u.de);
	Cn = getCn(alpha_deg, beta_deg, _fdm.u.de);
	
	Cxq = getCXq(alpha_deg);
	Cyp = getCYp(alpha_deg);
	Cyr = getCYr(alpha_deg);
	Czq = getCZq(alpha_deg);
	Clp = getCLp(alpha_deg);
	Clr = getCLr(alpha_deg);
	Cmq = getCMq(alpha_deg);
	Cnp = getCNp(alpha_deg);
	Cnr = getCNr(alpha_deg);
	
	delta_Cx_lef = getCx_lef(alpha_lef_deg, beta_deg) - getCx_dh_0(alpha_deg, beta_deg); 
	delta_Cy_lef = getCy_lef(alpha_lef_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cz_lef = getCz_lef(alpha_lef_deg, beta_deg) - getCz_dh_0(alpha_deg, beta_deg);
	delta_Cl_lef = getCl_lef(alpha_lef_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cm_lef = getCm_lef(alpha_lef_deg, beta_deg) - getCm_dh_0(alpha_deg, beta_deg);
	delta_Cn_lef = getCn_lef(alpha_lef_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	
	delta_Cxq_lef = getdCXq_lef(alpha_lef_deg);
	delta_Cyp_lef = getdCYp_lef(alpha_lef_deg);
	delta_Cyr_lef = getdCYr_lef(alpha_lef_deg);
	delta_Czq_lef = getdCZq_lef(alpha_lef_deg);
	delta_Clp_lef = getdCLp_lef(alpha_lef_deg);
	delta_Clr_lef = getdCLr_lef(alpha_lef_deg);
	delta_Cmq_lef = getdCMq_lef(alpha_lef_deg);
	delta_Cnp_lef = getdCNp_lef(alpha_lef_deg);
	delta_Cnr_lef = getdCNr_lef(alpha_lef_deg);
	
	delta_Cy_r30 = getCy_r30(alpha_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cl_r30 = getCl_r30(alpha_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cn_r30 = getCn_r30(alpha_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	
	delta_Cy_a20     = getCy_a20(alpha_deg, beta_deg) - getCy(alpha_deg, beta_deg);
	delta_Cy_a20_lef = getCy_a20_lef(alpha_lef_deg, beta_deg) - getCy_lef(alpha_lef_deg, beta_deg) - delta_Cy_a20;
	delta_Cn_a20     = getCn_a20(alpha_deg, beta_deg) - getCn_dh_0(alpha_deg, beta_deg);
	delta_Cn_a20_lef = getCn_a20_lef(alpha_lef_deg, beta_deg) - getCn_lef(alpha_lef_deg, beta_deg) - delta_Cn_a20;
	delta_Cl_a20     = getCl_a20(alpha_deg, beta_deg) - getCl_dh_0(alpha_deg, beta_deg);
	delta_Cl_a20_lef = getCl_a20_lef(alpha_lef_deg, beta_deg) - getCl_lef(alpha_lef_deg, beta_deg) - delta_Cl_a20;
	
	delta_Cnbeta = getdCn_beta(alpha_deg);
	delta_Clbeta = getdCl_beta(alpha_deg);
	delta_Cm     = getdCm(alpha_deg);
	eta_el       = getEta_el(_fdm.u.de);
	delta_Cm_ds  = getdCm_ds(alpha_deg,_fdm.u.de);
	
	/* Total force coefficients */
    /* Cx_tot */
    CX_tot = Cx + delta_Cx_lef * dlef_norm
            + (_fdm.mi.cref/(2*_fdm.x.vt))*(Cxq + delta_Cxq_lef * _fdm.u.dlef_norm) * _fdm.x.q;
    /* Cy_tot */
    CY_tot = Cy + delta_Cy_lef * dlef_norm
            + (delta_Cy_a20 + delta_Cy_a20_lef * dlef_norm) * _fdm.u.da_norm
            + delta_Cy_r30 * _fdm.u.dr_norm
            + (_fdm.mi.bref / (2*_fdm.x.vt))*(Cyr + delta_Cyr_lef * _fdm.u.dlef_norm) * _fdm.x.r
            + (_fdm.mi.bref/(2*_fdm.x.vt))*(Cyp + delta_Cyp_lef * _fdm.u.dlef_norm) * _fdm.x.p;
    /* Cz_tot */
    CZ_tot = Cz + delta_Cz_lef * _fdm.u.dlef_norm
            + (_fdm.mi.cref/(2*_fdm.x.vt))*(Czq + delta_Czq_lef * _fdm.u.dlef_norm) * _fdm.x.q;
    
    /* Total moment coefficients */
    /* Cl_tot */
    Cl_tot = Cl + delta_Cl_lef * _fdm.u.dlef_norm
            + (delta_Cl_a20 + delta_Cl_a20_lef * _fdm.u.dlef_norm) * _fdm.u.da_norm
            + delta_Cl_r30 * _fdm.u.dr_norm
            + (_fdm.mi.bref / ((2*_fdm.x.vt))*(Clr + delta_Clr_lef * _fdm.u.dlef_norm)) * _fdm.x.r
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Clp + delta_Clp_lef * _fdm.u.dlef_norm)) * _fdm.x.p
            + delta_Clbeta * _fdm.x.beta * r2d;
    /* Cm_tot */
    Cm_tot = Cm * eta_el + CZ_tot * (_fdm.mi.xcgr - _fdm.mi.xcg) + delta_Cm_lef * _fdm.u.dlef_norm
            + (_fdm.mi.cref / (2*_fdm.x.vt))*(Cmq + delta_Cmq_lef * _fdm.u.dlef_norm) * _fdm.x.q
            + delta_Cm + delta_Cm_ds;
    /* Cn_tot */
    Cn_tot = Cn + delta_Cn_lef * _fdm.u.dlef_norm
            - CY_tot * (_fdm.mi.xcgr - _fdm.mi.xcg)*(_fdm.mi.cref/_fdm.mi.bref)
            + (delta_Cn_a20 + delta_Cn_a20_lef * dlef_norm) * _fdm.u.da_norm
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Cnr + delta_Cnr_lef * _fdm.u.dlef_norm))* _fdm.x.r
            + ((_fdm.mi.bref / (2*_fdm.x.vt)) * (Cnp + delta_Cnp_lef * _fdm.u.dlef_norm)) * _fdm.x.p
            + delta_Cn_r30 * _fdm.u.dr_norm + delta_Cnbeta * _fdm.x.beta * r2d;
            
    // Udot,Vdot, Wdot,(as on NASA report TP-1538 p36) 
	_fdm.xdot.u_dot = _fdm.x.r*_fdm.x.v - _fdm.x.q * _fdm.x.w - g*st + _fdm.x.qbar*_fdm.mi.sref*CX_tot/_fdm.mi.mass + _fdm.x.thrust/_fdm.mi.mass;
	_fdm.xdot.v_dot = _fdm.x.p*_fdm.x.w - _fdm.x.r * _fdm.x.u + g*ct*sphi + _fdm.x.qbar*_fdm.mi.sref*CY_tot/_fdm.mi.mass;
	_fdm.xdot.w_dot = _fdm.x.q*_fdm.x.u - _fdm.x.p * _fdm.x.v + g*ct*cphi + _fdm.x.qbar*_fdm.mi.sref*CZ_tot/_fdm.mi.mass;
	
	// vt_dot equation (from S&L, p82)
	_fdm.xdot.vt_dot = (_fdm.x.u*_fdm.xdot.u_dot + _fdm.x.v*_fdm.xdot.v_dot + _fdm.x.w*_fdm.xdot.w_dot)/_fdm.x.vt;
	
	//alpha_dot et beta_dot
	_fdm.xdot.alpha_dot = (_fdm.x.u*_fdm.xdot.w_dot - _fdm.x.w*_fdm.xdot.u_dot)/(_fdm.x.u*_fdm.x.u + _fdm.x.w*_fdm.x.w);
	_fdm.xdot.beta_dot =  (_fdm.x.vt*_fdm.xdot.v_dot - _fdm.x.v*_fdm.xdot.vt_dot)/(_fdm.x.vt*_fdm.x.vt*cb);
	
	// get moments from coefficients 
	float L_tot, M_tot, N_tot, denom;
	L_tot = Cl_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.bref;      
	M_tot = Cm_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.cref;
	N_tot = Cn_tot*_fdm.x.qbar*_fdm.mi.sref*_fdm.mi.bref;
	denom = _fdm.mi.ixx*_fdm.mi.izz - _fdm.mi.ixz*_fdm.mi.ixz;
	
	_fdm.xdot.p_dot = (_fdm.mi.izz*L_tot + _fdm.mi.ixz*N_tot - (_fdm.mi.izz*(_fdm.mi.izz-_fdm.mi.iyy)+_fdm.mi.ixz*_fdm.mi.ixz)*_fdm.x.q*_fdm.x.r + _fdm.mi.ixz*(_fdm.mi.ixx-_fdm.mi.iyy+_fdm.mi.izz)*_fdm.x.p*_fdm.x.q + _fdm.mi.ixz*_fdm.x.q*_fdm.mi.heng)/denom;
	_fdm.xdot.q_dot = (M_tot + (_fdm.mi.izz-_fdm.mi.ixx)*_fdm.x.p*_fdm.x.r - _fdm.mi.ixz*(_fdm.x.p*_fdm.x.p - _fdm.x.r*_fdm.x.r) - _fdm.x.r*_fdm.mi.heng)/_fdm.mi.iyy;
	_fdm.xdot.r_dot = (_fdm.mi.ixx*N_tot + _fdm.mi.ixz*L_tot + (_fdm.mi.ixx*(_fdm.mi.ixx-_fdm.mi.iyy)+_fdm.mi.ixz*_fdm.mi.ixz)*_fdm.x.p*_fdm.x.q - _fdm.mi.ixz*(_fdm.mi.ixx-_fdm.mi.iyy+_fdm.mi.izz)*_fdm.x.q*_fdm.x.r + _fdm.mi.ixx*_fdm.x.q*_fdm.mi.heng)/denom;
	
	
	std::wcout << L"  _fdm.xdot.vt_dot = " <<  _fdm.xdot.vt_dot << std::endl;
	std::wcout << L" _fdm.xdot.beta_dot = " << _fdm.xdot.beta_dot << std::endl;
	std::wcout << L" _fdm.xdot.alpha_dot = " << _fdm.xdot.alpha_dot << std::endl;
	std::wcout << L" _fdm.xdot.alt_dot = " << _fdm.xdot.alt_dot << std::endl;
	
	// to trim fcs information
	u2fcs();
	
// ONCE TRIMMED, INITIALIZE ALL ODEs FDM structures
	_fdm_euler = _fdm;
	_fdm_trap = _fdm;
	_fdm_adba2 = _fdm;
	_fdm_adba3 = _fdm;

	cost = 10  * _fdm.xdot.vt_dot * _fdm.xdot.vt_dot
	     + 10  * _fdm.xdot.alt_dot * _fdm.xdot.alt_dot
	     + 100 * _fdm.xdot.alpha_dot * _fdm.xdot.alpha_dot
	     + 5 * _fdm.xdot.beta_dot * _fdm.xdot.beta_dot
	     //+ 1  * _f16_fdm_var.q0_dot * _f16_fdm_var.q0_dot
	     //+ 1  * _f16_fdm_var.q1_dot * _f16_fdm_var.q1_dot
	     //+ 1  * _f16_fdm_var.q2_dot * _f16_fdm_var.q2_dot
	     //+ 1  * _f16_fdm_var.q3_dot * _f16_fdm_var.q3_dot
	     //+ 2  * _f16_fdm_var.w_body_dot * _f16_fdm_var.w_body_dot
	     + 10  * _fdm.xdot.p_dot * _fdm.xdot.p_dot
	     + 10  * _fdm.xdot.q_dot * _fdm.xdot.q_dot
	     + 10  * _fdm.xdot.r_dot * _fdm.xdot.r_dot;
	     //+ 0   * _f16_fdm_var.x_earth_dot * _f16_fdm_var.x_earth_dot
	     //+ 0   * _f16_fdm_var.y_earth_dot * _f16_fdm_var.y_earth_dot
	     //+ 5   * _f16_fdm_var.z_earth_dot * _f16_fdm_var.z_earth_dot;
	     //+ 2  * _f16_fdm_var.pow_dot * _f16_fdm_var.pow_dot;
	
	return cost;
	
}



//---------------------------------------------------------------------
// 
void f16_fdm_f::trim_aircraft()
{
	
	float parameters[6] =  {_trim_init.alpha, _trim_init.de, _trim_init.da, _trim_init.dr, _trim_init.dth, _trim_init.thrust};
	std::vector<float> init(parameters, parameters+6);
	std::vector<std::vector<float> > x =  std::vector<std::vector<float> >();
	
	int iterations=1E5;
	float tol=1E5*std::numeric_limits<float>::epsilon();
	
	// SIMPLEX METHOD IMPLEMENTATION 
	//space dimension
	int N=init.size();                         
	// coefficients
	//a: reflection  -> xr  
	//b: expansion   -> xe 
	//g: contraction -> xc
	//h: full contraction to x1
    const float a=1.0, b=1.0, g=0.5, h=0.5;   //coefficients

    std::vector<float> xcentroid_old(N,0);   //simplex center * (N+1)
    std::vector<float> xcentroid_new(N,0);   //simplex center * (N+1)
    std::vector<float> vf(N+1,0);            //f evaluated at simplex vertexes       
    int x1=0, xn=0, xnp1=0;         //x1:   f(x1) = min { f(x1), f(x2)...f(x_{n+1} }
                                    //xnp1: f(xnp1) = max { f(x1), f(x2)...f(x_{n+1} }
                                    //xn:   f(xn)<f(xnp1) && f(xn)> all other f(x_i)
    int cnt=0; //iteration step number
    
	if(x.size()==0) //if no initial simplex is specified
	{ //construct the trial simplex
		//based upon the initial guess parameters
		std::vector<float> del( init );
		std::transform(del.begin(), del.end(), del.begin(), 
			std::bind2nd( std::divides<float>() , 100) );//'20' is picked 
													 //assuming initial trail close to true

		for(int i=0; i<N; ++i)
		{
			std::vector<float> tmp( init );
			tmp[i] +=  del[i];
			x.push_back( tmp );
		}
		x.push_back(init);//x.size()=N+1, x[i].size()=N

		//xcentriod
		std::transform(init.begin(), init.end(), 
		xcentroid_old.begin(), std::bind2nd(std::multiplies<float>(), N+1) );
	}//constructing the simplex finished

    //optimization begins
    for(cnt=0; cnt<iterations; ++cnt)
    {
		for(int i=0;i<N+1;++i)
		{
			vf[i]= trim_funct(x[i]); // function to optimize
		}
		x1=0; xn=0; xnp1=0;//find index of max, second max, min of vf.
      
		for(int i=0;i<vf.size();++i)
		{
			if(vf[i]<vf[x1]) x1=i;
			if(vf[i]>vf[xnp1])xnp1=i;
		}
		xn=x1;
      
		for(int i=0; i<vf.size();++i)
		{ 
			if( vf[i]<vf[xnp1] && vf[i]>vf[xn]) xn=i;
		}
		//x1, xn, xnp1 are found

		std::vector<float> xg(N, 0);//xg: centroid of the N best vertexes
		for(int i=0; i<x.size(); ++i)
		{
			if(i!=xnp1)
			std::transform(xg.begin(), xg.end(), x[i].begin(), xg.begin(), std::plus<float>() );
		}
		std::transform(xg.begin(), xg.end(), x[xnp1].begin(), xcentroid_new.begin(), std::plus<float>());
		std::transform(xg.begin(), xg.end(), xg.begin(), std::bind2nd(std::divides<float>(), N) );
		//xg found, xcentroid_new updated

		//termination condition
		float diff=0.0;          //calculate the difference of the simplex centers
						 //see if the difference is less than the termination criteria
		for(int i=0; i<N; ++i)     
			diff += fabs(xcentroid_old[i]-xcentroid_new[i]);

		if (diff/N < tol) break;              //terminate the optimizer
		else xcentroid_old.swap(xcentroid_new); //update simplex center
      
		//reflection:
		std::vector<float> xr(N,0); 
		for( int i=0; i<N; ++i)
		xr[i]=xg[i]+a*(xg[i]-x[xnp1][i]);
		
		//reflection, xr found
      
		float fxr=trim_funct(xr);//record function at xr
      
		if(vf[x1]<=fxr && fxr<=vf[xn])
		{
			std::copy(xr.begin(), xr.end(), x[xnp1].begin() );
		}
		else if(fxr<vf[x1])
		{ //expansion:
			std::vector<float> xe(N,0);
			for( int i=0; i<N; ++i)
				xe[i]=xr[i]+b*(xr[i]-xg[i]);
			if( trim_funct(xe) < fxr )
				std::copy(xe.begin(), xe.end(), x[xnp1].begin() );
			else
				std::copy(xr.begin(), xr.end(), x[xnp1].begin() );
		}//expansion finished,  xe is not used outside the scope
		else if( fxr > vf[xn] )
		{//contraction:
			std::vector<float> xc(N,0);
			for( int i=0; i<N; ++i)
				xc[i]=xg[i]+g*(x[xnp1][i]-xg[i]);
			if( trim_funct(xc) < vf[xnp1] )
			{
				std::copy(xc.begin(), xc.end(), x[xnp1].begin());
			}
			else
			{
				for( int i=0; i<x.size(); ++i )
				{
					if( i!=x1 )
					{ 
						for(int j=0; j<N; ++j) 
							x[i][j] = x[x1][j] + h * ( x[i][j]-x[x1][j] );
					}
				}
			}
		}//contraction finished, xc is not used outside the scope
	}//optimization is finished

    if(cnt==iterations){//max number of iteration achieves before tol is satisfied
      std::cout<<"Iteration limit achieves, result may not be optimal"<<std::endl;
    }
    
    //memset(&_f16_fdm_var, 0, sizeof(_f16_fdm_var));
	//trim_funct(x[x1]);
	
	std::cout <<"SIMPLEX TRIM DONE : " <<std::endl;
	std::wcout << L" _fdm.x.alpha = " << x[x1][0] << std::endl;
	std::wcout << L" _fdm.x.de = " << x[x1][1]<< std::endl;
	std::wcout << L" _fdm.x.da = " << x[x1][2] << std::endl;
	std::wcout << L" _fdm.x.dr = " << x[x1][3] << std::endl;
	std::wcout << L" _fdm.x.dth = " << x[x1][4] << std::endl;
	std::wcout << L" _fdm.x.thrust = " << _fdm.x.thrust << std::endl;
	
}

