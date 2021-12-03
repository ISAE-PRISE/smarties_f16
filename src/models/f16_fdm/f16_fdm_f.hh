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

#ifndef __F16_FDM_F_HH__
#define __F16_FDM_F_HH__

#include <float.h>
#include <queue>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <limits>
#include <algorithm>
#include <functional>
#include <iostream>
#include <f16_aero_data_f.h>
#include <f16_engine_data_f.h>
#include <f16_data_types_f.h>

// Here all is defined in static for fpga usage
// In case we use it for fpga
#define MAX_TABLE_COL 25
#define MAX_TABLE_ROW 25

class f16_fdm_f
{
	public:
		f16_fdm_f();
		~f16_fdm_f();
		
		void load_tables_aero_f();
		float interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val);
		float interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val);
		
		float getCx(float alpha_deg, float beta_deg, float dh_deg);
		float getCx_dh_0(float alpha_deg, float beta_deg);
		float getCy(float alpha_deg, float beta_deg);
		float getCz(float alpha_deg, float beta_deg, float dh_deg);
		float getCz_dh_0(float alpha_deg, float beta_deg);
		float getCl(float alpha_deg, float beta_deg, float dh_deg);
		float getCl_dh_0(float alpha_deg, float beta_deg);
		float getCm(float alpha_deg, float beta_deg, float dh_deg);
		float getCm_dh_0(float alpha_deg, float beta_deg);
		float getCn(float alpha_deg, float beta_deg, float dh_deg);
		float getCn_dh_0(float alpha_deg, float beta_deg);
		
		float getCXq(float alpha_deg);
		float getCYp(float alpha_deg);
		float getCYr(float alpha_deg);
		float getCZq(float alpha_deg);
		float getCLp(float alpha_deg);
		float getCLr(float alpha_deg);
		float getCMq(float alpha_deg);
		float getCNp(float alpha_deg);
		float getCNr(float alpha_deg);
		
		float getCx_lef(float alpha_lef_deg, float beta_deg);
		float getCy_lef(float alpha_lef_deg, float beta_deg);
		float getCz_lef(float alpha_lef_deg, float beta_deg);
		float getCl_lef(float alpha_lef_deg, float beta_deg);
		float getCm_lef(float alpha_lef_deg, float beta_deg);
		float getCn_lef(float alpha_lef_deg, float beta_deg);
		
		float getdCXq_lef(float alpha_lef_deg);
		float getdCYp_lef(float alpha_lef_deg);
		float getdCYr_lef(float alpha_lef_deg);
		float getdCZq_lef(float alpha_lef_deg);
		float getdCLp_lef(float alpha_lef_deg);
		float getdCLr_lef(float alpha_lef_deg);
		float getdCMq_lef(float alpha_lef_deg);
		float getdCNp_lef(float alpha_lef_deg);
		float getdCNr_lef(float alpha_lef_deg);
		
		float getCy_r30(float alpha_deg, float beta_deg);
		float getCl_r30(float alpha_deg, float beta_deg);
		float getCn_r30(float alpha_deg, float beta_deg);
		
		float getCy_a20(float alpha_deg, float beta_deg);
		float getCy_a20_lef(float alpha_lef_deg, float beta_deg);
		float getCn_a20(float alpha_deg, float beta_deg);
		float getCn_a20_lef(float alpha_lef_deg, float beta_deg);
		float getCl_a20(float alpha_deg, float beta_deg);
		float getCl_a20_lef(float alpha_lef_deg, float beta_deg);
		
		float getdCn_beta(float alpha_deg);
		float getdCl_beta(float alpha_deg);
		float getdCm(float alpha_deg);
		float getEta_el(float dh_deg);
		float getdCm_ds(float alpha_deg,float dh_deg);
		
		// Engines functions 
		float tgear (float throt);
		float rtau (float dp );
		float power_dot( float pw, float cpw );
		float thrust(float power, float mach, float alt_m);
		
		// Control surfaces functions
		void compute_cs(float dt);
		float saturation_pos(float input, float min, float max);
		float saturation_rate(float input_old, float input_new, float max_rate);
		float sign(float value);
		
		// Conversion de fcs to u
		void compute_u();
		void u2fcs();
		void u2cs();
		
		void set_fcs_cmd(f16_fcs_cmd_f_t fcs_cmd);
		f16_fcs_cmd_f_t get_fcs_cmd();
		
		f16_x_f_t get_x();
		f16_fdm_data_f_t get_fdm();
		f16_fdm_data_f_t get_fdm_euler();
		f16_fdm_data_f_t get_fdm_trap();
		f16_fdm_data_f_t get_fdm_adba2();
		f16_fdm_data_f_t get_fdm_adba3();
		
		void compute_xdot(bool is_thrust_external, float thrust_in);
		void compute_x(float dt);
		
		void trim_aircraft();
		float trim_funct(std::vector<float> inputs);
		void set_trim_conditions(float vt_trim
                                   , float npos_trim
                                   , float epos_trim
                                   , float alt_trim
                                   , float p_trim
                                   , float q_trim
                                   , float r_trim
                                   , float psi_trim);
	   void set_trim_init(float thrust_init
						  , float alpha_init
                          , float de_init
                          , float da_init
                          , float dr_init
                          , float dth_init);

	private:
		// Lift Force calculation Tables
		// Horizontal tail deflection (symetric) = dh
		//CX (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -25 deg 
		float _CX_alpha1_beta_dh_n25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];	
		//CX (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -10 deg 
		float _CX_alpha1_beta_dh_n10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CX (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 0 deg 
		float _CX_alpha1_beta_dh_0_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CX (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 10 deg 
		float _CX_alpha1_beta_dh_p10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CX (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 25 deg 
		float _CX_alpha1_beta_dh_p25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CX (alpha2_deg, beta1_deg)  
		float _CX_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CX_q(alpha1_deg)  
		float _CX_q_alpha1_d1_f[MAX_TABLE_ROW][2];
		//CX_q(alpha1_deg)  
		float _dCX_sb_alpha1_d1_f[MAX_TABLE_ROW][2];
		// dCXq_lef(alpha2) in matlab
		float _dCX_q_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		// dCX_tef_alpha1
		float _dCX_tef_alpha1[MAX_TABLE_ROW][2];
		
		
		// Drag Force calculation Tables
		//CZ (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -25 deg 
		float _CZ_alpha1_beta_dh_n25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];	
		//CZ (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -10 deg 
		float _CZ_alpha1_beta_dh_n10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 0 deg 
		float _CZ_alpha1_beta_dh_0_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 10 deg 
		float _CZ_alpha1_beta_dh_p10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = +25 deg 
		float _CZ_alpha1_beta_dh_p25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ (alpha2_deg, beta1_deg)  
		float _CZ_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ_q(alpha1_deg)  
		float _CZ_q_alpha1_d1_f[MAX_TABLE_ROW][2];
		//CZ_q(alpha1_deg)  
		float _dCZ_sb_alpha1_d1_f[MAX_TABLE_ROW][2];
		// dCZq_lef(alpha2) in matlab
		float _dCZ_q_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		// dCZ_tef_alpha1
		float _dCZ_tef_alpha1[MAX_TABLE_ROW][2];
		
		// Side Force calculation Tables
		float _CY_alpha1_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CY (alpha2_deg, beta1_deg)  
		float _CY_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CY (alpha1_deg, beta_deg) table for aileron deflection = 20 deg 
		float _CY_alpha1_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CY (alpha1_deg, beta_deg) table for aileron deflection = 20 deg 
		float _CY_lef_alpha2_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CY (alpha1_deg, beta_deg) table for rudder deflection = 30 deg 
		float _CY_alpha1_beta_dr_30_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//CZ_q(alpha1_deg)  
		float _CY_r_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCY_r_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		float _CY_p_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCY_p_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		
		
		
		//Cl (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -25 deg 
		float _Cl_alpha1_beta_dh_n25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cl (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 0 deg 
		float _Cl_alpha1_beta_dh_0_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cl (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = +25 deg 
		float _Cl_alpha1_beta_dh_p25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _Cl_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cl (alpha1_deg, beta_deg) table for aileron deflection = 20 deg 
		float _Cl_alpha1_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cl lef (alpha2_deg, beta_deg) table for aileron deflection = 20 deg 
		float _Cl_lef_alpha2_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cl (alpha1_deg, beta_deg) table for rudder deflection = 30 deg 
		float _Cl_alpha1_beta_dr_30_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _Cl_r_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCl_beta_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCl_r_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		float _Cl_p_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCl_p_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		
		//Cm (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -25 deg 
		float _Cm_alpha1_beta_dh_n25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];	
		//Cm (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -10 deg 
		float _Cm_alpha1_beta_dh_n10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cm (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 0 deg 
		float _Cm_alpha1_beta_dh_0_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cm (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = 10 deg 
		float _Cm_alpha1_beta_dh_p10_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cm (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = +25 deg 
		float _Cm_alpha1_beta_dh_p25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cm (alpha2_deg, beta1_deg)  
		float _Cm_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cm_sb(alpha1_deg)  
		float _dCm_sb_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCm_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _eta_dh_d1_f[MAX_TABLE_ROW][2];
		float _Cm_q_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCm_q_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		float _dCm_alpha1_ds_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		// dCm_tef_alpha1
		float _dCm_tef_alpha1[MAX_TABLE_ROW][2];
		
		
		
		//Cn (alpha1_deg, beta_deg) table for Horizontal tail (symetric, dh) deflection = -25 deg 
		float _Cn_alpha1_beta_dh_n25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _Cn_alpha1_beta_dh_0_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];	
		float _Cn_alpha1_beta_dh_p25_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cn (alpha2_deg, beta1_deg)  
		float _Cn_lef_alpha2_beta_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cn (alpha1_deg, beta_deg) table for aileron deflection = 20 deg 
		float _Cn_alpha1_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		//Cn lef (alpha2_deg, beta_deg) table for aileron deflection = 20 deg 
		float _Cn_lef_alpha2_beta_da_20_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		
		float _Cn_alpha1_beta_dr_30_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _Cn_r_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCn_beta_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCn_da_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCn_r_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		float _Cn_p_alpha1_d1_f[MAX_TABLE_ROW][2];
		float _dCn_p_lef_alpha2_d1_f[MAX_TABLE_ROW][2];
		
		// Engines handling
		float _thrust_idle_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_mil_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_max_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _pa, _pc, _thrust;
		
		//control surfaces, first order lag filter
		// for now it has position and rate limitation but we may need white noise
		f16_cockpit_data_f_t _cockpit_in; 
		float _tc_ele_r, _pos_ele_r, _pos_ele_r_old, _min_ele_r, _max_ele_r, _max_rate_ele_r; 
		float _tc_ele_l, _pos_ele_l, _pos_ele_l_old, _min_ele_l, _max_ele_l, _max_rate_ele_l; 
		float _tc_ail_r, _pos_ail_r, _pos_ail_r_old, _min_ail_r, _max_ail_r, _max_rate_ail_r; 
		float _tc_ail_l, _pos_ail_l, _pos_ail_l_old, _min_ail_l, _max_ail_l, _max_rate_ail_l;
		float _tc_lef_r, _pos_lef_r, _pos_lef_r_old, _min_lef_r, _max_lef_r, _max_rate_lef_r; 
		float _tc_lef_l, _pos_lef_l, _pos_lef_l_old, _min_lef_l, _max_lef_l, _max_rate_lef_l;
		float _tc_tef_r, _pos_tef_r, _pos_tef_r_old, _min_tef_r, _max_tef_r, _max_rate_tef_r; 
		float _tc_tef_l, _pos_tef_l, _pos_tef_l_old, _min_tef_l, _max_tef_l, _max_rate_tef_l;
		float _tc_rud, _pos_rud, _pos_rud_old, _min_rud, _max_rud, _max_rate_rud;  
		
		//f16 states
		f16_fdm_data_f_t _fdm;
		f16_fdm_data_f_t _fdm_euler;
		f16_fdm_data_f_t _fdm_trap;
		f16_fdm_data_f_t _fdm_adba2;
		f16_fdm_data_f_t _fdm_adba3;
		// f16_xdot_f_t _xdot; useless now as _xdot part of f16_fdm_data_f_t
		f16_xdot_f_t _xdot_old1;
		f16_xdot_f_t _xdot_old2; 
		
		// trim functions
		f16_trim_c_f_t _trim_cond;
		f16_trim_v_f_t _trim_init;
		
		// usefull variables
		float _r2d;
		float _d2r;

};




#endif // __B747_AERO_F_HH__
