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

#ifndef __F16_EFCS_F_HH__
#define __F16_EFCS_F_HH__

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
#include <iostream>
#include <fstream> 
#include <string>
#include <sstream> // sstream usage
#include <algorithm> //Not part of string, use "algorithms" replace()
#include <iostream>

// models
#include <fof_f.hh>
#include <llf_f.hh>
#include <sof_f.hh>
#include <f16_data_types_f.h>
#include <f16_efcs_data_f.h>
#include <pid_f.hh>

// defines for tables
// Here all is defined in static for fpga usage
// In case we use it for fpga
#define MAX_TABLE_COL_AP 2
#define MAX_TABLE_ROW_AP 10

#define F16_AP_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define F16_AP_MIN(x, y) (((x) < (y)) ? (x) : (y))

typedef enum {
    CLB,
    DES,
    ALT,
    ALT_STAR
} ap_long_mode_t;


class f16_efcs_f
{
	public:
		f16_efcs_f();
		~f16_efcs_f();
		
		void load_tables();
		float interp_2d_f(float table[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP], int row_nb, int col_nb, float row_val, float col_val);
		float interp_1d_f(float table[MAX_TABLE_ROW_AP][2], int row_nb, float row_val);
		
		float saturation(float input, float min, float max);
		float deadband(float start, float end, float value);
		void set_state(f16_x_f_t x);
		void set_cockpit(f16_cockpit_data_f_t cockpit_in);
		
		void fbw_pitch(float dt);
		void fbw_roll(float dt);
		void fbw_yaw(float dt);
		void fbw_tef(float dt);
		void fbw_lef(float dt);
		void ap_mode();
		void ap_laws(float dt);
		
		void set_fcs_cmd(f16_fcs_cmd_f fcs_cmd);
		f16_fcs_cmd_f get_fcs_cmd();
		
		// csv log handling
		void enable_log();
		void open_csv(const char *reportfilename);
		void close_csv();
		void write_csv();
		

	private:
	
		// log 
		bool _is_log_enabled;
		std::ofstream _out_csv_file;
		
		float _time_s;
	
		// aircraft state
		f16_x_f_t    _x_real;  // state from fdm
		f16_x_f_t    _x_fuse;  // state from sensor fusion
		
		//fcu orders inputs
		f16_fcu_f _fcu_in;
		
		// side stick trottle inputs
		f16_sst_f _sst_in;    
	
		//flight control out 
		f16_fcs_cmd_f _fcs_out;  
		
		// fbw lef
		llf_f _alpha_lef_lead_lag;
	
		// pitch fbw
		fof_f _stick_pitch_fo_lag;
		fof_f _alpha_fo_lag;  // warning sensor lag
		fof_f _g_fo_lag;
		fof_f _q_fo_lag; // warning sensor lag
		llf_f _q_lead_lag;
		fof_f _bias1_fo_lag; // warning sensor lag
		fof_f _az_fo_lag; // filter for acc Az
		llf_f _step1_lead_lag;
		sof_f _step2_so_filter;
		
		// roll fbw
		fof_f _stick_roll_fo_lag;
		fof_f _p_com_fo_lag; // warning sensor lag
		llf_f _p_com_pos_lead_lag; // "positive" feedback loop
		llf_f _p_com_neg_lead_lag; // "positive" feedback loop
		fof_f _p_fo_lag;
		sof_f _p_so_filter;
		
		float _p_filtered_B; // From roll to yaw
		
		// yaw fbw
		fof_f     _stick_yaw_fo_lag;                   
		fof_f     _omg_r_lag;                
		sof_f _omg_p_yaw;                
		llf_f _u_sum_ll1;                
		llf_f _u_sum_ll2;                
		sof_f _delta_r_fil;              
		fof_f     _delta_r_lag;  
		fof_f 	 _ay_fo_lag; // filter for acc Az           

		float _delta_r;                    ///< [deg] rudder deflection
		
		

		
		// inputs variables between channel;
		float _alpha_lag_A;
		float _dac; // general aileron deflection in degrees
		float _dfl;
		
		// saved for integration
		float _pitch_sum_integ;
		
		// interpolation tables
		float _pitch_cmd_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		float _pitch_trim_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		float _roll_cmd_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		float _roll_trim_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		float _yaw_cmd_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		float _yaw_trim_d1_f[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP];
		
		// usefull variables
		float _r2d;
		float _d2r;
		
		// name to be changes
		float _flaps_int;
		float _flaps_com;
		float _delta_fl;
		float _delta_fr;
		float _ailerons;
		float _flaps_te_norm;
		float _flaps_te;
		float _flaps_le_norm;
		float _flaps_le;
		
		float _delta_flc;
		float _delta_frc;
		
		// AP LONGITUDINAL MODE
		ap_long_mode_t _ap_long_mode;
		
		float _ap_va_state;
		float _ap_vz_state;
		
		// Altitude Hold
		float _ap_alt_ref; // FCU inputs
		float _ap_alt_switch;
		float _ap_alt_state;
		float _kp_alt;
		float _ki_alt;
		
		float _ap_vz_ref; // FCU inputs
		float _ap_vz_cons; // Computed per AP
		
		float _ap_theta_ref; // Computed per AP
		float _ap_theta_error; // Computed per AP
		float _ap_theta_q; // Computed per AP
		float _ap_theta_q_old; // Computed per AP
		float _ap_theta_q_ref; // Computed per AP
		float _ap_theta_q_ref_old; // Computed per AP
		
		
		// AP LATERAL
		float _ap_psi_ref; // FCU inputs
		float _ap_psi_error;
		float _ap_psi_max_lin;
		
		float _ap_phi_ref; // Computed per AP from psi fcu
		float _ap_phi_error;
		float _ap_phi_max_lin;
		
		fof_f _ele_track_fo_lag;
		fof_f _trav_track_fo_lag;
		llf_f _ele_track_ll; 
		llf_f _trav_track_ll; 
		sof_f _ele_track_so_filter;
		
		pid_f _roll_hold_pid;
		pid_f _pitch_hold_pid;
		pid_f _throttle_pid;
		
		float _ap_roll_trim;
		float _ap_pitch_trim;
		float _ap_pitch_trim_init;
		float _ap_yaw_trim;

		
		float _ap_nz_g_trim;
		
		// Engine
		float _ap_vt_ref; // FCU inputs
		float _ap_throttle_trim;
	

};




#endif // __F16_AP_F_HH__
