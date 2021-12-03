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

#ifndef __F16_ENGINE_DAEP_F_HH__
#define __F16_ENGINE_DAEP_F_HH__

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
#include <f16_engine_data_daep_f.h>
#include <f16_data_types_f.h>

// Here all is defined in static for fpga usage
// In case we use it for fpga
#define MAX_TABLE_COL 25
#define MAX_TABLE_ROW 25

class f16_engine_daep_f
{
	public:
		f16_engine_daep_f();
		~f16_engine_daep_f();
		
		// tables handling
		void load_tables_engine_f();
		float interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val);
		float interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val);
		
		// Engines functions 
		void tgear ();
		float rtau (float dp );
		void power_dot();
		void thrust_w_ab();
		void thrust_wo_ab();
		void power_ode(float dt);
		
		// set/get
		void set_mach(float mach_in);
		void set_alt(float alt_in);
		void set_dth(float dth_in);
		void set_power(float power);
		float get_thrust();
		
		
		// computation
		void dth_2_wf();
		void time_delay(float dt);

	private:
	
		//input / local / ouput variables
		float _mach_in, _alt_in, _dth_in;
		float _wf_loc, _wfab_loc;
		float _thurst_out;
		
		float _power, _cpower, _thrust;
		
		float _power_dot, _power_dot_old1, _power_dot_old2;
		
		// ode specific
		float _power_euler, _power_trap, _power_adba2, _power_adba3;
		
		// Engines handling
		float _thrust_wf_00_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_03_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_06_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_09_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_12_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_15_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_00_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_03_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_06_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_09_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_12_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_15_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_18_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_21_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_24_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_27_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_wf_18_wfab_30_mach_alt_f[MAX_TABLE_ROW][MAX_TABLE_COL];	

};

#endif // __F16_ENGINE_DAEP_F_HH__
