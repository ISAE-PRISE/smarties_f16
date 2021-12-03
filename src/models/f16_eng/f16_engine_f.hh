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

#ifndef __F16_ENGINE_F_HH__
#define __F16_ENGINE_F_HH__

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
#include "f16_engine_data_f.h"
#include "f16_data_types_f.h"

// Here all is defined in static for fpga usage
// In case we use it for fpga
#define MAX_TABLE_COL 25
#define MAX_TABLE_ROW 25

class f16_engine_f
{
	public:
		f16_engine_f();
		~f16_engine_f();
		
		void load_tables_engine_f();
		float interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val);
		float interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val);
		
		// Engines functions 
		float tgear (float throt);
		float rtau (float dp );
		float power_dot( float pw, float cpw );
		float thrust(float power, float mach, float alt_m);

	private:
		
		// Engines handling
		float _thrust_idle_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_mil_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _thrust_max_mach_alt_d1_f[MAX_TABLE_ROW][MAX_TABLE_COL];
		float _pa, _pc, _thrust;

};




#endif // __B747_AERO_F_HH__
