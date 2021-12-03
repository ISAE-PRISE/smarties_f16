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

#include "f16_efcs_f.hh"

//---------------------------------------------------------------------
f16_efcs_f::f16_efcs_f()
{
	memset(&_fcu_in, 0, sizeof(_fcu_in));
	memset(&_sst_in, 0, sizeof(_sst_in));
	memset(&_fcs_out, 0, sizeof(_fcs_out));
	
	// Set all tables to zeros
	// ------------- PITCH -------------------
	memset(&_pitch_cmd_d1_f, 0, sizeof(_pitch_cmd_d1_f));
	memset(&_pitch_trim_d1_f, 0, sizeof(_pitch_trim_d1_f));
	
	// ------------- ROLL -------------------
	memset(&_roll_cmd_d1_f, 0, sizeof(_roll_cmd_d1_f));
	memset(&_roll_trim_d1_f, 0, sizeof(_roll_trim_d1_f));
	
	// ------------- YAW -------------------
	memset(&_yaw_cmd_d1_f, 0, sizeof(_yaw_cmd_d1_f));
	memset(&_yaw_trim_d1_f, 0, sizeof(_yaw_trim_d1_f));
	
	// alpha lef
	_alpha_lef_lead_lag.set_c1to4(2.0,7.25,1.0,7.25);
	
	// pitch
	_stick_pitch_fo_lag.set_tc(1.0/60.0);
	_alpha_fo_lag.set_tc(1.0/10.0);
	_g_fo_lag.set_tc(1.0/8.3);
	_q_fo_lag.set_tc(1.0/50.0);
	_q_lead_lag.set_c1to4(1.0,0.0,1.0,1.0); // G(s) = s/(s+1) // c1=1, c2=0, c3=1, c4=1
	_bias1_fo_lag.set_tc(1.0/10.0);
	_az_fo_lag.set_tc(1.0/50.0);
	_step1_lead_lag.set_c1to4(3.0,12.0,1.0,12.0);
	_step2_so_filter.set_c1to6(2.0,20.0,3500.0,1.0, 40.0,3500.0);
	
	// roll
	_stick_roll_fo_lag.set_tc(1.0/60.0);
	_p_com_fo_lag.set_tc(1.0/10.0); 
	_p_com_pos_lead_lag.set_c1to4(6.0,0.0,1.0,20.0);
	_p_com_neg_lead_lag.set_c1to4(6.0,0.0,1.0,20.0);
	_p_fo_lag.set_tc(1.0/50.0);
	_p_so_filter.set_c1to6(4.0,64.0,6400.0,1.0,80.0,6400.0);
	
	// yaw
	_stick_yaw_fo_lag.set_tc(1.0/60.0);
	_omg_r_lag.set_tc( 1.0 / 50.0 );
    _omg_p_yaw.set_c1to6( 1.0, 0.0, 3025.0, 1.0, 110.0, 3025.0 );
    _u_sum_ll1.set_c1to4( 3.0, 15.0, 1.0, 15.0 );
    _u_sum_ll2.set_c1to4( 1.5,  0.0, 1.0,  1.0 );
    _delta_r_fil.set_c1to6( 1.0, 0.0, 1225.0, 1.0, 70.0, 1225.0 );
    _ay_fo_lag.set_tc(1.0/50.0);
	
	_r2d = 57.29577951;
	_d2r = 0.017453293;
	
	_dac = 0.0;
	_flaps_int = 0.0;
	_flaps_com = 0.0;
	_delta_fl = 0.0;
	_delta_fr = 0.0;
	_ailerons = 0.0;
	_flaps_te_norm = 0.0;
	_flaps_te = 0.0;
	
	_pitch_sum_integ = 0.0;
	
	
	// Autopilot longitudinal mode
	_ap_long_mode = ALT;
	_ap_alt_switch = 250.0;
	// Constants for Altitude Hold (Theta COnsigne)
	_ap_alt_state = 0;
	_kp_alt     = 0.0005;//0.0972
	_ki_alt     = 0.0004;//0.0038848
	_ap_theta_q = 0.0; // Computed per AP
	_ap_theta_q_old = 0.0; // Computed per AP
	_ap_theta_q_ref = 0.0; // Computed per AP
	_ap_theta_q_ref_old  = 0.0; // Computed per AP
	
	_ap_psi_max_lin = 5*_d2r;
	_ap_phi_max_lin = 20*_d2r;
	
	// 
	_ele_track_ll.set_c1to4(1.0,0.0,0.0,1.0);
	_trav_track_ll.set_c1to4(1.0,0.0,0.0,1.0);
	_ele_track_so_filter.set_c1to6( 0.0, 0.0, 1.0, 1.0, 2*1.0*0.6, 1.0 );
	
	// Kp, Ts, Ti, Td, Alpha, Beta, Gamma, Min, Max
	_roll_hold_pid.set_parameters(0.05,0.02,1.5,0.25,0.1,0.1,0.0125,-1.0,1.0);//< 30,000 ft
	//_roll_hold_pid.set_parameters(0.15,0.02,1.75,1.2,0.1,1.0,0.1,-1.0,1.0);
	//_pitch_hold_pid.set_parameters(-0.14,0.05,0.5,0.5,0.1,0.001,0.0125,-1.0,1.0);
	//_pitch_hold_pid.set_parameters(-0.14,0.04,0.55,0.5,0.1,0.001,0.0125,-1.0,1.0); // goOD ONE
	_pitch_hold_pid.set_parameters(-0.15,0.04,0.55,0.5,0.1,0.001,0.0125,-1.0,1.0);
	//_pitch_hold_pid.set_parameters(0.05,0.05,1.75,1.2,0.1,1.0,0.1,-1.0,1.0);
	_throttle_pid.set_parameters(0.15,0.05,1.2,0.5,0.1,0.1,0.0125,-1.0,1.0); 
	//_throttle_pid.set_parameters(0.12,0.02,1.2,0.15,0.1,0.1,0.0125,-1.0,1.0); 
	
	_is_log_enabled = false;
	_time_s = 0.0;
}

//---------------------------------------------------------------------
f16_efcs_f::~f16_efcs_f()
{

}

//---------------------------------------------------------------------
void f16_efcs_f::load_tables()
{
	int i,j;
	// ------------- PITCH -------------------
	for (i=0; i<8; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_pitch_cmd_d1_f[i][j] = pitch_cmd_d1_f[i][j];
		}
	}
	for (i=0; i<3; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_pitch_trim_d1_f[i][j] = pitch_trim_d1_f[i][j];
		}
	}
	for (i=0; i<9; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_roll_cmd_d1_f[i][j] = roll_cmd_d1_f[i][j];
		}
	}
	for (i=0; i<3; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_roll_trim_d1_f[i][j] = roll_trim_d1_f[i][j];
		}
	}
	for (i=0; i<5; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_yaw_cmd_d1_f[i][j] = yaw_cmd_d1_f[i][j];
		}
	}
	for (i=0; i<3; i++) 
	{
		for (j=0; j<2; j++) 
		{
			_yaw_trim_d1_f[i][j] = yaw_trim_d1_f[i][j];
		}
	}
	
}

//---------------------------------------------------------------------
// 
void f16_efcs_f::enable_log()
{
	_is_log_enabled = true;
}

//---------------------------------------------------------------------
// 
float f16_efcs_f::saturation(float input, float min, float max)
{
	float value = input;
	if (value > max) value = max;
	else if (value < min) value = min;
	return value;
}

//---------------------------------------------------------------------
// 
float f16_efcs_f::deadband(float start, float end, float value)
{
	if      ( value > end   ) return value - end;
	else if ( value < start ) return value - start;

	return 0.0;
}

//---------------------------------------------------------------------
// this need to be checked http://www.ida.upmc.fr/~zaleski/markers_doc/interpolation_8h-source.html
float f16_efcs_f::interp_1d_f(float table[MAX_TABLE_ROW_AP][2], int row_nb, float row_val)
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
float f16_efcs_f::interp_2d_f(float table[MAX_TABLE_ROW_AP][MAX_TABLE_COL_AP], int row_nb, int col_nb, float row_val, float col_val)
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
void f16_efcs_f::set_state(f16_x_f_t x)
{
	_x_real = x;  
	_x_fuse = x;  // state from sensor fusion it should be removed
}

//---------------------------------------------------------------------
void f16_efcs_f::set_cockpit(f16_cockpit_data_f_t cockpit_in)
{
	_fcu_in = cockpit_in.fcu;
	//_sst_in = cockpit_in.sst;  
}

//---------------------------------------------------------------------
// 
f16_fcs_cmd_f f16_efcs_f::get_fcs_cmd()
{
	return _fcs_out;
}

//---------------------------------------------------------------------
// 
void f16_efcs_f::set_fcs_cmd(f16_fcs_cmd_f fcs_cmd)
{
	_fcs_out = fcs_cmd;
	_ap_pitch_trim_init = fcs_cmd.del;
}

//---------------------------------------------------------------------
void f16_efcs_f::fbw_lef(float dt)
{
	float alpha_lef = _alpha_lef_lead_lag.compute(dt,_x_fuse.alpha*_r2d);

    // (NASA-TP-1538, p.34)
    // delta_lef = 1.38 (2s+7.25)/(s+7.25) alpha - 9.05 q/p_s + 1.45
    float flaps_le_deg = 1.38 * alpha_lef - 9.05 * _x_fuse.qbar/_x_fuse.ps + 1.45;

    _flaps_le = saturation(flaps_le_deg*_d2r, 0.0, 25.0*_d2r);
    _fcs_out.dlefr = _flaps_le;
    _fcs_out.dlefl = _flaps_le;
    _flaps_le_norm = _flaps_le / 25.0*_d2r;
	
}

//---------------------------------------------------------------------
void f16_efcs_f::fbw_tef(float dt)
{
    // (AD-A055-417, p.20)
    // float flaps_com = ( alt_flaps_ext || lg_handle_dn ) ? 20.0 : 0.0;
    float gain_f9 = 0.0;
	// (AD-A055-417, p.20 - F9)
	if ( _x_fuse.qbar/_x_fuse.ps > 1.008 )
	{
		gain_f9 = -2.0;
	}
	else if ( _x_fuse.qbar/_x_fuse.ps > 0.787 )
	{
		gain_f9 = 0.0 - 2.0 * ( _x_fuse.qbar/_x_fuse.ps - 0.787 ) / ( 1.008 - 0.787 );
	}

    float flaps_trans = saturation(gain_f9, -2.0, 2.0);
    float flaps_limit = saturation(flaps_trans - _flaps_int, -0.625, 0.625);
    _flaps_int = _flaps_int + 8.0 * dt * flaps_limit;
    _flaps_com = _flaps_int + 1.5;

    float flaperon_l = saturation(_delta_fl*_r2d,-21.5, 21.5);
    float flaperon_r = saturation(_delta_fr*_r2d,-21.5, 21.5);

    float flaps_te_r = flaperon_r - _ailerons; //real ailerons position from sensors
    float flaps_te_l = flaperon_l + _ailerons;

    _flaps_te_norm = 0.5 * ( flaps_te_r + flaps_te_l ) / 21.5;
    _flaps_te = _flaps_te_norm * 21.5*_d2r - 1.5*_d2r;
}

//---------------------------------------------------------------------
void f16_efcs_f::fbw_pitch(float dt)
{
	std::wcout << L" ***********  PITCH CHANNEL ************* " << std::endl;
	// Side stick + trim channel + ap
	float pitch_trim = 0.0; // _ap_pitch_trim; // find where does this comes from
	float pitch_ap_tie_in = 0.0; // _ap_nz_g_trim; // find where does this comes from
	std::wcout << L"pitch_ap_tie_in = " << pitch_ap_tie_in << std::endl;
	
	float sst_pitch_filt;
	sst_pitch_filt = _stick_pitch_fo_lag.compute(dt,_ap_pitch_trim);// =;_sst_in.pitch_cmd);
	std::wcout << L"sst_pitch_filt = " << sst_pitch_filt << std::endl;
	float pitch_cmd_gradient;
	pitch_cmd_gradient = interp_1d_f(_pitch_cmd_d1_f,8,_ap_pitch_trim);//sst_pitch_filt);
	std::wcout << L"sst_pitch_filt = " << sst_pitch_filt << std::endl;
	
	float pitch_trim_saturate;
	pitch_trim_saturate = interp_1d_f(_pitch_trim_d1_f,3,pitch_trim);
	std::wcout << L"pitch_trim_saturate = " << pitch_trim_saturate << std::endl;
	
	float pitch_g_sum;
	pitch_g_sum = pitch_cmd_gradient + pitch_trim_saturate;
	std::wcout << L"pitch_cmd_gradient = " << pitch_cmd_gradient << std::endl;
	
	float gmin = -4;
	float gmax = 8;
	
	// negative g limit (F1)
	if ( _x_fuse.qbar < 1628.0 ) // 34 psf
	{
		gmin = -1.0;
	}
	else if ( _x_fuse.qbar < 8810.0 ) // 184 psf
	{
		gmin = -1.0 - ( 3.0 / ( 8810.0 - 1628.0 ) ) * ( _x_fuse.qbar - 1628.0 );
	}
	
	float pitch_g_lag;
	pitch_g_lag = _g_fo_lag.compute(dt,pitch_g_sum);
	std::wcout << L"pitch_g_lag = " << pitch_g_lag << std::endl;
	
	// Alpha channel
	float alpha_deg = _x_fuse.alpha * _r2d;
	float pitch_alpha_sat;
	pitch_alpha_sat = saturation(alpha_deg, -5.0, 30.0);
	float pitch_alpha_filt;
	pitch_alpha_filt = _alpha_fo_lag.compute(dt,pitch_alpha_sat);
	std::wcout << L"pitch_alpha_filt = " << pitch_alpha_filt << std::endl;
	_alpha_lag_A = pitch_alpha_filt; // for YAW and ARI
	
	// Pitch rate i.e. Q channel
	float q_degs = _x_fuse.q * _r2d;
	float pitch_q_lag;
	pitch_q_lag = _q_fo_lag.compute(dt,q_degs);
	std::wcout << L"pitch_q_lag = " << pitch_q_lag << std::endl;
	float pitch_q_lead_lag;
	pitch_q_lead_lag = _q_lead_lag.compute(dt,pitch_q_lag);
	std::wcout << L"pitch_q_lead_lag = " << pitch_q_lead_lag << std::endl;
	
	// Az channel
	float pitch_az_lag;
	pitch_az_lag = _az_fo_lag.compute(dt,_x_fuse.nz-1.0); // Az-1.0
	std::wcout << L"pitch_az_lag = " << pitch_az_lag << std::endl;
	
	float pitch_f3_gain = 1.0;
	// gain for Q (F3)
    if ( _x_fuse.qbar > 143640.0 ) // 3,000 psf
    {
        pitch_f3_gain = 0.083;
    }
    else if ( _x_fuse.qbar > 38304.0 ) // 800 psf
    {
        pitch_f3_gain = 0.533 - ( 0.533 - 0.083 ) / ( 143640.0 - 38304.0 ) * ( _x_fuse.qbar - 38304.0 );
    }
    else if ( _x_fuse.qbar > 14364.0 ) // 300 psf
    {
        pitch_f3_gain = 1.0 - ( 1.0 - 0.533 ) / ( 38304.0 - 14364.0 ) * ( _x_fuse.qbar - 14364.0 );
    }
    
    float pitch_q_gain;
    pitch_q_gain = 0.7 * pitch_f3_gain * pitch_q_lead_lag;
    std::wcout << L"pitch_q_gain = " << pitch_q_gain << std::endl;
    float pitch_alpha_limit;
    pitch_alpha_limit = F16_AP_MAX( 0.0, 0.5*(pitch_alpha_filt-20.4+pitch_q_gain));
    std::wcout << L"pitch_alpha_limit = " << pitch_alpha_limit << std::endl;
	float pitch_bias1;
	pitch_bias1 = _bias1_fo_lag.compute(dt,6.0); // the others bias are used for Landing configuration
	std::wcout << L"pitch_bias1 = " << pitch_bias1 << std::endl;
	
	float pitch_alpha_bias = 9.0 + pitch_bias1;
	float u_sca_1 = 0.161*(pitch_q_gain+pitch_alpha_filt-pitch_alpha_bias); // find new name for him
	float u_sca_2 = 0.167*pitch_q_lead_lag + 0.5*pitch_az_lag; // touchdown not considered here (with bias = 0.231);
	float u_sca = F16_AP_MAX( 0.0, u_sca_1 ) + u_sca_2;
	std::wcout << L"u_sca_1 = " << u_sca_1 << std::endl;
	std::wcout << L"u_sca_2 = " << u_sca_2 << std::endl;
	std::wcout << L"u_sca = " << u_sca << std::endl;
	float u_sca_lag;
	u_sca_lag = _step1_lead_lag.compute(dt,u_sca);
	std::wcout << L"u_sca_lag = " << u_sca_lag << std::endl;
	float u_sca_fo_filter;
	u_sca_fo_filter = _step2_so_filter.compute(dt,u_sca_lag);
	std::wcout << L"u_sca_fo_filter = " << u_sca_fo_filter << std::endl;
	
	float pitch_g_command;
	pitch_g_command = pitch_alpha_limit - pitch_g_lag - pitch_ap_tie_in; // ap tie in not considered for now
	std::wcout << L"pitch_g_command = " << pitch_g_command << std::endl;
	float pitch_sum_gained;
	pitch_sum_gained = 3.0*pitch_f3_gain*(pitch_g_command+u_sca_fo_filter);
	std::wcout << L"pitch_sum_gained = " << pitch_sum_gained << std::endl;
	
	// float pitch_f2_gain = 0.5;  // (NASA-TP-1538, p.210)
	float pitch_f2_gain = 0.38; // (AD-A202-599, p.3-12)
	
	float pitch_alpha_sum_gained;
	pitch_alpha_sum_gained = pitch_f2_gain*pitch_alpha_filt;
	std::wcout << L"pitch_alpha_sum_gained = " << pitch_alpha_sum_gained << std::endl;
	float ka = 2000.0; // open loop amplifier gain (AD-A055-417, p.22)
	float pitch_sum_out = _pitch_sum_integ + pitch_sum_gained;
	float pitch_sum_deadband = 5.0*deadband( -25.0, 25.0, pitch_sum_out + pitch_alpha_sum_gained );
	float pitch_sum_inp = -0.5*_pitch_sum_integ - ka*deadband(-25.0, 25.0, _pitch_sum_integ);
    _pitch_sum_integ = _pitch_sum_integ + 5.0 * pitch_sum_inp * dt;
    std::wcout << L"_pitch_sum_integ = " << _pitch_sum_integ << std::endl;
    
    float sel_input = _pitch_sum_integ + pitch_sum_gained + pitch_alpha_sum_gained;
    std::wcout << L"_pitch_sum_integ = " << _pitch_sum_integ << std::endl;
    std::wcout << L"pitch_sum_gained = " << pitch_sum_gained << std::endl;
    std::wcout << L"pitch_alpha_sum_gained = " << pitch_alpha_sum_gained << std::endl;
    
    float pitch_f10_gain = 1.0; 
    float qbar_ps = _x_fuse.qbar/_x_fuse.ps; 
     // (AD-A055-417, p.20 - F10)
    if ( qbar_ps > 1.132 )
    {
        pitch_f10_gain = 0.5;
    }
    else if ( qbar_ps > 0.694 )
    {
        pitch_f10_gain = 0.25 + 0.25 * ( qbar_ps - 0.694 ) / ( 1.132 - 0.694 );
    }
    
    float differential_de = 0.5 * pitch_f10_gain * _dac;
    std::wcout << L"differential_de = " << differential_de << std::endl;
    
    _fcs_out.del = sel_input - differential_de;
    _fcs_out.der = sel_input + differential_de;
     std::wcout << L"_fcs_out.del = " << _fcs_out.del << std::endl;
     std::wcout << L"_fcs_out.der = " << _fcs_out.der << std::endl;
}

//---------------------------------------------------------------------
void f16_efcs_f::fbw_roll(float dt)
{
	std::wcout << L" ***********  ROLL CHANNEL ************* " << std::endl;
	// Side stick + trim channel + ap
	float roll_trim = 0.0; // _ap_roll_trim*_r2d; // find where does this comes from
	float roll_ap_tie_in = 0.0; // find where does this comes from
	
	std::wcout << L"roll_trim = " 
				     << roll_trim 
				     << std::endl;
	
	
	float sst_roll_filt;
	sst_roll_filt = _stick_pitch_fo_lag.compute(dt,_sst_in.roll_cmd);
	std::wcout << L"sst_roll_filt = " 
				     << sst_roll_filt 
				     << std::endl;
	
	float roll_cmd_gradient;
	roll_cmd_gradient = interp_1d_f(_roll_cmd_d1_f,9,sst_roll_filt);
	std::wcout << L"roll_cmd_gradient = "  << roll_cmd_gradient   << std::endl;
	
	float p_loop_pos = F16_AP_MAX( 0.0, _p_com_pos_lead_lag.get_y() );
    float p_loop_neg = F16_AP_MIN( 0.0, _p_com_neg_lead_lag.get_y());
    std::wcout << L"p_loop_pos = " << p_loop_pos << std::endl;
    std::wcout << L"p_loop_neg = " << p_loop_neg << std::endl;
    float roll_p_com_lag = _p_com_fo_lag.compute(dt, roll_cmd_gradient - p_loop_pos - p_loop_neg );
    std::wcout << L"roll_p_com_lag = " << roll_p_com_lag << std::endl;
    _p_com_pos_lead_lag.compute(dt, F16_AP_MAX( 0.0,roll_p_com_lag) );
    _p_com_neg_lead_lag.compute(dt, F16_AP_MIN( 0.0,roll_p_com_lag) );
    
    // Roll rate i.e. P channel
	float p_degs = _x_fuse.p * _r2d;
	float roll_p_lag;
	roll_p_lag = _p_fo_lag.compute(dt, p_degs);
	std::wcout << L"roll_p_lag = " << roll_p_lag << std::endl;
	float roll_p_so_filt;
	roll_p_so_filt = _p_so_filter.compute(dt,roll_p_lag);
	std::wcout << L"roll_p_so_filt = " << roll_p_so_filt << std::endl;
	
	_p_filtered_B = roll_p_so_filt;
	
	float roll_trim_saturate;
	roll_trim_saturate = interp_1d_f(_roll_trim_d1_f,3,roll_trim);
	std::wcout << L"roll_trim_saturate = " << roll_trim_saturate << std::endl;
	
	roll_ap_tie_in = _ap_roll_trim*_r2d;
	
	float roll_control = roll_p_so_filt - ( roll_p_com_lag + roll_trim_saturate * 1.67 ) - roll_ap_tie_in;
	std::wcout << L"roll_control = " << roll_control << std::endl;
    // WARNING GUN COMPENSATION NOT CONSIDERED + 1.67 * _gun_compensation;
    
    float aileron_com = saturation(0.12*roll_control, -21.5, 21.5); // +/-21.5 Max deflection
    std::wcout << L"roll_control = " << roll_control << std::endl;
    _fcs_out.dar = aileron_com;
    _fcs_out.dal = -aileron_com;
    
    std::wcout << L"aileron_com = " << aileron_com << std::endl;
    
    float limit_fl = F16_AP_MAX( 0.0, _delta_flc - 21.5 );
    float limit_fr = F16_AP_MAX( 0.0, _delta_frc - 21.5 );
    
     //float flaperons_max_deg = Units::rad2deg( _ailerons_max );
    _delta_flc = saturation(_flaps_com - aileron_com - limit_fr, -21.5, 21.5);
    _delta_frc = saturation(_flaps_com + aileron_com - limit_fl, -21.5, 21.5);

    _dac = _delta_frc - _delta_flc;
    
    //_fcs_out.dal = _delta_flc;
    //_fcs_out.dar = _delta_frc;
    
}

//---------------------------------------------------------------------
void f16_efcs_f::fbw_yaw(float dt)
{
	std::wcout << L" ***********  YAW CHANNEL ************* " << std::endl;
	// Roll rate i.e. P channel
	float r_degs = _x_fuse.r * _r2d;
	float yaw_r_lag;
	yaw_r_lag = _omg_r_lag.compute(dt, r_degs);
	std::wcout << L"yaw_r_lag = " << yaw_r_lag << std::endl;
	
	float yaw_trim = 0.0; //_ap_yaw_trim; // find where does this comes from
	float yaw_ap_tie_in = 0.0; // find where does this comes from
	
	float sst_yaw_filt;
	sst_yaw_filt = _stick_yaw_fo_lag.compute(dt,_sst_in.yaw_cmd);
	std::wcout << L"sst_yaw_filt = " << sst_yaw_filt << std::endl;
	
	float yaw_cmd_gradient;
	yaw_cmd_gradient = interp_1d_f(_yaw_cmd_d1_f,5,sst_yaw_filt);
	std::wcout << L"yaw_cmd_gradient = " << yaw_cmd_gradient << std::endl;
    
    _omg_p_yaw.compute(dt,_p_filtered_B);

    float r_com = _stick_yaw_fo_lag.get_y(); // + trimYaw; // trim to be considered
    std::wcout << L"r_com = " << r_com << std::endl;
    float u_sum = _omg_r_lag.get_y() - ( 1.0 / 57.3 ) * _omg_p_yaw.get_y() * _alpha_lag_A;
    std::wcout << L"u_sum = " << u_sum << std::endl;

    _u_sum_ll1.compute(dt, u_sum );
    _u_sum_ll2.compute(dt, _u_sum_ll1.get_y() );
    std::wcout << L"_u_sum_ll2 = " << _u_sum_ll2.get_y() << std::endl;

    // Aileron Rudder Interconnect (ARI)
    float ari_gain = 0.0;

    // (AD-A055-417, p.20)
    // (NASA-TP-1538, p.216)
    float alpha_abs = fabs( _alpha_lag_A );
    if ( alpha_abs < 10.0 )
    {
        ari_gain = 1.0 - alpha_abs / 10.0;
    }
    double gain_f7 = 1.0;
    float qbar_ps = _x_fuse.qbar/_x_fuse.ps; 
    // (AD-A055-417, p.20 - F7)
    if ( qbar_ps <= 0.187 )
    {
        gain_f7 = 0.0;
    }
    else if ( 0.187 <= qbar_ps && qbar_ps < 1.129 )
    {
        gain_f7 = ( qbar_ps - 0.187 ) / ( 1.129 - 0.187 );
    }
    else if ( 1.129 <= qbar_ps && qbar_ps < 1.2 )
    {
        gain_f7 = 1.0 - ( qbar_ps - 1.129 ) / ( 1.2 - 1.129 );
    }
    else
    {
        gain_f7 = 0.0;
    }
    float ari = - 0.65 * ari_gain * _alpha_lag_A * gain_f7;

	// Ay channel
	float yaw_ay_lag;
	yaw_ay_lag = _ay_fo_lag.compute(dt,_x_fuse.ny); // Az-1.0
	std::wcout << L"yaw_ay_lag = " << yaw_ay_lag << std::endl;
    float u_gy = -yaw_ay_lag; // TODO is it really -1?
    float gain_f8 = 1.0;
    if ( qbar_ps > 1.0 && qbar_ps <= 1.2 )
    {
        gain_f8 = 0.5 - 0.5 * ( qbar_ps - 1.0 ) / ( 1.2 - 1.0 );
    }
    else if ( qbar_ps > 1.0 )
    {
        gain_f8 = 0.0;
    }
    float r_auto = gain_f8 * ( _u_sum_ll2.get_y() + 19.32 * u_gy ) + 0.5 * _dac * ari;
    std::wcout << L"r_auto = " << r_auto << std::endl;

    float delta_rc = r_com + r_auto;
    
    std::wcout << L"delta_rc = " << delta_rc << std::endl;
    
    _fcs_out.dr = delta_rc;

}

//---------------------------------------------------------------------
void f16_efcs_f::ap_mode()
{
	// AP MODE
	int ap_mode_old=_ap_long_mode;
	switch (_ap_long_mode) 
	{
		case(ALT) : 
		  if ( _x_fuse.alt < _ap_alt_ref - _ap_alt_switch) _ap_long_mode = CLB; 
		  else if ( _x_fuse.alt > _ap_alt_ref + _ap_alt_switch) _ap_long_mode = DES;
		  break;
		case(CLB) :
		  if ( _x_fuse.alt > _ap_alt_ref ) _ap_long_mode = ALT; //Level-out
		  break;
		case(DES) :
		  if ( _x_fuse.alt < _ap_alt_ref ) _ap_long_mode = ALT; //Level-out
		  break;
		default:
		  _ap_long_mode = ALT;
		  break;
	}
	if (_ap_long_mode!=ap_mode_old) // Reset derivatives to prevent wierd behaviors due to changes in AP modes. 
	{ 
		_ap_alt_state = 0.0;
		_ap_va_state  = 0.0;
		_ap_vz_state  = 0.0;
	}
}

void f16_efcs_f::ap_laws(float dt)
{
	_ap_alt_ref = _fcu_in.alt_ref;
	_ap_vz_ref = _fcu_in.vs_ref;
	std::wcout << L"_ap_alt_ref = " 
				     << _ap_alt_ref 
				     << std::endl;
	//increment time
	_time_s +=dt;
	// VZ consigne
    switch(_ap_long_mode) 
    {
    	case (ALT): // ALT
	    _ap_vz_ref =  _kp_alt*(_ap_alt_ref - _x_fuse.alt) -_ki_alt*_x_fuse.vs; //Steady changes in VZ until VZ = 0
	    break;
	case 1: // CLB
		if(_ap_vz_ref <= 0)  _ap_vz_cons = 0.0; //[m/s]  _ThetaConsigne = 10.0;  
		else _ap_vz_cons = -_ap_vz_ref; //mVzCons en m/s, mAP en ft/min
	    break;
	case -1: // DES
		if(_ap_vz_ref >= 0) _ap_vz_cons = 0.0; 
		else _ap_vz_cons =  -_ap_vz_ref; //mVzCons en m/s, mAP en ft/min
	    break;

	default:
		break;
	}
	
	//_ap_vz_ref =  _kp_alt*(_ap_alt_ref - _x_fuse.alt) -_ki_alt*_x_fuse.vs;
	//float gamma_ref = _ap_vz_ref;
	//if (gamma_ref < -30.0)
    //{
		//gamma_ref = -30.0;
	//}
    //else if(gamma_ref >  30.0)
    //{
		//gamma_ref =  30.0;
	//}
	//_ap_theta_ref = gamma_ref; //_kp_alt*(_ap_alt_ref-_x_fuse.alt);// + _ki_alt*(_ap_vz_cons - _x_fuse.v); 
    //if (_ap_theta_ref < -10.0*_d2r)
    //{
		//_ap_theta_ref = -10.0*_d2r;
	//}
    //else if(_ap_theta_ref >  40.0*_d2r)
    //{
		//_ap_theta_ref =  40.0*_d2r;
	//}
	//_ap_theta_error = _ap_theta_ref - _x_fuse.theta;
	//if (_ap_theta_error < -5.0*_d2r)
    //{
		//_ap_theta_error = -5.0*_d2r;
	//}
    //else if(_ap_theta_ref >  5.0*_d2r)
    //{
		//_ap_theta_error =  5.0*_d2r;
	//}
	//_ap_theta_q_old = _ap_theta_q;
	//_ap_theta_q = _x_fuse.q;
	//_ap_theta_q_ref_old = _ap_theta_q_ref;
	//_ap_theta_q_ref = 10*(_ap_theta_error - _x_fuse.q);//((_ap_theta_q - _ap_theta_q_old)/dt));
	
	//float pid_theta = 0.0;
	//pid_theta = 0.125 * _ap_theta_q_ref + 0.0015*((_ap_theta_q_ref - _ap_theta_q_ref_old)/dt);
	//float gain_pid_theta = (11245.10407/_x_fuse.qbar)*pid_theta;
	//_fcs_out.del = gain_pid_theta ;
    //_fcs_out.der = gain_pid_theta ;
	
	_ap_theta_ref = _kp_alt*(_ap_alt_ref-_x_fuse.alt);// + _ki_alt*(_ap_vz_cons - _x_fuse.v); 
    if (_ap_theta_ref < -asin(20/_x_fuse.vt))
    {
		_ap_theta_ref = -asin(20/_x_fuse.vt);
	}
    else if(_ap_theta_ref >  asin(20/_x_fuse.vt))
    {
		_ap_theta_ref =  asin(20/_x_fuse.vt);
	}
    _ap_theta_ref += _x_fuse.alpha; 
    std::wcout << L"_ap_theta_ref (deg) = " 
				     << _ap_theta_ref * _r2d
				     << std::endl;
	// PSI CONSIGNE
	_ap_psi_ref = _fcu_in.psi_ref * _d2r; //conversion deg to rad
	_ap_psi_error = _ap_psi_ref - _x_fuse.psi;
	_ap_yaw_trim = _ap_psi_ref ;
	// handle periodicity
	float pi_value = 3.14159265358979323846;
	while (_ap_psi_error >= pi_value) _ap_psi_error-=2*pi_value;
	if (_ap_psi_error <= -pi_value) _ap_psi_error+=2*pi_value;
	while (_ap_psi_ref >= pi_value) _ap_psi_ref-=2*pi_value;
	if (_ap_psi_ref <= -pi_value) _ap_psi_ref+=2*pi_value;
	// PHI CONSIGNE
	if (_ap_psi_error > _ap_psi_max_lin) _ap_phi_ref = _ap_phi_max_lin;
	else if (_ap_psi_error < -_ap_psi_max_lin) _ap_phi_ref = -_ap_phi_max_lin;
	else _ap_phi_ref = _ap_psi_error * _ap_phi_max_lin / _ap_psi_max_lin;
	std::wcout << L"_ap_psi_ref (deg) = " 
				     << _ap_psi_ref * _r2d
				     << std::endl;
	std::wcout << L"_ap_psi_error (deg)  = " 
				     << _ap_psi_error * _r2d
				     << std::endl;
	std::wcout << L"_ap_phi_ref (deg) = " 
				     << _ap_phi_ref * _r2d
				     << std::endl;
	
	// From here this is the document fbw_doc1.pdf page 15
	// 
	// Aileron
	if     (_ap_phi_ref > 30*_d2r) 
	{
		_ap_phi_ref = 30*_d2r;
	}
    else if(_ap_phi_ref <-30*_d2r) 
    {
		_ap_phi_ref =-30*_d2r;
	}
	
    _ap_phi_error = _ap_phi_ref -_x_fuse.phi;
    if     (_ap_phi_error <-7.5*_d2r) 
    {
		_ap_phi_ref =-7.5*_d2r + _x_fuse.phi;
	}
    else if(_ap_phi_error > 7.5*_d2r) 
    {
		_ap_phi_ref = 7.5*_d2r + _x_fuse.phi;
	}
	_ap_phi_error = _ap_phi_ref -_x_fuse.phi;
	_ap_roll_trim = _ap_phi_error;
	//_ap_roll_trim = _roll_hold_pid.compute(_x_fuse.phi, _ap_phi_ref);
	
	std::wcout << L"_ap_roll_trim (deg) = " 
				     << _ap_roll_trim * _r2d
				     << std::endl;
	//_fcs_out.dal = _ap_roll_trim ;
    //_fcs_out.dar = -_ap_roll_trim ;
    
    // Elevator
	if     (_ap_theta_ref > 25*_d2r)
	{
		 _ap_theta_ref = 25*_d2r;
	}
    else if(_ap_theta_ref <-15*_d2r) 
    {
		_ap_theta_ref =-15*_d2r;
	}
	_ap_theta_error = _ap_theta_ref - _x_fuse.theta;
    std::wcout << L"_ap_theta_error (deg) = " 
				     << _ap_theta_error * _r2d
				     << std::endl;
    if     (_ap_theta_error <-20*_d2r)
    {
		_ap_theta_ref =-20*_d2r + _x_fuse.theta;
	}
    else if(_ap_theta_ref > 20*_d2r) 
    {
		_ap_theta_ref = 20*_d2r + _x_fuse.theta;
	}
	_ap_theta_error = _ap_theta_ref - _x_fuse.theta;
	
	
	_ap_pitch_trim = _pitch_hold_pid.compute(_x_fuse.theta, _ap_theta_ref);
	//_fcs_out.del = _ap_pitch_trim + _ap_pitch_trim_init;
    //_fcs_out.der = _ap_pitch_trim + _ap_pitch_trim_init;
    
    // Throttle 
    _ap_vt_ref = _fcu_in.vt_ref;
	_ap_throttle_trim = _throttle_pid.compute(_x_fuse.vt, _ap_vt_ref); //mIAS (m/s) -> fps -> kts, kts
	if ( _ap_throttle_trim > 1) 
	{
		_ap_throttle_trim = 1;
	}
	else if ( _ap_throttle_trim < 0.02 ) 
	{
		_ap_throttle_trim = 0.02; //0.1
	}
	_fcs_out.dth = _ap_throttle_trim;
	
}

//---------------------------------------------------------------------
void f16_efcs_f::open_csv(const char *reportfilename)
{
	_out_csv_file.open(reportfilename);
	std::string ite_name = "time,";
	std::string fcu_data = "fcu_vt_ref,fcu_alt_ref,fcu_heading_ref,";
	std::string ap_lev1_data = "ap_pitch_rad_ref,ap_roll_rad_ref,ap_yaw_rad_ref,";
	std::string ap_lev2_data = "ap_de_rad_cmd,ap_da_rad_rad,ap_dr_rad_cmd,ap_dth_norm_cmd,";
	std::string aircraft_state1 = "pitch_rad,roll_rad,yaw_rad,p_rads,q_rads,r_rads,alpha_rad,beta_rad,";
	std::string aircraft_state2 = "lon_deg,lat_deg,alt_m,vt_ms,vs_ms,ax_g,ay_g,az_g";
	std::string end_of_line = "\n";
	_out_csv_file << ite_name + fcu_data + ap_lev1_data + ap_lev2_data + aircraft_state1 + aircraft_state2 + end_of_line;

}

//---------------------------------------------------------------------
void f16_efcs_f::close_csv()
{
	_out_csv_file.close();
}

//---------------------------------------------------------------------
void f16_efcs_f::write_csv()
{
	float time,fcu_vt_ref,fcu_alt_ref,fcu_heading_ref;
	float ap_pitch_rad_ref,ap_roll_rad_ref,ap_yaw_rad_ref;
	float ap_de_rad_cmd,ap_da_rad_rad,ap_dr_rad_cmd,ap_dth_norm_cmd;
	float pitch_rad,roll_rad,yaw_rad,p_rads,q_rads,r_rads,alpha_rad,beta_rad;
	float lon_deg,lat_deg,alt_m,vt_ms,vs_ms,ax_g,ay_g,az_g;
	
	time = _time_s;
	// fcu
	fcu_vt_ref = _ap_vt_ref;
	fcu_alt_ref = _ap_alt_ref;
	fcu_heading_ref = _ap_psi_ref;
	// ap stage 1
	ap_pitch_rad_ref = _ap_theta_ref;
	ap_roll_rad_ref = _ap_phi_ref;
	ap_yaw_rad_ref = _ap_psi_ref;
	
	// ap stage 2
	ap_de_rad_cmd = _fcs_out.der * _d2r;
	ap_da_rad_rad =  _fcs_out.dar * _d2r;
	ap_dr_rad_cmd = _fcs_out.dr * _d2r;
	ap_dth_norm_cmd = _fcs_out.dth;
	
	// aircraft state 1
	pitch_rad = _x_fuse.theta;
	roll_rad = _x_fuse.phi;
	yaw_rad = _x_fuse.psi;
	p_rads = _x_fuse.p;
	q_rads = _x_fuse.q;
	r_rads = _x_fuse.r;
	alpha_rad = _x_fuse.alpha;
	beta_rad = _x_fuse.beta;
	
	// aircraft state 2
	lon_deg = _x_fuse.lon;
	lat_deg = _x_fuse.lat;
	alt_m = _x_fuse.alt;
	vt_ms = _x_fuse.vt;
	vs_ms = _x_fuse.vs;
	ax_g = _x_fuse.nx;
	ay_g = _x_fuse.ny;
	az_g = _x_fuse.nz;
	
	
	_out_csv_file << time << ","  << fcu_vt_ref << "," << fcu_alt_ref << "," << fcu_heading_ref << ","
	// float ap_pitch_rad_ref,ap_roll_rad_ref,ap_yaw_rad_ref;
	<< ap_pitch_rad_ref << "," << ap_roll_rad_ref << "," << ap_yaw_rad_ref << ","
	
	// float ap_de_rad_cmd,ap_da_rad_rad,ap_dr_rad_cmd,ap_dth_norm_cmd;
	<< ap_de_rad_cmd << "," << ap_da_rad_rad << "," << ap_dr_rad_cmd << "," << ap_dth_norm_cmd << ","
	
	// float pitch_rad,roll_rad,yaw_rad,p_rads,q_rads,r_rads,alpha_rad,beta_rad;
	<< pitch_rad << "," << roll_rad << "," << yaw_rad << "," << p_rads << "," << q_rads << "," << r_rads << "," << alpha_rad << "," << beta_rad << ","

	// float lon_deg,lat_deg,alt_m,vt_ms,vs_ms,ax_g,ay_g,az_g;
	<< lon_deg << "," << lat_deg << "," << alt_m << "," << vt_ms << "," << vs_ms << "," << ax_g << "," << ay_g << "," << az_g << "\n"; // END OF LINE
}
