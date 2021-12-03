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

#include "f16_engine_daep_f.hh"

//---------------------------------------------------------------------
f16_engine_daep_f::f16_engine_daep_f()
{
	// ------------- Engines -------------------
	memset(&_thrust_wf_00_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_00_wfab_00_mach_alt_f));
	memset(&_thrust_wf_03_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_03_wfab_00_mach_alt_f));
	memset(&_thrust_wf_06_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_06_wfab_00_mach_alt_f));
	memset(&_thrust_wf_09_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_09_wfab_00_mach_alt_f));
	memset(&_thrust_wf_12_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_12_wfab_00_mach_alt_f));
	memset(&_thrust_wf_15_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_15_wfab_00_mach_alt_f));
	memset(&_thrust_wf_18_wfab_00_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_00_mach_alt_f));
	memset(&_thrust_wf_18_wfab_03_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_03_mach_alt_f));
	memset(&_thrust_wf_18_wfab_06_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_06_mach_alt_f));
	memset(&_thrust_wf_18_wfab_09_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_09_mach_alt_f));
	memset(&_thrust_wf_18_wfab_12_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_12_mach_alt_f));
	memset(&_thrust_wf_18_wfab_15_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_15_mach_alt_f));
	memset(&_thrust_wf_18_wfab_18_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_18_mach_alt_f));
	memset(&_thrust_wf_18_wfab_21_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_21_mach_alt_f));
	memset(&_thrust_wf_18_wfab_24_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_24_mach_alt_f));
	memset(&_thrust_wf_18_wfab_27_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_27_mach_alt_f));
	memset(&_thrust_wf_18_wfab_30_mach_alt_f, 0, sizeof(_thrust_wf_18_wfab_30_mach_alt_f));

	_mach_in = 0.0;
	_alt_in = 0.0;
	_dth_in = 0.0;
	_wf_loc = 0.0;
	_wfab_loc = 0.0;
	_thurst_out = 0.0;
	
	_power_dot = 0.0;
	_power_dot_old1 = 0.0;
	_power_dot_old2 = 0.0;
	
	_power_euler = 0.0;
	_power_trap = 0.0;
	_power_adba2 = 0.0;
	_power_adba3 = 0.0;
}

//---------------------------------------------------------------------
f16_engine_daep_f::~f16_engine_daep_f()
{

}

//---------------------------------------------------------------------
//
void f16_engine_daep_f::load_tables_engine_f()
{
	int i,j;
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_00_wfab_00_mach_alt_f[i][j] = thrust_wf_00_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_03_wfab_00_mach_alt_f[i][j] = thrust_wf_03_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_06_wfab_00_mach_alt_f[i][j] = thrust_wf_06_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_09_wfab_00_mach_alt_f[i][j] = thrust_wf_09_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_12_wfab_00_mach_alt_f[i][j] = thrust_wf_12_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_15_wfab_00_mach_alt_f[i][j] = thrust_wf_15_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_00_mach_alt_f[i][j] = thrust_wf_18_wfab_00_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_03_mach_alt_f[i][j] = thrust_wf_18_wfab_03_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_06_mach_alt_f[i][j] = thrust_wf_18_wfab_06_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_09_mach_alt_f[i][j] = thrust_wf_18_wfab_09_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_12_mach_alt_f[i][j] = thrust_wf_18_wfab_12_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_15_mach_alt_f[i][j] = thrust_wf_18_wfab_15_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_18_mach_alt_f[i][j] = thrust_wf_18_wfab_18_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_21_mach_alt_f[i][j] = thrust_wf_18_wfab_21_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_24_mach_alt_f[i][j] = thrust_wf_18_wfab_24_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_27_mach_alt_f[i][j] = thrust_wf_18_wfab_27_mach_alt_f[i][j];
		}
	}
	for (i=0; i<8; i++) 
	{
		for (j=0; j<16; j++) 
		{
			_thrust_wf_18_wfab_30_mach_alt_f[i][j] = thrust_wf_18_wfab_30_mach_alt_f[i][j];
		}
	}
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::set_mach(float mach_in)
{
	_mach_in = mach_in;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::set_alt(float alt_in)
{
	_alt_in = alt_in;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::set_dth(float dth_in)
{
	_dth_in = dth_in;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::set_power(float power)
{
	_power = power;
	_power_euler = power;
	_power_trap = power;
	_power_adba2 = power;
	_power_adba3 = power;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::dth_2_wf()
{
  
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::tgear ()
{
    //float tgear_value; 
    if (_dth_in <= 0.77)
    {
		_cpower = 64.94 * _dth_in;
    }
    else
    {
		_cpower = 217.38 * _dth_in - 117.38;
    }
    //std::wcout << L"-------> _cpower = " <<  _cpower << std::endl;
    //return _tgear;    
}

//---------------------------------------------------------------------
// 
float f16_engine_daep_f::rtau (float dp )
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
void f16_engine_daep_f::power_ode(float dt)
{
	_power_euler = _power_euler + _power_dot*dt;
	_power_trap = _power_trap + 0.5*dt*(_power_dot + _power_dot_old1);
	_power_adba2 = _power_adba2 + dt*(1.5*_power_dot - 0.5*_power_dot_old1);
	_power_adba3 = _power_adba3 + (1.0/12.0)*dt*(23.0*_power_dot - 16.0*_power_dot_old1 + 5.0*_power_dot_old2);
	// Set past values    
	_power_dot_old2 = _power_dot_old1;
	_power_dot_old1 = _power_dot;
	
	// Here default is euler
	_power = _power_euler;
	
}

//---------------------------------------------------------------------
// 
float f16_engine_daep_f::get_thrust()
{
	return _thurst_out;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::power_dot()
{

    float tpw, ta;

    if ( _cpower >= 50.0 )
    {
        if ( _power >= 50.0)
        {
                tpw = _cpower;
                ta = 5.0;
        }
        else
        {
            tpw = 60.0;
            ta = rtau ( tpw - _power );
        }
    }
    else
    {
        if ( _power >= 50.0)
        {
            tpw = 40.0;
            ta = 5.0;
        }
        else
        {
            tpw = _cpower;
            ta = rtau ( tpw - _power );
        }
    }
    _power_dot = ta * ( tpw - _power );
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::thrust_wo_ab()
{
	//float thrust_idle = 0.0, thrust_mil = 0.0, thrust_max = 0.0;
	//thrust_idle = interp_2d_f(_thrust_idle_mach_alt_d1_f,6,7,mach,alt_m);
	//thrust_mil = interp_2d_f(_thrust_mil_mach_alt_d1_f,6,7,mach,alt_m);
	//thrust_max = interp_2d_f(_thrust_max_mach_alt_d1_f,6,7,mach,alt_m);
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;
    std::wcout << L"MACH =  " <<  _mach_in << " ALT  =  " << _alt_in << " POWER  =  " << _power << std::endl; 
	if (_power >= 0 && _power < 16.666)
	{
		coef = ( _power) / 16.666;
        l = interp_2d_f(_thrust_wf_00_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_03_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_03_wfab_00_mach_alt_f " << std::endl; 
	}
	else if (_power >= 16.666 && _power < 2*16.666)
	{
		coef = ( _power - 16.666) / 16.666;
        l = interp_2d_f(_thrust_wf_03_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_06_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_06_wfab_00_mach_alt_f " << std::endl; 
	}
	else if (_power >= 2*16.666 && _power < 3*16.666)
	{
		coef = ( _power - 2*16.666) / 16.666;
        l = interp_2d_f(_thrust_wf_06_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_09_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_09_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 3*16.666 && _power < 4*16.666)
	{
		coef = ( _power - 3*16.666) / 16.666;
        l = interp_2d_f(_thrust_wf_09_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_12_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_12_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 4*16.666 && _power < 5*16.666)
	{
		coef = ( _power - 4*16.666) / 8.33;
        l = interp_2d_f(_thrust_wf_12_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_15_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_15_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 5*16.666 && _power < 6*16.666)
	{
		coef = ( _power - 5*16.666) / 16.666;
        l = interp_2d_f(_thrust_wf_15_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_00_mach_alt_f " << std::endl;
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
	_thurst_out = l + coef * ( h - l );
	//_thurst_out = 0.9*_thurst_out;
}

//---------------------------------------------------------------------
// 
void f16_engine_daep_f::thrust_w_ab()
{
	//float thrust_idle = 0.0, thrust_mil = 0.0, thrust_max = 0.0;
	//thrust_idle = interp_2d_f(_thrust_idle_mach_alt_d1_f,6,7,mach,alt_m);
	//thrust_mil = interp_2d_f(_thrust_mil_mach_alt_d1_f,6,7,mach,alt_m);
	//thrust_max = interp_2d_f(_thrust_max_mach_alt_d1_f,6,7,mach,alt_m);
	float coef = 0.0;
    float l = 0.0;
    float h = 0.0;
    std::wcout << L"MACH =  " <<  _mach_in << "ALT  =  " << _alt_in << std::endl; 
	if (_power >= 0 && _power < 8.333)
	{
		coef = ( _power) / 8.33;
        l = interp_2d_f(_thrust_wf_00_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_03_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_03_wfab_00_mach_alt_f " << std::endl; 
	}
	else if (_power >= 8.333 && _power < 2*8.333)
	{
		coef = ( _power - 8.33) / 8.33;
        l = interp_2d_f(_thrust_wf_03_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_06_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_06_wfab_00_mach_alt_f " << std::endl; 
	}
	else if (_power >= 2*8.333 && _power < 3*8.333)
	{
		coef = ( _power - 2*8.33) / 8.33;
        l = interp_2d_f(_thrust_wf_06_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_09_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_09_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 3*8.333 && _power < 4*8.333)
	{
		coef = ( _power - 3*8.33) / 8.33;
        l = interp_2d_f(_thrust_wf_09_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_12_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_12_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 4*8.333 && _power < 5*8.333)
	{
		coef = ( _power - 4*8.33) / 8.33;
        l = interp_2d_f(_thrust_wf_12_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_15_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_15_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 5*8.333 && _power < 6*8.333)
	{
		coef = ( _power - 5*8.33) / 8.33;
        l = interp_2d_f(_thrust_wf_15_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_00_mach_alt_f " << std::endl;
	}
	else if (_power >= 6*8.333 && _power < 55.0)
	{
		coef = ( _power - 6*8.33) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_00_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_03_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_03_mach_alt_f " << std::endl;
	}
	else if (_power >= 55.0 && _power < 60.0)
	{
		coef = ( _power - 55.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_03_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_06_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_06_mach_alt_f " << std::endl;
	}
	else if (_power >= 60.0 && _power < 65.0)
	{
		coef = ( _power - 60.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_06_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_09_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_09_mach_alt_f " << std::endl;
	}
	else if (_power >= 65.0 && _power < 70.0)
	{
		coef = ( _power - 65.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_09_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_12_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_12_mach_alt_f " << std::endl;
	}
	else if (_power >= 70.0 && _power < 75.0)
	{
		coef = ( _power - 70.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_12_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_15_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_15_mach_alt_f " << std::endl;
	}
	else if (_power >= 75.0 && _power < 80.0)
	{
		coef = ( _power - 75.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_15_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_18_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_18_mach_alt_f " << std::endl;
	}
	else if (_power >= 80.0 && _power < 85.0)
	{
		coef = ( _power - 80.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_18_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_21_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_21_mach_alt_f " << std::endl;
	}
	else if (_power >= 85.0 && _power < 90.0)
	{
		coef = ( _power - 85.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_21_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_24_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_24_mach_alt_f " << std::endl;
	}
	else if (_power >= 90.0 && _power < 95.0)
	{
		coef = ( _power - 90.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_24_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_27_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_27_mach_alt_f " << std::endl;
	}
	else if (_power >= 95.0 && _power < 100.0)
	{
		coef = ( _power - 95.0) / 5.0;
        l = interp_2d_f(_thrust_wf_18_wfab_27_mach_alt_f,8,16,_mach_in,_alt_in);
        h = interp_2d_f(_thrust_wf_18_wfab_30_mach_alt_f,8,16,_mach_in,_alt_in);
        std::wcout << L"f16_engine_daep_f.cc: table ==> _thrust_wf_18_wfab_30_mach_alt_f " << std::endl;
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
	_thurst_out = l + coef * ( h - l );
	_thurst_out = _thurst_out/2;
}


//---------------------------------------------------------------------
// this need to be checked http://www.ida.upmc.fr/~zaleski/markers_doc/interpolation_8h-source.html
float f16_engine_daep_f::interp_1d_f(float table[MAX_TABLE_ROW][2], int row_nb, float row_val)
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
float f16_engine_daep_f::interp_2d_f(float table[MAX_TABLE_ROW][MAX_TABLE_COL], int row_nb, int col_nb, float row_val, float col_val)
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



