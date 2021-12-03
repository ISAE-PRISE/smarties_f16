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

#ifndef __HLA_DATA_TYPES_F_HH__
#define __HLA_DATA_TYPES_F_HH__

#include <f16_data_types_f.h>
#include <perf_data_types_f.h>

#ifdef __cplusplus
extern "C" 
{
#endif

// HLA DATA TYPE
typedef struct efcs_data_hla
{ 
	f16_fcs_cmd_f_t f16_fcs_fed;
	f16_fcs_cmd_f_t f16_fcs_quad1;
	f16_fcs_cmd_f_t f16_fcs_quad2;
	f16_fcs_cmd_f_t f16_fcs_quad3;
	f16_fcs_cmd_f_t f16_fcs_quad4;
	system_perf_data_t fcs_fed_perf;
} 
efcs_data_hla_t;

typedef struct efcs_only_data_hla
{ 
	f16_fcs_cmd_f_t f16_fcs;
	system_perf_data_t fc_perf;
} 
efcs_only_data_hla_t;

typedef struct sensors_data_hla
{ 
	f16_fdm_data_f_t f16_fdm_debug;
} 
sensors_data_hla_t;

typedef struct fdm_data_hla
{ 
	f16_fdm_data_f_t f16_fdm;
	f16_fdm_data_f_t f16_fdm_euler;
	f16_fdm_data_f_t f16_fdm_trap;
	f16_fdm_data_f_t f16_fdm_adba2;
	f16_fdm_data_f_t f16_fdm_adba3;
	float extime;
} 
fdm_data_hla_t;

typedef struct hyd_act_data_hla
{ 
	f16_cs_f_t f16_hyd_act;
} 
hyd_act_data_hla_t;

typedef struct config_data_hla
{ 
	bool test1;
	bool test2;
} 
config_data_hla_t;

typedef struct sim_time_data_hla
{ 
	float ms1;
	float ms2;
} 
sim_time_data_hla_t;

typedef struct engine_data_hla
{ 
	f16_engine_data_f_t f16_eng;
} 
engine_data_hla_t;

typedef struct env_data_hla
{ 
	float temp1;
	float temp2;
} 
env_data_hla_t;

typedef struct cockpit_data_hla
{ 
	f16_cockpit_data_f_t f16_cockpit;
} 
cockpit_data_hla_t;

typedef struct hla_to_fcc_data_hla
{ 
	f16_cockpit_data_f_t f16_cockpit;
	f16_fdm_data_f_t f16_fdm_debug;
} 
hla_to_fcc_data_hla_t;


#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __HLA_DATA_TYPES_F_HH__
