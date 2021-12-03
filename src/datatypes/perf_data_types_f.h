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


#ifndef __PERF_DATA_TYPES_F_HH__
#define __PERF_DATA_TYPES_F_HH__

#ifdef __cplusplus
extern "C" 
{
#endif

typedef struct net_perf_data 
{ 
	uint64_t old_cnt;
	uint64_t new_cnt;
	uint64_t delta_cnt;
	uint64_t miss_cnt;
	bool faulty;
	uint64_t fault_cnt;
} 
net_perf_data_t;

typedef struct system_perf_data 
{ 
	uint64_t old_cnt;
	uint64_t old_cnt2;
	float wcet_ms;
} 
system_perf_data_t;


#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __PERF_DATA_TYPES_F_HH__
