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

#ifndef __F16_FCS_ENTITY_HH__
#define __F16_FCS_ENTITY_HH__

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <fstream>
#include <stdint.h>
#include <memory>
#include <time.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <errno.h>
#include <semaphore.h>

#include <ReceivingSocketUDP.hh>
#include <SendingSocketUDP.hh>
#include <MessageBuffer.hh>
#include <hla_data_types_f.h>

// Models
#include <f16_efcs_f.hh>

class f16_fcs_entity
{
	private:
	
		// computing core
		f16_efcs_f _f16_efcs;
		float _dt_efcs;
	
		int _id;
		
		// status (booleans)
		bool _is_active;
		bool _is_trimmed;
		bool _is_init;
		bool _is_localhost;
		
		// Messages structures
		hla_to_fcc_data_hla_t _hla_to_fcc_data_in;
		efcs_only_data_hla_t _fcc_data_out;
		
		// Sockets
		SendingSocketUDP _output_link;
		ReceivingSocketUDP _input_link;
		MessageBuffer _input_mb;
		MessageBuffer _ouput_mb;
		
		// Socket receive handling with select
		fd_set _fd_set;
		int _max_fd, _ret;
		int _remote_udp_port;
		std::string _fcs_ip_addr;
		int _fcs_udp_port;
		std::string _bridge_ip_addr;
		int _bridge_udp_port;
		
		// Forrealtime measurements
		timespec _ts1, _ts2;
		timespec _exec_time;
		float _wcet_ms;

	public:
	
		f16_fcs_entity();
		~f16_fcs_entity();
		void init(float dt);
		void compute();
		void set_udp_bridge( std::string fcs_ip_addr
		                   , int fcs_udp_port
		                   , std::string bridge_ip_addr
		                   , int bridge_udp_port);
		                
			
};


#endif // __F16_FCS_ENTITY_HH__
