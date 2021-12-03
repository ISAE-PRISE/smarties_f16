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

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "f16_fcs_entity.hh"

using namespace std;

//----------------------------------------------------------------------
// Constructor 
f16_fcs_entity::f16_fcs_entity() 
{
	_id = 0;
	_is_trimmed = false;
	_is_active = false;
	_is_init = false;
	_is_localhost = false;
	
	// set structs to 0
	memset(&_hla_to_fcc_data_in, 0, sizeof(hla_to_fcc_data_hla_t));
	memset(&_fcc_data_out, 0, sizeof(efcs_only_data_hla_t));
	
	_input_mb.reset();
	_ouput_mb.reset();
	_input_mb.resize(1400);
	_ouput_mb.resize(1400);
	
	// load model tables
	_f16_efcs.load_tables();
}

//----------------------------------------------------------------------
// Destructor 
f16_fcs_entity::~f16_fcs_entity()
{
	// Nothing to do
}

//----------------------------------------------------------------------
// 
void f16_fcs_entity::init(float dt)
{
	_dt_efcs = dt;
	_f16_efcs.load_tables();
}

//----------------------------------------------------------------------
// 
void f16_fcs_entity::set_udp_bridge( std::string fcs_ip_addr
		                                       , int fcs_udp_port
		                                       , std::string bridge_ip_addr
		                                       , int bridge_udp_port)
{
	_fcs_ip_addr = fcs_ip_addr;
	_fcs_udp_port = fcs_udp_port;
	_bridge_ip_addr = bridge_ip_addr;
	_bridge_udp_port = bridge_udp_port;
	_output_link.constructSocket(fcs_ip_addr, fcs_udp_port,bridge_ip_addr, bridge_udp_port);
	_input_link.constructSocket(fcs_ip_addr, fcs_udp_port);
}

//----------------------------------------------------------------------
// Start Threads
void f16_fcs_entity::compute()
{
	FD_ZERO(&_fd_set);
	FD_SET(_input_link.getSocket(), &_fd_set);
	_max_fd = _input_link.getSocket();
	_max_fd = _max_fd + 1;
         
	 while(1)
	 {
			switch(select(_max_fd, &_fd_set, NULL, NULL, NULL)) 
			{
			case -1: 
				perror("error");
				exit(-1);	
				break;

			case 0:
				printf("timeout.\n");
				break;

			default:
				if (FD_ISSET(_input_link.getSocket(), &_fd_set)) 
				{
					_input_mb.reset();        
					_ret = _input_link.receive(static_cast<char*>(_input_mb(0)), _input_mb.maxSize());
					if (_ret > 0)
					{
						_input_mb.assumeSizeFromReservedBytes();
						_remote_udp_port =  _input_link.getLastRegisteredRemoteUdpPort();
						std::cout << "Receive from remote = " << _remote_udp_port << std::endl;
						if (_remote_udp_port == _bridge_udp_port)
						{
							_input_mb.read_bytes((char*)&_hla_to_fcc_data_in, sizeof(hla_to_fcc_data_hla_t)) ;
							if (!_is_trimmed)
							{
								_is_trimmed = true;
								_f16_efcs.set_state(_hla_to_fcc_data_in.f16_fdm_debug.x);
								_f16_efcs.set_cockpit(_hla_to_fcc_data_in.f16_cockpit);
								std::cout << "FCS COMPONENT TRIMMED" << std::endl;
							}
							else if (_is_trimmed)
							{
								clock_gettime(CLOCK_MONOTONIC, &_ts1);
								_f16_efcs.set_state(_hla_to_fcc_data_in.f16_fdm_debug.x);
								_f16_efcs.set_cockpit(_hla_to_fcc_data_in.f16_cockpit);
								_f16_efcs.ap_mode();
								_f16_efcs.ap_laws(_dt_efcs);
								_f16_efcs.fbw_lef(_dt_efcs);
								_f16_efcs.fbw_tef(_dt_efcs);
								_f16_efcs.fbw_roll(_dt_efcs);
								_f16_efcs.fbw_pitch(_dt_efcs);
								_f16_efcs.fbw_yaw(_dt_efcs);
								_fcc_data_out.f16_fcs = _f16_efcs.get_fcs_cmd();
								clock_gettime(CLOCK_MONOTONIC, &_ts2);
								if ((_ts2.tv_nsec-_ts1.tv_nsec)<0) 
								{  
									_exec_time.tv_sec = _ts2.tv_sec-_ts1.tv_sec-1;  
									_exec_time.tv_nsec = 1000000000+_ts2.tv_nsec-_ts1.tv_nsec;  
								} 
								else 
								{  
									_exec_time.tv_sec = _ts2.tv_sec-_ts1.tv_sec;  
									_exec_time.tv_nsec = _ts2.tv_nsec-_ts1.tv_nsec;  
								} 
								_wcet_ms = (float) ((_exec_time.tv_nsec) / 1000000.0);
								_fcc_data_out.fc_perf.wcet_ms = _wcet_ms;
								_ouput_mb.reset();
								_ouput_mb.write_bytes((char*)&_fcc_data_out, sizeof(efcs_only_data_hla_t));
								_ouput_mb.updateReservedBytes();
								_output_link.send(static_cast<char*>(_ouput_mb(0)), _ouput_mb.size());
							}
						}

					}
				}
				break;
		}
	}  
}


