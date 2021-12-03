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

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <fstream>
#include <stdint.h>
#include <memory>
#include <time.h>

#include <f16_fcs_entity.hh>

int main(int argc, char *argv[])
{
	if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << "<bridge_ip_addr> <bridge_udp_port> <fcs_ip_addr> <fcs_udp_port> " << std::endl;
		return 1;
    }
    int fcs_udp_port = 0;
    int bridge_udp_port = 0;
    std::string bridge_ip_addr(argv[1]);
    bridge_udp_port = atoi(argv[2]);
    std::string fcs_ip_addr(argv[3]);
    fcs_udp_port = atoi(argv[4]);
    
	f16_fcs_entity _my_fcs;
	_my_fcs.set_udp_bridge(fcs_ip_addr, fcs_udp_port, bridge_ip_addr, bridge_udp_port);
	_my_fcs.init(0.02);
	_my_fcs.compute();
	

	return 1;
}
