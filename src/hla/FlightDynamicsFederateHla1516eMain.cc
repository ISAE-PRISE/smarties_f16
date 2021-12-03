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

// System standard includes 
#include <iostream>
#include <sstream>
#include <cstdlib>

// Federate Specific includes 
#include "FlightDynamicsFederateHla1516e.hh"

int main(int argc, char *argv[])
{
	// Federation, Federate and FOM file name 
	std::string federationName =  "Smarties_F16_Federation";
	std::string federateName   =  "f16_fdm";
	std::string fomFile        =  "isae_prise_hla1516e.xml";
	
	std::wstring FederationName = FlightDynamicsFederateHla1516e::getWString(federationName.c_str());
	std::wstring FederateName = FlightDynamicsFederateHla1516e::getWString(federateName.c_str());
	std::wstring FomFile = FlightDynamicsFederateHla1516e::getWString(fomFile.c_str());
	
	float timestep_remote_eng = 0.01;
	if (argc > 1)
    {
		timestep_remote_eng = (float) atof(argv[1]);
	}

	// Create a federate object.
	// This object inherit from appropriate FederateAmbassador class
	// and embbed the appropriate RTIambassador object.
	FlightDynamicsFederateHla1516e myFederate(FederationName, FederateName, FomFile);

	// Create, Join, Publish, Subscribe and Register
	myFederate.createFederationExecution();
	myFederate.joinFederationExecution();
	myFederate.getAllHandles();
	myFederate.publishAndSubscribe();
	myFederate.registerObjectInstances();

	// HLA tm settings
	myFederate.setHlaTimeManagementSettings( 0.01 // Timestep
                                           , 0.002 // Lookahead
                                           , 0.0 // LocalTime
                                           , 400000000 // SimulationEndTime
                                           );
	myFederate.enableTimeRegulation();
	myFederate.enableTimeConstrained();
	myFederate.enableAsynchronousDelivery();
	myFederate.set_ts_dr(timestep_remote_eng);

	// Initial Sync protocol
	myFederate.pauseInitTrim();
	myFederate.initialization();
	myFederate.pauseInitSim();

	// Execution / Simulation Loop
	#ifdef DEBUG_STEP_BY_STEP
	while (myFederate.getEndOfSimulation())
	{
		myFederate.runOneStep();
		std::wcout << L">> PRESS ENTER TO GO TO NEXT SIMULATION STEP" << endl;      
        std::cin.get();
	}
	#else
	myFederate.run();
	#endif
	
	// Unpublish, unsubscribe, Resign and Destroy
	myFederate.unpublishAndUnsubscribe();
	myFederate.resignFederationExecution();
	myFederate.destroyFederationExecution();

}

