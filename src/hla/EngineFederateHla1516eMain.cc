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
#include "EngineFederateHla1516e.hh"

int main(int argc, char *argv[])
{
	// Federation, Federate and FOM file name 
	std::string federationName =  "Smarties_F16_Federation";
	std::string federateName   =  "f16_engine_daep";
	std::string fomFile        =  "isae_prise_hla1516e.xml";
	
	std::wstring FederationName = EngineFederateHla1516e::getWString(federationName.c_str());
	std::wstring FederateName = EngineFederateHla1516e::getWString(federateName.c_str());
	std::wstring FomFile = EngineFederateHla1516e::getWString(fomFile.c_str());
	
	float time_step = 0.01;
	if (argc > 1)
    {
		time_step = (float) atof(argv[1]);
	}

	// Create a federate object.
	// This object inherit from appropriate FederateAmbassador class
	// and embbed the appropriate RTIambassador object.
	EngineFederateHla1516e myFederate(FederationName, FederateName, FomFile);

	// Create, Join, Publish, Subscribe and Register
	myFederate.createFederationExecution();
	myFederate.joinFederationExecution();
	myFederate.getAllHandles();
	myFederate.publishAndSubscribe();
	myFederate.registerObjectInstances();

	// HLA tm settings
	myFederate.setHlaTimeManagementSettings( time_step // Timestep
                                           , 0.002 // Lookahead
                                           , 0.0 // LocalTime
                                           , 400000000 // SimulationEndTime
                                           );
	myFederate.enableTimeRegulation();
	myFederate.enableTimeConstrained();
	myFederate.enableAsynchronousDelivery();

	// Initial Sync protocol
	myFederate.pauseInitTrim();
	myFederate.initialization();
	myFederate.pauseInitSim();

	// Execution / Simulation Loop
	#ifdef DEBUG_STEP_BY_STEP
	while (myFederate.getEndOfSimulation())
	{
		myFederate.runOneStep();
		std::cout << L">> PRESS ENTER TO GO TO NEXT SIMULATION STEP" << endl;      
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

