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
	std::string federateName   =  "Smarties_F16_fdm";
	std::string fomFile        =  "isae_prise_hla1516e.xml";
	
	std::wstring FederationName = FlightDynamicsFederateHla1516e::getWString(federationName.c_str());
	std::wstring FederateName = FlightDynamicsFederateHla1516e::getWString(federateName.c_str());
	std::wstring FomFile = FlightDynamicsFederateHla1516e::getWString(fomFile.c_str());

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
	myFederate.setHlaTimeManagementSettings( 20.0 // Timestep
                                           , 1.0 // Lookahead
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
		std::cout << ">> PRESS ENTER TO GO TO NEXT SIMULATION STEP" << endl;      
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

