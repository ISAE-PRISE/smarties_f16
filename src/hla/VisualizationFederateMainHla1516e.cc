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
#include "VisualizationFederateHla1516e.hh"

int main(int argc, char *argv[])
{
	std::string federate_id;
	// This is done in order to be able to instanciate as many visu federate we want
	if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " ./VisualizationFederateHla1516e <ID>" << std::endl;
        std::cerr << "ID is set to 0" << std::endl;
        federate_id = "0";

    }
    else
    {
		federate_id    =  argv[1];
	}
	// Federation, Federate and FOM file name 
	
	std::string federationName =  "Smarties_F16_Federation";
	std::string federateName   =  "Smarties_visualization_" + federate_id;
	std::string fomFile        =  "isae_prise_hla1516e.xml";
	
	
	std::wstring FederationName = VisualizationFederateHla1516e::getWString(federationName.c_str());
	std::wstring FederateName = VisualizationFederateHla1516e::getWString(federateName.c_str());
	std::wstring FomFile = VisualizationFederateHla1516e::getWString(fomFile.c_str());

	// Create a federate object.
	// This object inherit from appropriate FederateAmbassador class
	// and embbed the appropriate RTIambassador object.
	VisualizationFederateHla1516e myFederate(FederationName, FederateName, FomFile);

	// Create, Join, Publish, Subscribe and Register
	myFederate.createFederationExecution();
	myFederate.joinFederationExecution();
	myFederate.getAllHandles();
	myFederate.publishAndSubscribe();
	myFederate.registerObjectInstances();

	// HLA tm settings
	myFederate.setHlaTimeManagementSettings( 0.02 // Timestep
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
	myFederate.run();

	// Unpublish, unsubscribe, Resign and Destroy
	myFederate.unpublishAndUnsubscribe();
	myFederate.resignFederationExecution();
	myFederate.destroyFederationExecution();

}

