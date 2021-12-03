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
#include "CockpitFederateHla1516e.hh"
#include "CockpitFederateThreadHla1516e.hh"
#include "CockpitMainWindow.hh"

//  Link with Qt Window
CockpitMainWindow *CockpitFederateThreadHla1516e::window;
CockpitFederateThreadHla1516e::CockpitFederateThreadHla1516e(CockpitMainWindow *_window)
{    
  CockpitFederateThreadHla1516e::window =_window;
}

void CockpitFederateThreadHla1516e::run()
{

	std::wstring FederationName = L"Smarties_F16_Federation";
	std::wstring FederateName = L"cockpit_" + std::to_wstring(rand());
	std::wstring FomFile = L"isae_prise_hla1516e.xml";

	// Create a federate object.
	// This object inherit from appropriate FederateAmbassador class
	// and embbed the appropriate RTIambassador object.
	CockpitFederateHla1516e myFederate(FederationName, FederateName, FomFile);

	// Create, Join, Publish, Subscribe and Register
	myFederate.createFederationExecution();
	myFederate.joinFederationExecution();
	myFederate.getAllHandles();
	myFederate.publishAndSubscribe();
	myFederate.registerObjectInstances();
	myFederate.waitForAllObjectDiscovered();

	// HLA tm settings
	myFederate.setHlaTimeManagementSettings( 0.02 // Timestep
                                           , 0.002 // Lookahead
                                           , 0.0 // LocalTime
                                           , 400000000 // SimulationEndTime
                                           );
	myFederate.enableTimeRegulation();
	myFederate.enableTimeConstrained();
	myFederate.enableAsynchronousDelivery();
	myFederate.initHlaStructs();
	// Initial Sync protocol
	myFederate.pauseInitTrim();
	myFederate.initialization();
	myFederate.pauseInitSim();

	// Initial Sync protocol

	// Execution / Simulation Loop
	while (myFederate.getEndOfSimulation())
	{
		Q_EMIT gps_update_signal();
		Q_EMIT pfd_update_signal();
		Q_EMIT ecam_update_signal();
		myFederate.runOneStep();
	}

	// Unpublish, unsubscribe, Resign and Destroy
	myFederate.unpublishAndUnsubscribe();
	myFederate.resignFederationExecution();
	myFederate.destroyFederationExecution();

}
