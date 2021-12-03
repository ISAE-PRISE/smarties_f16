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

#include "FlightDynamicsFederateHla1516e.hh"

//----------------------------------------------------------------------
// 
FlightDynamicsFederateHla1516e::FlightDynamicsFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = _IsSyncAnnonced=  false ;
	// Note: FD is creator for the SDSE federation
	_IsCreator = true;
	_SyncRegSuccess = _SyncRegFailed = _InPause = false ;
	_IsOutMesTimespamped = true;
	_dr_cnt = 0;
	
	_DiscovObjectInstanceFullEngineModelData_daep = false;
	_DiscovObjectInstanceFullConfigurationData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;

	memset(&_eng_data_nasa_out, 0, sizeof(_eng_data_nasa_out));
	memset(&_eng_data_dr1_out, 0, sizeof(_eng_data_dr1_out));
	memset(&_eng_data_dr2_out, 0, sizeof(_eng_data_dr2_out));
	memset(&_eng_data_daep_in, 0, sizeof(_eng_data_daep_in));
	memset(&_hyd_act_data_out, 0, sizeof(_hyd_act_data_out));	
	memset(&_env_data_out, 0, sizeof(_env_data_out));
	memset(&_fdm_data_out, 0, sizeof(_fdm_data_out));	
	memset(&_sim_time_data_out, 0, sizeof(_sim_time_data_out));
	memset(&_efcs_data_in, 0, sizeof(_efcs_data_in));
	memset(&_config_data_in, 0, sizeof(_config_data_in));
	memset(&_sensors_data_out, 0, sizeof(_sensors_data_out));

	_dt_f16_fdm = 0.01;
	_ts_dr = 0.01;
    
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> Constructor(): End" << std::endl;
	#endif
	
	std::string testTag ("test tag");
	_MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	std::string testSyncTag ("");
	_MySyncTag.setData (testSyncTag.c_str (), testSyncTag.size () + 1);
	
	///
	/// 1. create the RTIambassador
	///
  std::wcout << L"=>create RTI Ambassador" << std::endl;
  
  bool test = true;  
  try
  {
	std::unique_ptr < rti1516e::RTIambassadorFactory > rtiAmbFact (new rti1516e::RTIambassadorFactory ());
    std::unique_ptr < rti1516e::RTIambassador > rtiAmbP (rtiAmbFact->createRTIambassador ());
    _RtiAmb = rtiAmbP.release ();
    std::wcout << L"* Ambassador created" << std::endl; 
  }
   
  catch (rti1516e::Exception & e)
  {
    test = false;
   std:: wcout << L"\t->createAmbassador" << std::endl;
    std::wcout << L"* Error creating ambassador" << e.what() << std::endl;  
  }

  if (test) {
      
      /* HLA Evolved requires to 'connect' to the RTI before using RTIAmb */
      try {
           _RtiAmb->connect((* this), rti1516e::HLA_EVOKED);
           std::wcout << L"* Ambassador connected" << std::endl;     
      } 
      catch (rti1516e::Exception& e) {
           std::wcout << L"RTIambassador connect caught Error " << e.what() <<std::endl;
      }
  }
}

//----------------------------------------------------------------------
// TemplateFederateHla13 Destructor
FlightDynamicsFederateHla1516e::~FlightDynamicsFederateHla1516e()
{
}

//----------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void FlightDynamicsFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb->createFederationExecution( _FederationName
										 , _FomFileName
										 );
    } 
    catch ( rti1516e::FederationExecutionAlreadyExists ) 
    {
		std::wcout << L"CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void FlightDynamicsFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb->destroyFederationExecution(_FederationName);
		}
		catch (rti1516e::Exception& e) 
		{
			std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
		}
	}
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"flightDyn"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle FlightDynamicsFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


//----------------------------------------------------------------------
// Set all Time management settings for federate
void FlightDynamicsFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void FlightDynamicsFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Published 
		std::wstring ClassFullSimulationTimeModel (L"ClassFullSimulationTimeModel");
		std::wstring AttrFullSimulationTimeModelData (L"AttrFullSimulationTimeModelData");
		_ObjectClassHandleClassFullSimulationTimeModel = _RtiAmb->getObjectClassHandle(ClassFullSimulationTimeModel);
		_AttributeHandleFullSimulationTimeModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSimulationTimeModel, AttrFullSimulationTimeModelData);
		
		std::wstring ClassFullFlightDynamicsModel (L"ClassFullFlightDynamicsModel");
		std::wstring AttrFullFlightDynamicsModelData (L"AttrFullFlightDynamicsModelData");
		_ObjectClassHandleClassFullFlightDynamicsModel = _RtiAmb->getObjectClassHandle(ClassFullFlightDynamicsModel);
		_AttributeHandleFullFlightDynamicsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullFlightDynamicsModel, AttrFullFlightDynamicsModelData);
		
		
		std::wstring ClassFullEnvironmentModel (L"ClassFullEnvironmentModel");
		std::wstring AttrFullEnvironmentModelData (L"AttrFullEnvironmentModelData");
		_ObjectClassHandleClassFullEnvironmentModel = _RtiAmb->getObjectClassHandle(ClassFullEnvironmentModel);
		_AttributeHandleFullEnvironmentModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEnvironmentModel, AttrFullEnvironmentModelData);
		
		std::wstring ClassFullHydraulicActuatorModel (L"ClassFullHydraulicActuatorModel");
		std::wstring AttrFullHydraulicActuatorModelData (L"AttrFullHydraulicActuatorModelData");
		_ObjectClassHandleClassFullHydraulicActuatorModel = _RtiAmb->getObjectClassHandle(ClassFullHydraulicActuatorModel);
		_AttributeHandleFullHydraulicActuatorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullHydraulicActuatorModel, AttrFullHydraulicActuatorModelData);
		
		std::wstring ClassFullSensorModel (L"ClassFullSensorModel");
		std::wstring AttrFullSensorModelData (L"AttrFullSensorModelData");
		_ObjectClassHandleClassFullSensorModel = _RtiAmb->getObjectClassHandle(ClassFullSensorModel);
		_AttributeHandleFullSensorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSensorModel, AttrFullSensorModelData);
		
		// Subscribed
		std::wstring ClassFullEfcsModel (L"ClassFullEfcsModel");
		std::wstring AttrFullEfcsModelData (L"AttrFullEfcsModelData");
		_ObjectClassHandleClassFullEfcsModel = _RtiAmb->getObjectClassHandle(ClassFullEfcsModel);
		_AttributeHandleFullEfcsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEfcsModel, AttrFullEfcsModelData);
		
		std::wstring ClassFullConfiguration (L"ClassFullConfiguration");
		std::wstring AttrFullConfigurationData (L"AttrFullConfigurationData");
		_ObjectClassFullConfiguration = _RtiAmb->getObjectClassHandle(ClassFullConfiguration);
		_AttributeHandleFullConfigurationData = _RtiAmb->getAttributeHandle(_ObjectClassFullConfiguration, AttrFullConfigurationData);
		
		// Published + Subscribed
		std::wstring ClassFullEngineModel (L"ClassFullEngineModel");
		std::wstring AttrFullEngineModelData (L"AttrFullEngineModelData");
		_ObjectClassHandleClassFullEngineModel = _RtiAmb->getObjectClassHandle(ClassFullEngineModel);
		_AttributeHandleFullEngineModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEngineModel, AttrFullEngineModelData);
		
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void FlightDynamicsFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes published
		_attrFullSimulationTimeModelData.insert(_AttributeHandleFullSimulationTimeModelData);      
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullSimulationTimeModel,_attrFullSimulationTimeModelData);
		_attrFullFlightDynamicsModelData.insert(_AttributeHandleFullFlightDynamicsModelData);      
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullFlightDynamicsModel,_attrFullFlightDynamicsModelData);
		_attrFullEnvironmentModelData.insert(_AttributeHandleFullEnvironmentModelData);      
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullEnvironmentModel,_attrFullEnvironmentModelData);
		_attrFullHydraulicActuatorModelData.insert(_AttributeHandleFullHydraulicActuatorModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullHydraulicActuatorModel,_attrFullHydraulicActuatorModelData);
		_attrFullSensorModelData.insert(_AttributeHandleFullSensorModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullSensorModel,_attrFullSensorModelData);  
		
		// For Class/Attributes subscribed    
		_attrFullEfcsModelData.insert(_AttributeHandleFullEfcsModelData); 
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEfcsModel,_attrFullEfcsModelData);        
		_attrFullConfigurationData.insert(_AttributeHandleFullConfigurationData);
        _RtiAmb->subscribeObjectClassAttributes(_ObjectClassFullConfiguration,_attrFullConfigurationData);
        
        // For Class/Attributes published AND subscribed
        _attrFullEngineModelData_nasa.insert(_AttributeHandleFullEngineModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullEngineModel,_attrFullEngineModelData_nasa);
		_attrFullEngineModelData_dr1.insert(_AttributeHandleFullEngineModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullEngineModel,_attrFullEngineModelData_dr1);
		_attrFullEngineModelData_dr2.insert(_AttributeHandleFullEngineModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullEngineModel,_attrFullEngineModelData_dr2);
		_attrFullEngineModelData_daep.insert(_AttributeHandleFullEngineModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEngineModel,_attrFullEngineModelData_daep);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring FullSimulationTimeModelData (L"FullSimulationTimeModelData");
        _ObjectInstanceHandleFullSimulationTimeModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullSimulationTimeModel,FullSimulationTimeModelData);
        std::wstring FullFlightDynamicsModelData (L"FullFlightDynamicsModelData");
        _ObjectInstanceHandleFullFlightDynamicsModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullFlightDynamicsModel,FullFlightDynamicsModelData);
        std::wstring FullEnvironmentModelData (L"FullEnvironmentModelData");
        _ObjectInstanceHandleFullEnvironmentModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullEnvironmentModel,FullEnvironmentModelData);
        std::wstring FullHydraulicActuatorModelData (L"FullHydraulicActuatorModelData");
        _ObjectInstanceHandleFullHydraulicActuatorModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullHydraulicActuatorModel,FullHydraulicActuatorModelData);
        std::wstring FullSensorModelData (L"FullSensorModelData");
        _ObjectInstanceHandleFullSensorModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullSensorModel,FullSensorModelData);
        
        std::wstring FullEngineModelData_nasa (L"FullEngineModelData_nasa");
        _ObjectInstanceHandleFullEngineModelData_nasa = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullEngineModel,FullEngineModelData_nasa);
        std::wstring FullEngineModelData_dr1 (L"FullEngineModelData_dr1");
        _ObjectInstanceHandleFullEngineModelData_dr1 = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullEngineModel,FullEngineModelData_dr1);
        std::wstring FullEngineModelData_dr2 (L"FullEngineModelData_dr2");
        _ObjectInstanceHandleFullEngineModelData_dr2 = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullEngineModel,FullEngineModelData_dr2);

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void FlightDynamicsFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ObjectClassFullConfiguration);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEngineModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEfcsModel);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }      
    try 
    {
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullSimulationTimeModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullFlightDynamicsModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullEnvironmentModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullEngineModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullHydraulicActuatorModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullSensorModel);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_DiscovObjectInstanceFullEngineModelData_daep ||
			//!_DiscovObjectInstanceFullConfigurationData ||
			!_DiscovObjectInstanceFullEfcsModelData)
	{
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	}
	_DiscovObjectInstanceFullEngineModelData_daep = false;
	_DiscovObjectInstanceFullConfigurationData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (//!_NewAttributeFullEngineModelData_daep ||
		   //!_NewAttributeFullConfigurationData ||
		   !_NewAttributeFullEfcsModelData)
	{
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
			
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	_NewAttributeFullEngineModelData_daep = false;
	_NewAttributeFullConfigurationData = false;
	_NewAttributeFullEfcsModelData = false;
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Callback : discover object instance
void FlightDynamicsFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_FLIGHT_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring FullEngineModelData_daep (L"FullEngineModelData_daep");
    std::wstring FullConfigurationData (L"FullConfigurationData");
    std::wstring FullEfcsModelData (L"FullEfcsModelData");

    if ( (theObjectClass == _ObjectClassHandleClassFullEngineModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEngineModelData_daep.c_str())) ) 
	{
		_DiscovObjectInstanceFullEngineModelData_daep = true;
		_ObjectInstanceHandleFullEngineModelData_daep = theObject;
		#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
		std::wcout << L"< FullEngineModelData_daep > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassFullConfiguration) &&  (!wcscmp(theObjectInstanceName.c_str(),FullConfigurationData.c_str())) ) 
	{
		_DiscovObjectInstanceFullConfigurationData = true;
		_ObjectInstanceHandleFullConfiguration = theObject;
		#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
		std::wcout << L"< FullFlightDynamicsModelData_f16 > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ObjectClassHandleClassFullEfcsModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEfcsModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullEfcsModelData = true;
		_ObjectInstanceHandleFullEfcsModelData = theObject;
		#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
		std::wcout << L"< FullFlightDynamicsModelData_f16 > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else
	{
		#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
		std::wcout << L"RAV PROBEL > Object Instance has been discovered" << std::endl;
		#endif
	}
    #ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void FlightDynamicsFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
    #endif
	std::wstring TrimString (L"Trimming");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START TRIMMING " << std::endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        std::wcout << L"Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb->registerFederationSynchronizationPoint(TrimString, _MySyncTag);
        }
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
		while (_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
			} 
			catch ( rti1516e::Exception &e ) 
			{
				std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
			} 
			catch ( ... ) 
			{
				std::wcout  << "Error: unknown non-RTI exception." << std::endl;
			}
			std::wcout << L">> Waiting for success or failure of synchronisation point init. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::wcout << L">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_IsSyncAnnonced) 
	{
		std::wcout << L">> Waiting for synchronisation point Init announcement." << std::endl;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb->synchronizationPointAchieved(TrimString);
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L">> Init Synchronisation point satisfied." << std::endl;     

	while (_InPause) 
	{
		std::wcout << L">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void FlightDynamicsFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
    #endif
	std::wstring SimString (L"Simulating");
    if (_IsCreator) 
    {
		_SyncRegSuccess = false;
		std::wcout << L">> PRESS ENTER TO START SIMULATING " << std::endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        std::wcout << L"Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb->registerFederationSynchronizationPoint(SimString, _MySyncTag);
        }
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
		while (_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
			} 
			catch ( rti1516e::Exception &e ) 
			{
				std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
			} 
			catch ( ... ) 
			{
				std::wcout  << "Error: unknown non-RTI exception." << std::endl;
			}
			std::wcout << L">> Waiting for success or failure of synchronisation point init. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::wcout << L">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::wcout << L">> Waiting for synchronisation point Init announcement." << std::endl;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb->synchronizationPointAchieved(SimString);
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L">> Init Synchronisation point satisfied." << std::endl;     

	while (_InPause) 
	{
		std::wcout << L">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Init phase for Efcs
void FlightDynamicsFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	
	float alt_trim = 5000.0; // m
	float vt_trim = 250.0 ;	 // m/s
	float psi_trim = 0.09;	 // rad
	float p_trim = 0.0;	 // rad/s
	float q_trim = 0.0;	 // rad/s
	float r_trim = 0.0;	 // rad/s
	_dt_f16_fdm = _TimeStep.getFedTime(); // set model time to fed timestep
	_f16_fdm.load_tables_aero_f();
	_f16_fdm.set_trim_conditions(vt_trim, 0.0, 0.0, alt_trim, p_trim, q_trim, r_trim, psi_trim);
	_f16_fdm.set_trim_init(20000, 0.3, -0.03 * 57.29577951, 0.0, 0.0, 0.3);
	_f16_fdm.trim_aircraft();
	_f16_fdm.compute_xdot(false, _thrust_dr1);
	_f16_fdm.compute_x(_dt_f16_fdm);
	_f16_fdm.u2fcs();
	_f16_fdm.u2cs();
	
	sendInitialHlaAttributesTrim();
    
}

//----------------------------------------------------------------------
// Calculate State values for Efcs
void FlightDynamicsFederateHla1516e::calculateState() 
{
	// dead-reckoning
	_thrust_dr1 = _thrust_dr1_p + _thrust_dr1_v * _dr_cnt * _dt_f16_fdm;
	_thrust_dr2 = _thrust_dr1_p + _thrust_dr1_v * _dr_cnt * _dt_f16_fdm + 0.5*(_thrust_dr1_a *_dr_cnt * _dt_f16_fdm* _dr_cnt * _dt_f16_fdm);
	_dr_cnt++;
	_f16_fdm.compute_xdot(false, _thrust_dr1);
}

//----------------------------------------------------------------------
// Calculate Ouput values for Efcs
void FlightDynamicsFederateHla1516e::calculateOutput() 
{
	_f16_fdm.compute_x(_dt_f16_fdm);
}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendInitialHlaAttributes()
{
	//std::wcout << L"FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;

}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendInitialHlaAttributesTrim()
{
	std::wcout << L"FlightDynamicsFederateHla1516e::sendInitialHlaAttributesTrim: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;
	rti1516e::AttributeHandleValueMap ahvpsSensorModelData;
	rti1516e::AttributeHandleValueMap ahvpsEngineModelData;

	rti1516e::VariableLengthData MyTag1;
	std::string testTag ("f16_fdm_trim");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    
	_OutputMessagebuffer.reset() ;
	_fdm_data_out.f16_fdm = _f16_fdm.get_fdm();
	_fdm_data_out.f16_fdm_euler = _f16_fdm.get_fdm_euler();
	_fdm_data_out.f16_fdm_trap = _f16_fdm.get_fdm_trap();
	_fdm_data_out.f16_fdm_adba2 = _f16_fdm.get_fdm_adba2();
	_fdm_data_out.f16_fdm_adba3 = _f16_fdm.get_fdm_adba3();
	_OutputMessagebuffer.write_bytes((char*)&_fdm_data_out, sizeof(_fdm_data_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	// fdm
	ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFlightDynamicsModelData,attrValue1));
	// sensors
	_OutputMessagebuffer.reset() ;
	_sensors_data_out.f16_fdm_debug = _f16_fdm.get_fdm();
	_OutputMessagebuffer.write_bytes((char*)&_sensors_data_out, sizeof(_sensors_data_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsSensorModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullSensorModelData,attrValue2));
	
	_OutputMessagebuffer.reset() ;
	_eng_data_nasa_out.f16_eng.thrust = _fdm_data_out.f16_fdm.x.thrust;
	_OutputMessagebuffer.write_bytes((char*)&_eng_data_nasa_out, sizeof(_eng_data_nasa_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsEngineModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullEngineModelData,attrValue3));
    
    try 
    {
		_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFlightDynamicsModelData, ahvpsFlightDynamicsModelData, MyTag1);
		_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullSensorModelData, ahvpsSensorModelData, MyTag1);
		_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_nasa, ahvpsEngineModelData, MyTag1);
    }
    catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L"FlightDynamicsFederateHla1516e::sendInitialHlaAttributes: End" << std::endl;
}


//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendHlaAttributes(RTI1516fedTime UpdateTime)
{
	std::wcout << L"FlightDynamicsFederateHla1516e::sendHlaAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;
	rti1516e::AttributeHandleValueMap ahvpsSensorModelData;
	rti1516e::AttributeHandleValueMap ahvpsEngineModelData;
	rti1516e::AttributeHandleValueMap ahvpsEngineModelDataDr1;
	rti1516e::AttributeHandleValueMap ahvpsEngineModelDataDr2;

	rti1516e::VariableLengthData MyTag1;
	std::string testTag ("f16_fdm");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);

	_OutputMessagebuffer.reset() ;
	_fdm_data_out.f16_fdm = _f16_fdm.get_fdm();
	_fdm_data_out.f16_fdm_euler = _f16_fdm.get_fdm_euler();
	_fdm_data_out.f16_fdm_trap = _f16_fdm.get_fdm_trap();
	_fdm_data_out.f16_fdm_adba2 = _f16_fdm.get_fdm_adba2();
	_fdm_data_out.f16_fdm_adba3 = _f16_fdm.get_fdm_adba3();
	_OutputMessagebuffer.write_bytes((char*)&_fdm_data_out, sizeof(_fdm_data_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	// fdm
	ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFlightDynamicsModelData,attrValue1));
	// sensors
	_OutputMessagebuffer.reset() ;
	_sensors_data_out.f16_fdm_debug = _f16_fdm.get_fdm();
	_OutputMessagebuffer.write_bytes((char*)&_sensors_data_out, sizeof(_sensors_data_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsSensorModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullSensorModelData,attrValue2));
	
	_OutputMessagebuffer.reset() ;
	_eng_data_nasa_out.f16_eng.thrust = _fdm_data_out.f16_fdm.x.thrust;
	_OutputMessagebuffer.write_bytes((char*)&_eng_data_nasa_out, sizeof(_eng_data_nasa_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsEngineModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullEngineModelData,attrValue3));
	
	_OutputMessagebuffer.reset() ;
	_eng_data_dr1_out.f16_eng.thrust = _thrust_dr1;
	_OutputMessagebuffer.write_bytes((char*)&_eng_data_dr1_out, sizeof(_eng_data_dr1_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsEngineModelDataDr1.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullEngineModelData,attrValue4));
	
	_OutputMessagebuffer.reset() ;
	_eng_data_dr2_out.f16_eng.thrust = _thrust_dr2;
	_OutputMessagebuffer.write_bytes((char*)&_eng_data_dr2_out, sizeof(_eng_data_dr2_out)) ;
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsEngineModelDataDr2.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullEngineModelData,attrValue5));
	
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFlightDynamicsModelData, ahvpsFlightDynamicsModelData, MyTag1);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullSensorModelData, ahvpsSensorModelData, MyTag1);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_nasa, ahvpsEngineModelData, MyTag1);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_dr1, ahvpsEngineModelDataDr1, MyTag1);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_dr2, ahvpsEngineModelDataDr2, MyTag1);
		}
		else
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFlightDynamicsModelData, ahvpsFlightDynamicsModelData, MyTag1, UpdateTime);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullSensorModelData, ahvpsSensorModelData, MyTag1, UpdateTime);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_nasa, ahvpsEngineModelData, MyTag1, UpdateTime);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_dr1, ahvpsEngineModelDataDr1, MyTag1, UpdateTime);
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEngineModelData_dr2, ahvpsEngineModelDataDr2, MyTag1, UpdateTime);
        }
    }
    catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L"FlightDynamicsFederateHla1516e::sendHlaAttributes: End" << std::endl;
}

//----------------------------------------------------------------------
// Callback : reflect attribute values with time
void FlightDynamicsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::LogicalTime const &theTime,
														rti1516e::OrderType receivedOrdering,
														rti1516e::MessageRetractionHandle theHandle,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
											 throw ( rti1516e::FederateInternalError) 
{
	// Same function as without Logical Time
	reflectAttributeValues(theObject, theAttributes, theUserSuppliedTag, sentOrdering, theTransport, theReflectInfo);
}

//----------------------------------------------------------------------
// Callback : reflect attribute values without time
void FlightDynamicsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
                                             throw ( rti1516e::FederateInternalError) 
{
	uint32_t valueLength ;
	rti1516e::AttributeHandle parmHandle ;
	MessageBuffer buffer;
	rti1516e::AttributeHandleValueMap::const_iterator it;
	double tmp_value;
    
    if (theObject == _ObjectInstanceHandleFullEngineModelData_daep)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			//assert(valueLength>0);
			_InputMessagebuffer.resize(valueLength);        
			_InputMessagebuffer.reset(); 
			std::memcpy(static_cast<char*>(_InputMessagebuffer(0)),(it->second).data (), valueLength);                    
			_InputMessagebuffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _AttributeHandleFullEngineModelData)  
            { 
				_InputMessagebuffer.read_bytes((char*)&_eng_data_daep_in, sizeof(_eng_data_daep_in));
				_dr_cnt = 0; // reset counter
				_thrust_dr1_p_ll = _thrust_dr1_p_l;
				_thrust_dr1_p_l  = _thrust_dr1_p;
				_thrust_dr1_p    = _eng_data_daep_in.f16_eng.thrust;
				_thrust_dr1_v_ll = _thrust_dr1_v_l;
				_thrust_dr1_v_l  = _thrust_dr1_v;
				_thrust_dr1_v    = (_thrust_dr1_p - _thrust_dr1_p_l) / _ts_dr;
				_thrust_dr1_a_ll = _thrust_dr1_a_l;
				_thrust_dr1_a_l  = _thrust_dr1_a;
				_thrust_dr1_a    = (_thrust_dr1_v - _thrust_dr1_v_l) / _ts_dr;
				
				//_f16_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEngineModelData_daep = true;
    } 
    else if (theObject == _ObjectInstanceHandleFullConfiguration)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			//assert(valueLength>0);
			_InputMessagebuffer.resize(valueLength);        
			_InputMessagebuffer.reset(); 
			std::memcpy(static_cast<char*>(_InputMessagebuffer(0)),(it->second).data (), valueLength);                    
			_InputMessagebuffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _AttributeHandleFullConfigurationData)  
            { 
				_InputMessagebuffer.read_bytes((char*)&_config_data_in, sizeof(_config_data_in));
				//_f16_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullConfigurationData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEfcsModelData)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			//assert(valueLength>0);
			_InputMessagebuffer.resize(valueLength);        
			_InputMessagebuffer.reset(); 
			std::memcpy(static_cast<char*>(_InputMessagebuffer(0)),(it->second).data (), valueLength);                    
			_InputMessagebuffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _AttributeHandleFullEfcsModelData)  
            { 
				_InputMessagebuffer.read_bytes((char*)&_efcs_data_in, sizeof(_efcs_data_in));
				_f16_fdm.set_fcs_cmd(_efcs_data_in.f16_fcs_fed);
				//_f16_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEfcsModelData = true;
    }
	else
	{ 
		std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

//----------------------------------------------------------------------
// Callback : timeRegulationEnabled
void FlightDynamicsFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::NoRequestToEnableTimeRegulationWasPending,
            rti1516e::FederateInternalError) */
{
	_IsTimeReg = true ;
} 

//----------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void FlightDynamicsFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::NoRequestToEnableTimeConstrainedWasPending,
            rti1516e::FederateInternalError) */
{
	_IsTimeConst = true ;
} 

//----------------------------------------------------------------------
// Callback : timeAdvanceGrant
void FlightDynamicsFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_FLIGHT_DYNAMICS_FEDERATE_1516E
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->timeAdvanceRequest(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->timeAdvanceRequestAvailable(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->nextMessageRequest(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

//----------------------------------------------------------------------
// function : nextEventAvailable
void FlightDynamicsFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->nextMessageRequestAvailable(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

//----------------------------------------------------------------------
// Callback : synchronizationPointRegistrationSucceeded
void FlightDynamicsFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
//----------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void FlightDynamicsFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void FlightDynamicsFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : federationSynchronized
void FlightDynamicsFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

//----------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla1516e::run()
{
	int cnt = 0;
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
		// Model calculations
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		calculateState();
		calculateOutput();
		sendHlaAttributes(updateTime);
		timeAdvanceRequest(_TimeStep);
		clock_gettime(CLOCK_MONOTONIC, &_TimeStamp);
		if ((_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec)<0) 
		{  
			_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec-1;  
			_ExecutionTime.tv_nsec = 1000000000+_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
		} 
		else 
		{  
			_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec;  
			_ExecutionTime.tv_nsec = _TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
		} 
		_TotalRealTimeMs = ((_ExecutionTime.tv_nsec) / 1000000);
		_fdm_data_out.extime = _TotalRealTimeMs;
		if (_TotalRealTimeMs < _TimeStep.getFedTime()*1000)
	{
		usleep((_TimeStep.getFedTime()*1000-_TotalRealTimeMs)*1000);
	}
	else
	{
		std::wcout << L"WARNING: _TotalRealTimeMs = " << _TotalRealTimeMs << std::endl;
	}
		cnt++;
	}
} 

//----------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla1516e::runOneStep()
{
	clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
	RTI1516fedTime updateTime(0.0);
	updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
	// Model calculations
	calculateState();
	calculateOutput();
	sendHlaAttributes(updateTime);
	timeAdvanceRequest(_TimeStep);
	clock_gettime(CLOCK_MONOTONIC, &_TimeStamp);
	if ((_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec)<0) 
	{  
		_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec-1;  
		_ExecutionTime.tv_nsec = 1000000000+_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
	} 
	else 
	{  
		_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec;  
		_ExecutionTime.tv_nsec = _TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
	} 
	_TotalRealTimeMs = ((_ExecutionTime.tv_nsec) / 1000000);
	
	if (_TotalRealTimeMs < _TimeStep.getFedTime()*1000)
	{
		usleep((_TimeStep.getFedTime()*1000-_TotalRealTimeMs)*1000);
	}
	else
	{
		std::wcout << L"WARNING: _TotalRealTimeMs = " << _TotalRealTimeMs << std::endl;
	}
} 

bool FlightDynamicsFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
