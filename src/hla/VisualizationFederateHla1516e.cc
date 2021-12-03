//**********************************************************************
//**********************************************************************
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// Copyright (C) 2018-2021  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email: jean-baptiste.chaudron@isae.fr
//
//**********************************************************************
//**********************************************************************

#include "VisualizationFederateHla1516e.hh"

//----------------------------------------------------------------------
// 
VisualizationFederateHla1516e::VisualizationFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_IsCreator = false;
	_SyncRegSuccess = _SyncRegFailed = _InPause = false ;
	_IsOutMesTimespamped = true;
	
	_DiscovObjectInstanceFullSimulationTimeModelData = false;
	_NewAttributeFullSimulationTimeModelData = false;
	_DiscovObjectInstanceFullFlightDynamicsModelData = false;
	_NewAttributeFullFlightDynamicsModelData = false;
	_DiscovObjectInstanceFullEngineModelData_nasa = false;
	_NewAttributeFullEngineModelData_nasa = false;
	_DiscovObjectInstanceFullHydraulicActuatorModelData = false;
	_NewAttributeFullHydraulicActuatorModelData = false;
	_DiscovObjectInstanceFullSensorModelData = false;
	_NewAttributeFullSensorModelData = false;
	_DiscovObjectInstanceFullCockpitModelData = false;
	_NewAttributeFullCockpitModelData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;
	_NewAttributeFullEfcsModelData = false;
	_DiscovObjectInstanceFullEfcsComponentPerformanceData = false;
	_NewAttributeFullEfcsComponentPerformanceData = false;
	_DiscovObjectInstanceFullExternalAircraftModelData = false;
	_NewAttributeFullExternalAircraftModelData = false;
	_DiscovObjectInstanceFullEngineModelData_dr1 = false;
	_DiscovObjectInstanceFullEngineModelData_dr2 = false;
	_DiscovObjectInstanceFullEngineModelData_daep = false;
	_NewAttributeFullEngineModelData_dr1 = false;
	_NewAttributeFullEngineModelData_dr2 = false;
	_NewAttributeFullEngineModelData_daep = false;
	
	memset(&_eng_data_nasa_in, 0, sizeof(_eng_data_nasa_in));
	memset(&_eng_data_dr1_in, 0, sizeof(_eng_data_dr1_in));
	memset(&_eng_data_dr2_in, 0, sizeof(_eng_data_dr2_in));
	memset(&_eng_data_daep_in, 0, sizeof(_eng_data_daep_in));
	memset(&_hyd_act_data_in, 0, sizeof(_hyd_act_data_in));	
	memset(&_env_data_in, 0, sizeof(_env_data_in));
	memset(&_fdm_data_in, 0, sizeof(_fdm_data_in));	
	memset(&_sim_time_data_in, 0, sizeof(_sim_time_data_in));
	memset(&_efcs_data_in, 0, sizeof(_efcs_data_in));
	memset(&_config_data_in, 0, sizeof(_config_data_in));
	memset(&_sensors_data_in, 0, sizeof(_sensors_data_in));
	memset(&_cockpit_data_in, 0, sizeof(_cockpit_data_in));
	
    
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
VisualizationFederateHla1516e::~VisualizationFederateHla1516e()
{
}

//----------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void VisualizationFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void VisualizationFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L"VisualizationFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle VisualizationFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


//----------------------------------------------------------------------
// Set all Time management settings for federate
void VisualizationFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void VisualizationFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
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
		
		std::wstring ClassFullEngineModel (L"ClassFullEngineModel");
		std::wstring AttrFullEngineModelData (L"AttrFullEngineModelData");
		_ObjectClassHandleClassFullEngineModel = _RtiAmb->getObjectClassHandle(ClassFullEngineModel);
		_AttributeHandleFullEngineModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEngineModel, AttrFullEngineModelData);
		
		std::wstring ClassFullHydraulicActuatorModel (L"ClassFullHydraulicActuatorModel");
		std::wstring AttrFullHydraulicActuatorModelData (L"AttrFullHydraulicActuatorModelData");
		_ObjectClassHandleClassFullHydraulicActuatorModel = _RtiAmb->getObjectClassHandle(ClassFullHydraulicActuatorModel);
		_AttributeHandleFullHydraulicActuatorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullHydraulicActuatorModel, AttrFullHydraulicActuatorModelData);
		
		std::wstring ClassFullSensorModel (L"ClassFullSensorModel");
		std::wstring AttrFullSensorModelData (L"AttrFullSensorModelData");
		_ObjectClassHandleClassFullSensorModel = _RtiAmb->getObjectClassHandle(ClassFullSensorModel);
		_AttributeHandleFullSensorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSensorModel, AttrFullSensorModelData);
		
		std::wstring ClassFullCockpitModel (L"ClassFullCockpitModel");
		std::wstring AttrFullCockpitModelData (L"AttrFullCockpitModelData");
		_ObjectClassHandleClassFullCockpitModel = _RtiAmb->getObjectClassHandle(ClassFullCockpitModel);
		_AttributeHandleFullCockpitModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullCockpitModel, AttrFullCockpitModelData);
		
		std::wstring ClassFullEfcsModel (L"ClassFullEfcsModel");
		std::wstring AttrFullEfcsModelData (L"AttrFullEfcsModelData");
		_ObjectClassHandleClassFullEfcsModel = _RtiAmb->getObjectClassHandle(ClassFullEfcsModel);
		_AttributeHandleFullEfcsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEfcsModel, AttrFullEfcsModelData);
		
		std::wstring ClassFullExternalAircraftModel (L"ClassFullExternalAircraftModel");
		std::wstring AttrFullExternalAircraftModelData (L"AttrFullExternalAircraftModelData");
		_ObjectClassHandleClassFullExternalAircraftModel = _RtiAmb->getObjectClassHandle(ClassFullExternalAircraftModel);
		_AttributeHandleFullExternalAircraftModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullExternalAircraftModel, AttrFullExternalAircraftModelData);
		
		std::wstring ClassFullEfcsComponentPerformance (L"ClassFullEfcsComponentPerformance");
		std::wstring AttrFullEfcsComponentPerformanceData (L"AttrFullEfcsComponentPerformanceData");
		_ObjectClassHandleClassFullEfcsComponentPerformance = _RtiAmb->getObjectClassHandle(ClassFullEfcsComponentPerformance);
		_AttributeHandleFullEfcsComponentPerformanceData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEfcsComponentPerformance, AttrFullEfcsComponentPerformanceData);
		
		// Published
		std::wstring ClassFullConfiguration (L"ClassFullConfiguration");
		std::wstring AttrFullConfigurationData (L"AttrFullConfigurationData");
		_ObjectClassFullConfiguration = _RtiAmb->getObjectClassHandle(ClassFullConfiguration);
		_AttributeHandleFullConfigurationData = _RtiAmb->getAttributeHandle(_ObjectClassFullConfiguration, AttrFullConfigurationData);
		
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void VisualizationFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		_attrFullSimulationTimeModelData.insert(_AttributeHandleFullSimulationTimeModelData);      
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSimulationTimeModel,_attrFullSimulationTimeModelData);
		_attrFullFlightDynamicsModelData.insert(_AttributeHandleFullFlightDynamicsModelData);      
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullFlightDynamicsModel,_attrFullFlightDynamicsModelData);
		_attrFullEnvironmentModelData.insert(_AttributeHandleFullEnvironmentModelData);      
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEnvironmentModel,_attrFullEnvironmentModelData);
		_attrFullEngineModelData_nasa.insert(_AttributeHandleFullEngineModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEngineModel,_attrFullEngineModelData_nasa);
		_attrFullHydraulicActuatorModelData.insert(_AttributeHandleFullHydraulicActuatorModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullHydraulicActuatorModel,_attrFullHydraulicActuatorModelData);
		_attrFullSensorModelData.insert(_AttributeHandleFullSensorModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSensorModel,_attrFullSensorModelData);
		_attrFullCockpitModelData.insert(_AttributeHandleFullCockpitModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullCockpitModel,_attrFullCockpitModelData);
		_attrFullEfcsModelData.insert(_AttributeHandleFullEfcsModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEfcsModel,_attrFullEfcsModelData);
		_attrFullExternalAircraftModelData.insert(_AttributeHandleFullExternalAircraftModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullExternalAircraftModel,_attrFullExternalAircraftModelData);
		_attrFullEfcsComponentPerformanceData.insert(_AttributeHandleFullEfcsComponentPerformanceData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEfcsComponentPerformance,_attrFullEfcsComponentPerformanceData);
		// For Class/Attributes published
		_attrFullConfigurationData.insert(_AttributeHandleFullConfigurationData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassFullConfiguration,_attrFullConfigurationData);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring FullConfigurationData (L"FullConfigurationData");
        _ObjectInstanceHandleFullConfiguration = _RtiAmb->registerObjectInstance(_ObjectClassFullConfiguration,FullConfigurationData);

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void VisualizationFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullSimulationTimeModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullFlightDynamicsModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEnvironmentModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEngineModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullHydraulicActuatorModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullSensorModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullCockpitModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEfcsModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullExternalAircraftModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEfcsComponentPerformance);
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
        _RtiAmb->unpublishObjectClass(_ObjectClassFullConfiguration);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( //!_DiscovObjectInstanceFullSimulationTimeModelData ||
			!_DiscovObjectInstanceFullFlightDynamicsModelData ||
			//!_DiscovObjectInstanceFullEnvironmentModelData ||
			//!_DiscovObjectInstanceFullEngineModelData_nasa ||
			//!_DiscovObjectInstanceFullEngineModelData_dr1 ||
			//!_DiscovObjectInstanceFullEngineModelData_dr2 ||
			!_DiscovObjectInstanceFullEngineModelData_daep ||
			!_DiscovObjectInstanceFullCockpitModelData ||
			//!_DiscovObjectInstanceFullHydraulicActuatorModelData ||
			!_DiscovObjectInstanceFullSensorModelData ||
			!_DiscovObjectInstanceFullEfcsModelData 
			//!_DiscovObjectInstanceFullExternalAircraftModelData ||
			//!_DiscovObjectInstanceFullEfcsComponentPerformanceData
			)
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
	_DiscovObjectInstanceFullSimulationTimeModelData = false;
	_DiscovObjectInstanceFullFlightDynamicsModelData = false;
	_DiscovObjectInstanceFullEnvironmentModelData = false;
	_DiscovObjectInstanceFullEngineModelData_nasa = false;
	_DiscovObjectInstanceFullEngineModelData_dr1 = false;
	_DiscovObjectInstanceFullEngineModelData_dr2 = false;
	_DiscovObjectInstanceFullEngineModelData_daep = false;
	_DiscovObjectInstanceFullHydraulicActuatorModelData = false;
	_DiscovObjectInstanceFullSensorModelData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;
	_DiscovObjectInstanceFullExternalAircraftModelData = false;
	_DiscovObjectInstanceFullEfcsComponentPerformanceData = false;
	_DiscovObjectInstanceFullCockpitModelData = false;

	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (//!_NewAttributeFullSimulationTimeModelData ||
		   !_NewAttributeFullFlightDynamicsModelData ||
		   //!_NewAttributeFullEnvironmentModelData ||
		   //!_NewAttributeFullEngineModelData_nasa ||
		   //!_NewAttributeFullEngineModelData_dr1 ||
		   //!_NewAttributeFullEngineModelData_dr2 ||
		   !_NewAttributeFullEngineModelData_daep ||
		   //!_NewAttributeFullHydraulicActuatorModelData ||
		   !_NewAttributeFullSensorModelData ||
		   !_NewAttributeFullCockpitModelData ||
		   !_NewAttributeFullEfcsModelData 
		   //!_NewAttributeFullExternalAircraftModelData ||
		   //!_NewAttributeFullEfcsComponentPerformanceData
		   )
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
	_NewAttributeFullSimulationTimeModelData = false;
	_NewAttributeFullFlightDynamicsModelData = false;
	_NewAttributeFullEnvironmentModelData = false;
	_NewAttributeFullEngineModelData_nasa = false;
	_NewAttributeFullEngineModelData_dr1 = false;
	_NewAttributeFullEngineModelData_dr2 = false;
	_NewAttributeFullEngineModelData_daep = false;
	_NewAttributeFullHydraulicActuatorModelData = false;
	_NewAttributeFullSensorModelData = false;
	_NewAttributeFullCockpitModelData = false;
	_NewAttributeFullEfcsModelData = false;
	_NewAttributeFullExternalAircraftModelData = false;
	_NewAttributeFullEfcsComponentPerformanceData = false;
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Callback : discover object instance
void VisualizationFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_FLIGHT_DYNAMICS_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring SimulationTimeModelData (L"SimulationTimeModelData");
    std::wstring FullFlightDynamicsModelData (L"FullFlightDynamicsModelData");
    std::wstring FullEnvironmentModelData (L"FullEnvironmentModelData");
    std::wstring FullEngineModelData_nasa (L"FullEngineModelData_nasa");
    std::wstring FullEngineModelData_dr1 (L"FullEngineModelData_dr1");
    std::wstring FullEngineModelData_dr2 (L"FullEngineModelData_dr2");
    std::wstring FullEngineModelData_daep (L"FullEngineModelData_daep");
    std::wstring FullHydraulicActuatorModelData (L"FullHydraulicActuatorModelData");
    std::wstring FullSensorModelData (L"FullSensorModelData");
    std::wstring CockpitModelData (L"CockpitModelData");
    std::wstring FullEfcsModelData (L"FullEfcsModelData");
    std::wstring ExternalAircraftModelData (L"ExternalAircraftModelData");
    std::wstring EfcsComponentPerformanceData (L"EfcsComponentPerformanceData");
    if ( (theObjectClass == _ObjectClassHandleClassFullSimulationTimeModel) &&  (!wcscmp(theObjectInstanceName.c_str(),SimulationTimeModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullSimulationTimeModelData = true;
		_ObjectInstanceHandleFullSimulationTimeModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< SimulationTimeModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullFlightDynamicsModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullFlightDynamicsModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullFlightDynamicsModelData = true;
		_ObjectInstanceHandleFullFlightDynamicsModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullFlightDynamicsModelData > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ObjectClassHandleClassFullEnvironmentModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEnvironmentModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullEnvironmentModelData = true;
		_ObjectInstanceHandleFullEnvironmentModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullEnvironmentModelData > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ObjectClassHandleClassFullEngineModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEngineModelData_nasa.c_str())) ) 
	{
		_DiscovObjectInstanceFullEngineModelData_nasa = true;
		_ObjectInstanceHandleFullEngineModelData_nasa = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullEngineModelData_nasa > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullEngineModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEngineModelData_dr1.c_str())) ) 
	{
		_DiscovObjectInstanceFullEngineModelData_dr1 = true;
		_ObjectInstanceHandleFullEngineModelData_dr1 = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullEngineModelData_dr1 > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullEngineModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEngineModelData_dr2.c_str())) ) 
	{
		_DiscovObjectInstanceFullEngineModelData_dr2 = true;
		_ObjectInstanceHandleFullEngineModelData_dr2 = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullEngineModelData_dr2 > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullEngineModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEngineModelData_daep.c_str())) ) 
	{
		_DiscovObjectInstanceFullEngineModelData_daep = true;
		_ObjectInstanceHandleFullEngineModelData_daep = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullEngineModelData_dr2 > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullHydraulicActuatorModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullHydraulicActuatorModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullHydraulicActuatorModelData = true;
		_ObjectInstanceHandleFullHydraulicActuatorModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullHydraulicActuatorModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullSensorModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullSensorModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullSensorModelData = true;
		_ObjectInstanceHandleFullSensorModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< FullSensorModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullCockpitModel) &&  (!wcscmp(theObjectInstanceName.c_str(),CockpitModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullCockpitModelData = true;
		_ObjectInstanceHandleFullCockpitModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< CockpitModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullEfcsModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEfcsModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullEfcsModelData = true;
		_ObjectInstanceHandleFullEfcsModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< EfcsModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullExternalAircraftModel) &&  (!wcscmp(theObjectInstanceName.c_str(),ExternalAircraftModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullExternalAircraftModelData = true;
		_ObjectInstanceHandleFullExternalAircraftModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< ExternalAircraftModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullEfcsComponentPerformance) &&  (!wcscmp(theObjectInstanceName.c_str(),EfcsComponentPerformanceData.c_str())) ) 
	{
		_DiscovObjectInstanceFullEfcsComponentPerformanceData = true;
		_ObjectInstanceHandleFullEfcsComponentPerformanceData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< EfcsComponentPerformanceData > Object Instance has been discovered" << std::endl;
		#endif
	}
    #ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void VisualizationFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void VisualizationFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
    std::wcout << L"VisualizationFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Init phase for Efcs
void VisualizationFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
}

//----------------------------------------------------------------------
// Calculate State values for Efcs
void VisualizationFederateHla1516e::calculateState() 
{

}

//----------------------------------------------------------------------
// Calculate Ouput values for Efcs
void VisualizationFederateHla1516e::calculateOutput() 
{
	// Model output calculation
	MyFlightGearNativeFdm.FlighGearSocketSend();
}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void VisualizationFederateHla1516e::sendInitialHlaAttributesTrim()
{
	//std::wcout << L"FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;

}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void VisualizationFederateHla1516e::sendInitialHlaAttributes()
{
	//std::wcout << L"FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	//rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;

    //rti1516e::VariableLengthData MyTag1;
	//std::string testTag ("Aircraft_F16-Trim");
	//MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    
    //_OutputMessagebuffer.reset() ;
    //_fdm_out = _fdm_entity.get_fdm();
    //_OutputMessagebuffer.write_bytes((char*)&_fdm_out, sizeof(_fdm_out)) ;
    //_OutputMessagebuffer.updateReservedBytes();
    
    //rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	//ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFdmModelData,attrValue1)); 
    
    try 
    {
		//_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1);
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


//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void VisualizationFederateHla1516e::sendHlaAttributes(RTI1516fedTime UpdateTime)
{
	//std::wcout << L"FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	//rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;

    //rti1516e::VariableLengthData MyTag1;
	//std::string testTag ("Aircraft_F16");
	//MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    
    //_OutputMessagebuffer.reset() ;
    //_fdm_out = _fdm_entity.get_fdm();
    //_OutputMessagebuffer.write_bytes((char*)&_fdm_out, sizeof(_fdm_out)) ;
    //_OutputMessagebuffer.updateReservedBytes();
    
    //rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	//ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFdmModelData,attrValue1)); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			//_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1);
		}
		else
        {
			//_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1, UpdateTime);
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
}

//----------------------------------------------------------------------
// Callback : reflect attribute values with time
void VisualizationFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void VisualizationFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
    
    if (theObject == _ObjectInstanceHandleFullSimulationTimeModelData)
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
            if      (parmHandle == _AttributeHandleFullSimulationTimeModelData)  
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _sim_time_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_sim_time_data_in, sizeof(_sim_time_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullSimulationTimeModelData = true;
    } 
    else if (theObject == _ObjectInstanceHandleFullFlightDynamicsModelData)
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
            if      (parmHandle == _AttributeHandleFullFlightDynamicsModelData)  
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _fdm_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_fdm_data_in, sizeof(_fdm_data_in));
				MyFlightGearNativeFdm.setRightAileron(_fdm_data_in.f16_fdm.cs.dar);
				MyFlightGearNativeFdm.setLeftAileron(_fdm_data_in.f16_fdm.cs.dal);
				MyFlightGearNativeFdm.setElevator(_fdm_data_in.f16_fdm.cs.der);
				MyFlightGearNativeFdm.setRudder(_fdm_data_in.f16_fdm.cs.del);
				MyFlightGearNativeFdm.setFlaps(_fdm_data_in.f16_fdm.cs.dr);
				//yFlightGearNativeFdm.setSpoilers(_FcsToSimMessage.cmd_left_flap);
				//MyFlightGearNativeFdm.setGears(_FcsToSimMessage.cmd_gears);
				MyFlightGearNativeFdm.setEngineThrottle(_fdm_data_in.f16_fdm.fcs.dth);
				MyFlightGearNativeFdm.setLongitude(_fdm_data_in.f16_fdm.x.lon * 57.29577951);
				MyFlightGearNativeFdm.setLatitude(_fdm_data_in.f16_fdm.x.lat * 57.29577951);
				MyFlightGearNativeFdm.setAltitude(_fdm_data_in.f16_fdm.x.alt);
				MyFlightGearNativeFdm.setPhi(_fdm_data_in.f16_fdm.x.phi);
				MyFlightGearNativeFdm.setTheta(_fdm_data_in.f16_fdm.x.theta);
				MyFlightGearNativeFdm.setPsi(_fdm_data_in.f16_fdm.x.psi);
				MyFlightGearNativeFdm.setUspeed(_fdm_data_in.f16_fdm.x.u);
				MyFlightGearNativeFdm.setVspeed(_fdm_data_in.f16_fdm.x.v);
				MyFlightGearNativeFdm.setWspeed(_fdm_data_in.f16_fdm.x.w);
				MyFlightGearNativeFdm.setXacc(_fdm_data_in.f16_fdm.x.nx);
				MyFlightGearNativeFdm.setYacc(_fdm_data_in.f16_fdm.x.ny);
				MyFlightGearNativeFdm.setZacc(_fdm_data_in.f16_fdm.x.nz);
				MyFlightGearNativeFdm.setVcas(_fdm_data_in.f16_fdm.x.vt);
				MyFlightGearNativeFdm.setAlpha(_fdm_data_in.f16_fdm.x.alpha);
				MyFlightGearNativeFdm.setAlpha(_fdm_data_in.f16_fdm.x.beta);
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullFlightDynamicsModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEnvironmentModelData)
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
            if      (parmHandle == _AttributeHandleFullEnvironmentModelData)  
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _env_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_env_data_in, sizeof(_env_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEnvironmentModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEngineModelData_nasa)
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
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _eng_data_nasa_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_eng_data_nasa_in, sizeof(_eng_data_nasa_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEngineModelData_nasa = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEngineModelData_dr1)
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
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _eng_data_dr1_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_eng_data_dr1_in, sizeof(_eng_data_dr1_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEngineModelData_dr1 = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEngineModelData_dr2)
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
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _eng_data_dr2_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_eng_data_dr2_in, sizeof(_eng_data_dr2_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEngineModelData_dr2 = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEngineModelData_daep)
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
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _eng_data_daep_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_eng_data_daep_in, sizeof(_eng_data_daep_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEngineModelData_daep = true;
    }
    else if (theObject == _ObjectInstanceHandleFullHydraulicActuatorModelData)
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
            if      (parmHandle == _AttributeHandleFullHydraulicActuatorModelData)  
            { 
				//_InputMessagebuffer.read_bytes((char*)&_cockpit_in, sizeof(_cockpit_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullHydraulicActuatorModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullSensorModelData)
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
            if      (parmHandle == _AttributeHandleFullSensorModelData)  
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _sensors_data_in " << std::endl;
				_InputMessagebuffer.read_bytes((char*)&_sensors_data_in, sizeof(_sensors_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullSensorModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullCockpitModelData)
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
            if      (parmHandle == _AttributeHandleFullCockpitModelData)  
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _cockpit_data_in " << std::endl;
				_InputMessagebuffer.read_bytes((char*)&_cockpit_data_in, sizeof(_cockpit_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullCockpitModelData = true;
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
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> _efcs_data_in " << std::endl;
				//_InputMessagebuffer.read_bytes((char*)&_cockpit_in, sizeof(_cockpit_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEfcsModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullExternalAircraftModelData)
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
            if      (parmHandle == _AttributeHandleFullExternalAircraftModelData)  
            { 
				//_InputMessagebuffer.read_bytes((char*)&_cockpit_in, sizeof(_cockpit_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullExternalAircraftModelData = true;
    }
    else if (theObject == _ObjectInstanceHandleFullEfcsComponentPerformanceData)
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
            if      (parmHandle == _AttributeHandleFullEfcsComponentPerformanceData)  
            { 
				//_InputMessagebuffer.read_bytes((char*)&_cockpit_in, sizeof(_cockpit_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEfcsComponentPerformanceData = true;
    }
	else
	{ 
		std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

//----------------------------------------------------------------------
// Callback : timeRegulationEnabled
void VisualizationFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void VisualizationFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void VisualizationFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_MANAGER_FEDERATE_1516E
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

//----------------------------------------------------------------------
void VisualizationFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
//----------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void VisualizationFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void VisualizationFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : federationSynchronized
void VisualizationFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

//----------------------------------------------------------------------
// function : run
void VisualizationFederateHla1516e::run()
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
		std::wcout << L"_TotalRealTimeMs = " 
				     << _TotalRealTimeMs
				     << std::endl;
		if (_TotalRealTimeMs < 20.0)
		{
			usleep((_TimeStep.getFedTime()*1000-_TotalRealTimeMs)*1000);
		}
		cnt++;
	}
} 

//----------------------------------------------------------------------
// function : run
void VisualizationFederateHla1516e::runOneStep()
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
	std::wcout << L"_TotalRealTimeMs = " 
				     << _TotalRealTimeMs
				     << std::endl;
	if (_TotalRealTimeMs < 20.0)
	{
		usleep((_TimeStep.getFedTime()*1000-_TotalRealTimeMs)*1000);
	}
} 

bool VisualizationFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
