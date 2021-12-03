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

#include "EfcsBridgeFederateHla1516e.hh"

//----------------------------------------------------------------------
// 
EfcsBridgeFederateHla1516e::EfcsBridgeFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_DiscovObjectInstanceFullSensorModelData = false;
	_NewAttributeFullSensorModelData = false;
	_DiscovObjectInstanceFullConfigurationData = false;
	_NewAttributeFullConfigurationData = false;
	_DiscovObjectInstanceFullCockpitModelData = false;
	_NewAttributeFullCockpitModelData = false;
	
	memset(&_efcs_data_out, 0, sizeof(_efcs_data_out));
	memset(&_efcs_data_in_from_remote, 0, sizeof(efcs_only_data_hla_t));
	memset(&_sensors_data_in, 0, sizeof(_sensors_data_in));
	memset(&_config_data_in, 0, sizeof(_config_data_in));
	memset(&_sim_time_data_in, 0, sizeof(_sim_time_data_in));
	memset(&_cockpit_data_in, 0, sizeof(_cockpit_data_in));
	memset(&_hla_to_fcc_data_out, 0, sizeof(hla_to_fcc_data_hla_t));
	
	_InputSocketMessagebuffer.reset();
	_OutputSocketMessagebuffer.reset();
	_InputSocketMessagebuffer.resize(1400);
	_OutputSocketMessagebuffer.resize(1400);
    
    
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
EfcsBridgeFederateHla1516e::~EfcsBridgeFederateHla1516e()
{
}

//----------------------------------------------------------------------
// Start Threads
void EfcsBridgeFederateHla1516e::setAndConfigureUdpBridge( bool IsExternalFcs
		                                                 , std::string BridgeIpAddress
		                                                 , int BridgeUdpPort
		                                                 , std::string FcsIpAddress
		                                                 , int FcsUdpPort)
{
	_IsExternalFcs = IsExternalFcs;
	_BridgeIpAddress = BridgeIpAddress;
	_BridgeUdpPort = BridgeUdpPort;
	_FcsIpAddress = FcsIpAddress;
	_FcsUdpPort = FcsUdpPort;
	_OutputSocket.constructSocket(BridgeIpAddress, BridgeUdpPort,FcsIpAddress, FcsUdpPort);
	_InputSocket.constructSocket(BridgeIpAddress, BridgeUdpPort);
}

//----------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EfcsBridgeFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void EfcsBridgeFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L"EfcsBridgeFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle EfcsBridgeFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


//----------------------------------------------------------------------
// Set all Time management settings for federate
void EfcsBridgeFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void EfcsBridgeFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
		std::wstring ClassFullSimulationTimeModel (L"ClassFullSimulationTimeModel");
		std::wstring AttrFullSimulationTimeModelData (L"AttrFullSimulationTimeModelData");
		_ObjectClassHandleClassFullSimulationTimeModel = _RtiAmb->getObjectClassHandle(ClassFullSimulationTimeModel);
		_AttributeHandleFullSimulationTimeModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSimulationTimeModel, AttrFullSimulationTimeModelData);
		
		std::wstring ClassFullSensorModel (L"ClassFullSensorModel");
		std::wstring AttrFullSensorModelData (L"AttrFullSensorModelData");
		_ObjectClassHandleClassFullSensorModel = _RtiAmb->getObjectClassHandle(ClassFullSensorModel);
		_AttributeHandleFullSensorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSensorModel, AttrFullSensorModelData);
		
		std::wstring ClassFullConfiguration (L"ClassFullConfiguration");
		std::wstring AttrFullConfigurationData (L"AttrFullConfigurationData");
		_ObjectClassFullConfiguration = _RtiAmb->getObjectClassHandle(ClassFullConfiguration);
		_AttributeHandleFullConfigurationData = _RtiAmb->getAttributeHandle(_ObjectClassFullConfiguration, AttrFullConfigurationData);
		
		std::wstring ClassFullCockpitModel (L"ClassFullCockpitModel");
		std::wstring AttrFullCockpitModelData (L"AttrFullCockpitModelData");
		_ObjectClassHandleClassFullCockpitModel = _RtiAmb->getObjectClassHandle(ClassFullCockpitModel);
		_AttributeHandleFullCockpitModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullCockpitModel, AttrFullCockpitModelData);
		
		// Published
		std::wstring ClassFullEfcsModel (L"ClassFullEfcsModel");
		std::wstring AttrFullEfcsModelData (L"AttrFullEfcsModelData");
		_ObjectClassHandleClassFullEfcsModel = _RtiAmb->getObjectClassHandle(ClassFullEfcsModel);
		_AttributeHandleFullEfcsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEfcsModel, AttrFullEfcsModelData);
		
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void EfcsBridgeFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed 
		_attrFullSimulationTimeModelData.insert(_AttributeHandleFullSimulationTimeModelData);      
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSimulationTimeModel,_attrFullSimulationTimeModelData);
		_attrFullSensorModelData.insert(_AttributeHandleFullSensorModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSensorModel,_attrFullSensorModelData);     
		_attrFullConfigurationData.insert(_AttributeHandleFullConfigurationData);
        _RtiAmb->subscribeObjectClassAttributes(_ObjectClassFullConfiguration,_attrFullConfigurationData);
        _attrFullCockpitModelData.insert(_AttributeHandleFullCockpitModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullCockpitModel,_attrFullCockpitModelData);
        
        // For Class/Attributes published 
        _attrFullEfcsModelData.insert(_AttributeHandleFullEfcsModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullEfcsModel,_attrFullEfcsModelData);	
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void EfcsBridgeFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{

        std::wstring FullEfcsModelData (L"FullEfcsModelData");
        _ObjectInstanceHandleFullEfcsModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullEfcsModel,FullEfcsModelData);

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void EfcsBridgeFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {    
		_RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullSimulationTimeModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassFullConfiguration);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullSensorModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullCockpitModel);
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
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullEfcsModel);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void EfcsBridgeFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( //!_DiscovObjectInstanceFullSimulationTimeModelData ||
			!_DiscovObjectInstanceFullSensorModelData ||
			!_DiscovObjectInstanceFullCockpitModelData 
			//!_DiscovObjectInstanceFullConfigurationData 
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
	_DiscovObjectInstanceFullSensorModelData = false;
	_DiscovObjectInstanceFullConfigurationData = false;
	_DiscovObjectInstanceFullCockpitModelData = false;
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void EfcsBridgeFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (//!_NewAttributeFullSimulationTimeModelData ||
		   !_NewAttributeFullSensorModelData)// ||
		   //!_NewAttributeFullCockpitModelData 
		   //!_NewAttributeFullConfigurationData
		   //)
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
	_NewAttributeFullSensorModelData = false;
	_NewAttributeFullConfigurationData = false;
	_NewAttributeFullCockpitModelData = false;
    #ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Callback : discover object instance
void EfcsBridgeFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_FLIGHT_DYNAMICS_FED
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring SimulationTimeModelData (L"SimulationTimeModelData");
    std::wstring FullSensorModelData (L"FullSensorModelData");
    std::wstring FullConfigurationData (L"FullConfigurationData");
    std::wstring CockpitModelData (L"CockpitModelData");
    if ( (theObjectClass == _ObjectClassHandleClassFullSimulationTimeModel) &&  (!wcscmp(theObjectInstanceName.c_str(),SimulationTimeModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullSimulationTimeModelData = true;
		_ObjectInstanceHandleFullSimulationTimeModelData = theObject;
		#ifdef DEBUG_EFCS_FEDERATE_1516E
		std::wcout << L"< SimulationTimeModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassFullConfiguration) &&  (!wcscmp(theObjectInstanceName.c_str(),FullConfigurationData.c_str())) ) 
	{
		_DiscovObjectInstanceFullConfigurationData = true;
		_ObjectInstanceHandleFullConfiguration = theObject;
		#ifdef DEBUG_EFCS_FEDERATE_1516E
		std::wcout << L"< FullConfigurationData > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ObjectClassHandleClassFullSensorModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullSensorModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullSensorModelData = true;
		_ObjectInstanceHandleFullSensorModelData = theObject;
		#ifdef DEBUG_EFCS_FEDERATE_1516E
		std::wcout << L"< FullSensorModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullCockpitModel) &&  (!wcscmp(theObjectInstanceName.c_str(),CockpitModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullCockpitModelData = true;
		_ObjectInstanceHandleFullCockpitModelData = theObject;
		#ifdef DEBUG_EFCS_FEDERATE_1516E
		std::wcout << L"< CockpitModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
	else
	{
		#ifdef DEBUG_EFCS_FEDERATE_1516E
		std::wcout << L"RAV PROBEL > Object Instance has been discovered" << std::endl;
		#endif
	}
    #ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EfcsBridgeFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void EfcsBridgeFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FEDERATE_1516E
    std::wcout << L"EfcsBridgeFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Init phase for Efcs
void EfcsBridgeFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	_f16_efcs.load_tables();
	_OutputSocketMessagebuffer.reset();
	_OutputSocketMessagebuffer.write_bytes((char*)&_hla_to_fcc_data_out, sizeof(hla_to_fcc_data_hla_t));
	_OutputSocketMessagebuffer.updateReservedBytes();
	_OutputSocket.send(static_cast<char*>(_OutputSocketMessagebuffer(0)), _OutputSocketMessagebuffer.size());
}

//----------------------------------------------------------------------
// Calculate State values for Efcs
void EfcsBridgeFederateHla1516e::calculateState() 
{
	// Send to remote Fcs
	if (_IsExternalFcs)
	{
		_OutputSocketMessagebuffer.reset();
		_OutputSocketMessagebuffer.write_bytes((char*)&_hla_to_fcc_data_out, sizeof(hla_to_fcc_data_hla_t));
		_OutputSocketMessagebuffer.updateReservedBytes();
		_OutputSocket.send(static_cast<char*>(_OutputSocketMessagebuffer(0)), _OutputSocketMessagebuffer.size());
	}
	_f16_efcs.ap_mode();
	_f16_efcs.ap_laws(0.02);
	_f16_efcs.fbw_lef(0.02);
	_f16_efcs.fbw_tef(0.02);
	_f16_efcs.fbw_roll(0.02);
	_f16_efcs.fbw_pitch(0.02);
	_f16_efcs.fbw_yaw(0.02);
}

//----------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EfcsBridgeFederateHla1516e::calculateOutput() 
{
	// Send to remote Fcs
	if (_IsExternalFcs)
	{
		_InputSocketMessagebuffer.reset(); 
		_RetInputSocket = _InputSocket.receive(static_cast<char*>(_InputSocketMessagebuffer(0)), _InputSocketMessagebuffer.maxSize());
		if (_RetInputSocket > 0)
		{
			_InputSocketMessagebuffer.assumeSizeFromReservedBytes();
			_RemoteUdpPort =  _InputSocket.getLastRegisteredRemoteUdpPort();
			if ( (_RemoteUdpPort == _FcsUdpPort))
			{
				_InputSocketMessagebuffer.read_bytes((char*)&_efcs_data_in_from_remote, sizeof(efcs_only_data_hla_t));
			}
		}
	}
}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void EfcsBridgeFederateHla1516e::sendInitialHlaAttributesTrim()
{
	//std::wcout << L"FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;

}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void EfcsBridgeFederateHla1516e::sendInitialHlaAttributes()
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
		std::wcout  << L"Error: unknown non-RTI exception." << std::endl;
	}
}


//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void EfcsBridgeFederateHla1516e::sendHlaAttributes(RTI1516fedTime UpdateTime)
{
	std::wcout  << L"EfcsBridgeFederateHla1516e::sendHlaAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvpsEfcsModelData;

    rti1516e::VariableLengthData MyTag1;
	std::string testTag ("efcs_data");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    

    // _fdm_out = _fdm_entity.get_fdm();
    if (_IsExternalFcs)
	{
		_efcs_data_out.f16_fcs_fed = _efcs_data_in_from_remote.f16_fcs;
		_efcs_data_out.fcs_fed_perf = _efcs_data_in_from_remote.fc_perf;
	}
	else
	{
		_efcs_data_out.f16_fcs_fed = _f16_efcs.get_fcs_cmd();
		_efcs_data_out.fcs_fed_perf.wcet_ms = (float) _TotalRealTimeMs;
	}
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_bytes((char*)&_efcs_data_out, sizeof(_efcs_data_out)) ;
    _OutputMessagebuffer.updateReservedBytes();
    
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsEfcsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullEfcsModelData,attrValue1)); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEfcsModelData, ahvpsEfcsModelData, MyTag1);
		}
		else
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullEfcsModelData, ahvpsEfcsModelData, MyTag1, UpdateTime);
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
	std::wcout << L"EfcsBridgeFederateHla1516e::sendHlaAttributes: End" << std::endl;
}

//----------------------------------------------------------------------
// Callback : reflect attribute values with time
void EfcsBridgeFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void EfcsBridgeFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
				_InputMessagebuffer.read_bytes((char*)&_sim_time_data_in, sizeof(_sim_time_data_in));
				//_ap.set_cockpit(_cockpit_in);
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> _sim_time_data_in " << std::endl; 
			}
            else
            { 
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " << parmHandle << "." << std::endl; 
			}
        }
        _NewAttributeFullSimulationTimeModelData = true;
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
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> _sensors_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_sensors_data_in, sizeof(_sensors_data_in));
				_f16_efcs.set_state(_sensors_data_in.f16_fdm_debug.x);
				_hla_to_fcc_data_out.f16_fdm_debug = _sensors_data_in.f16_fdm_debug;
			}
            else
            { 
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " << parmHandle << "." << std::endl; 
			}
        }
        _NewAttributeFullSensorModelData = true;
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
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> _config_data_in " << std::endl; 
			}
            else
            { 
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " << parmHandle << "." << std::endl; 
			}
        }
        _NewAttributeFullConfigurationData = true;
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
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> _cockpit_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_cockpit_data_in, sizeof(_cockpit_data_in));
				_f16_efcs.set_cockpit(_cockpit_data_in.f16_cockpit);
				_hla_to_fcc_data_out.f16_cockpit = _cockpit_data_in.f16_cockpit;
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " << parmHandle << "." << std::endl; 
			}
        }
        _NewAttributeFullCockpitModelData = true;
    }
	else
	{ 
		std::wcout << L"EfcsBridgeFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

//----------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EfcsBridgeFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void EfcsBridgeFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void EfcsBridgeFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_EFCS_FEDERATE_1516E
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

//----------------------------------------------------------------------
void EfcsBridgeFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void EfcsBridgeFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void EfcsBridgeFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void EfcsBridgeFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void EfcsBridgeFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
//----------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EfcsBridgeFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EfcsBridgeFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : federationSynchronized
void EfcsBridgeFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

//----------------------------------------------------------------------
// function : run
void EfcsBridgeFederateHla1516e::run()
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
void EfcsBridgeFederateHla1516e::runOneStep()
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

bool EfcsBridgeFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
