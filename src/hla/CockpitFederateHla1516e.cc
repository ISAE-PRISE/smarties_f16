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

#include "CockpitFederateHla1516e.hh"
#include "CockpitFederateThreadHla1516e.hh"
#include "Cockpit.hh"
#include "ConfigurationParametersParser.hh"
using namespace ConfigurationParser;




//----------------------------------------------------------------------
// TemplateFederateHla13 Constructor
CockpitFederateHla1516e::CockpitFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false ;
	_IsOutMesTimespamped = true;
	
	_DiscovObjectInstanceFullSimulationTimeModelData = false;
	_NewAttributeFullSimulationTimeModelData = false;
	_DiscovObjectInstanceFullSensorModelData = false;
	_NewAttributeFullSensorModelData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;
	_NewAttributeFullEfcsModelData = false;
	
	memset(&_efcs_data_in, 0, sizeof(_efcs_data_in));
	memset(&_sensors_data_in, 0, sizeof(_sensors_data_in));
	memset(&_config_data_in, 0, sizeof(_config_data_in));
	memset(&_sim_time_data_in, 0, sizeof(_sim_time_data_in));
	memset(&_cockpit_data_out, 0, sizeof(_cockpit_data_out));
	
	
	_OneAircraftConnected = false;
	//_ConnectedAircraftStringId = L"";
	//_ObjectInstanceHandleForClassControlCommandModelName = L"";
	
	// Local variables
	_Aileron = _Elevator = _Rudder = _Throttle_Left = 0.0;
	_Throttle_Right = _Flaps = _Spoilers = _Gears = 0.0;
	_Brakes = _Stabilizer = _ParkingBrakes = 0.0;


	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
CockpitFederateHla1516e::~CockpitFederateHla1516e()
{
}


//----------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void CockpitFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb->createFederationExecution( _FederationName
										 , _FomFileName
										 );
	  _IsCreator = true;
    } 
    catch ( rti1516e::FederationExecutionAlreadyExists ) 
    {
		std::wcout << L"CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
		 _IsCreator = false;
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
    #ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void CockpitFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"cockpit"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle CockpitFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


//----------------------------------------------------------------------
// Set all Time management settings for federate
void CockpitFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void CockpitFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Published 
		std::wstring ClassFullCockpitModel (L"ClassFullCockpitModel");
		std::wstring AttrFullCockpitModelData (L"AttrFullCockpitModelData");
		_ObjectClassHandleClassFullCockpitModel = _RtiAmb->getObjectClassHandle(ClassFullCockpitModel);
		_AttributeHandleFullCockpitModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullCockpitModel, AttrFullCockpitModelData);
		
		// Subscribe
		std::wstring ClassFullSimulationTimeModel (L"ClassFullSimulationTimeModel");
		std::wstring AttrFullSimulationTimeModelData (L"AttrFullSimulationTimeModelData");
		_ObjectClassHandleClassFullSimulationTimeModel = _RtiAmb->getObjectClassHandle(ClassFullSimulationTimeModel);
		_AttributeHandleFullSimulationTimeModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSimulationTimeModel, AttrFullSimulationTimeModelData);
		
		std::wstring ClassFullSensorModel (L"ClassFullSensorModel");
		std::wstring AttrFullSensorModelData (L"AttrFullSensorModelData");
		_ObjectClassHandleClassFullSensorModel = _RtiAmb->getObjectClassHandle(ClassFullSensorModel);
		_AttributeHandleFullSensorModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullSensorModel, AttrFullSensorModelData);
		
		std::wstring ClassFullEfcsModel (L"ClassFullEfcsModel");
		std::wstring AttrFullEfcsModelData (L"AttrFullEfcsModelData");
		_ObjectClassHandleClassFullEfcsModel = _RtiAmb->getObjectClassHandle(ClassFullEfcsModel);
		_AttributeHandleFullEfcsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEfcsModel, AttrFullEfcsModelData);
		
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
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void CockpitFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes  subscribed
		_attrFullConfigurationData.insert(_AttributeHandleFullConfigurationData);
        _RtiAmb->subscribeObjectClassAttributes(_ObjectClassFullConfiguration,_attrFullConfigurationData);
		_attrFullSimulationTimeModelData.insert(_AttributeHandleFullSimulationTimeModelData);      
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSimulationTimeModel,_attrFullSimulationTimeModelData);
		_attrFullSensorModelData.insert(_AttributeHandleFullSensorModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullSensorModel,_attrFullSensorModelData);
		_attrFullEfcsModelData.insert(_AttributeHandleFullEfcsModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEfcsModel,_attrFullEfcsModelData);

		// For Class/Attributes published
		_attrFullCockpitModelData.insert(_AttributeHandleFullCockpitModelData);       
		_RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullCockpitModel,_attrFullCockpitModelData);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		//std::wstring CockpitObjectBaseName (L"ExternalCockpitObject-"); 
		//_ObjectInstanceHandleForClassControlCommandModelName = CockpitObjectBaseName + _FederateName;
		std::wstring CockpitModelData (L"CockpitModelData");
		// _ObjectInstanceHandleForClassControlCommandModelName = CockpitModelData;
		_ObjectInstanceHandleFullCockpitModelData = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullCockpitModel,CockpitModelData);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void CockpitFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullSimulationTimeModel);
		_RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEfcsModel);
		_RtiAmb->unsubscribeObjectClass(_ObjectClassFullConfiguration);
		_RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullSensorModel);
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
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullCockpitModel);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( //!_DiscovObjectInstanceFullSimulationTimeModelData ||
			!_DiscovObjectInstanceFullSensorModelData ||
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
	_DiscovObjectInstanceFullSimulationTimeModelData = false;
	_DiscovObjectInstanceFullSensorModelData = false;
	_DiscovObjectInstanceFullEfcsModelData = false;
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (//!_NewAttributeFullSimulationTimeModelData ||
			!_NewAttributeFullSensorModelData ||
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
    _NewAttributeFullSimulationTimeModelData = false;
	_NewAttributeFullSensorModelData = false;
	_NewAttributeFullEfcsModelData = false;
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Callback : discover object instance
void CockpitFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring SimulationTimeModelData (L"SimulationTimeModelData");
    std::wstring FullSensorModelData (L"FullSensorModelData");
    std::wstring FullEfcsModelData (L"FullEfcsModelData");

    if ( (theObjectClass == _ObjectClassHandleClassFullSimulationTimeModel) &&  (!wcscmp(theObjectInstanceName.c_str(),SimulationTimeModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullSimulationTimeModelData = true;
		_ObjectInstanceHandleFullSimulationTimeModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< SimulationTimeModelData > Object Instance has been discovered" << std::endl;
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
	else if ( (theObjectClass == _ObjectClassHandleClassFullEfcsModel) &&  (!wcscmp(theObjectInstanceName.c_str(),FullEfcsModelData.c_str())) ) 
	{
		_DiscovObjectInstanceFullEfcsModelData = true;
		_ObjectInstanceHandleFullEfcsModelData = theObject;
		#ifdef DEBUG_MANAGER_FEDERATE_1516E
		std::wcout << L"< EfcsModelData > Object Instance has been discovered" << std::endl;
		#endif
	}
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}


//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void CockpitFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void CockpitFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}


//----------------------------------------------------------------------
// 
void CockpitFederateHla1516e::initHlaStructs() 
{
	// Data transmitted
}
//----------------------------------------------------------------------
// 
void CockpitFederateHla1516e::initialization() 
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> initialization(): Start" << std::endl;
	#endif
	
	//waitForAllObjectDiscovered();
	
	// Parse the init XML file for joystick
	XMLDocument* params = loadFileToDoc("SideStick1Config.xml");
	vector<string> input_vector(3);
	input_vector[0] = "JOYSTICK";
	input_vector[1] = "input";
	input_vector[2] = "Aileron";
	_Aileron_port   = getIntValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_port  = getIntValue(params, input_vector); 
	input_vector[2] = "Rudder";
	_Rudder_port    = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_port  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_port  = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Up";
	_Flaps_Up_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Flaps_Down";
	_Flaps_Down_port= getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Spoilers";
	_Spoilers_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Gears";
	_Gears_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Brakes";
	_Brakes_port = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[1] = "port";
	input_vector[2] = "Aileron";
	_Aileron_jnb = getIntValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Rudder";
	_Rudder_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Up";
	_Flaps_Up_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Down";
	_Flaps_Down_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Spoilers";
	_Spoilers_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Gears";
	_Gears_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Brakes";
	_Brakes_jnb    = getIntValue(params, input_vector);
	input_vector[1] = "sensitivity";
	input_vector[2] = "Aileron";
	_Aileron_sens   = getDoubleValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_sens  = getDoubleValue(params, input_vector);
	input_vector[2] = "Rudder";
	_Rudder_sens    = getDoubleValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_sens  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_sens = getIntValue(params, input_vector);
	delete params;
	
	std::wcout << L"Side Stick System created with following configuration: " << std::endl;
	std::wcout << L"Configuration Joystick:  " << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Aileron_port :  " << _Aileron_port << std::endl;
    std::wcout << L"_Aileron_jnb  :  " << _Aileron_jnb << std::endl;
    std::wcout << L"_Aileron_sens :  " << _Aileron_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Elevator_port :  " << _Elevator_port << std::endl;
    std::wcout << L"_Elevator_jnb  :  " << _Elevator_jnb << std::endl;
    std::wcout << L"_Elevator_sens :  " << _Elevator_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Rudder_port :  " << _Rudder_port << std::endl;
    std::wcout << L"_Rudder_jnb  :  " << _Rudder_jnb << std::endl;
    std::wcout << L"_Rudder_sens :  " << _Rudder_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Throttle_Left_port :  " << _Throttle_Left_port << std::endl;
    std::wcout << L"_Throttle_Left_jnb  :  " << _Throttle_Left_jnb << std::endl;
    std::wcout << L"_Throttle_Left_sens :  " << _Throttle_Left_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Throttle_Right_port :  " << _Throttle_Right_port << std::endl;
    std::wcout << L"_Throttle_Right_jnb  :  " << _Throttle_Right_jnb << std::endl;
    std::wcout << L"_Throttle_Right_sens :  " << _Throttle_Right_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Flaps_Up_port :  " << _Flaps_Up_port << std::endl;
    std::wcout << L"_Flaps_Up_jnb  :  " << _Flaps_Up_jnb << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Flaps_Down_port :  " << _Flaps_Down_port << std::endl;
    std::wcout << L"_Flaps_Down_jnb  :  " << _Flaps_Down_jnb << std::endl;
    std::wcout << L"" << std::endl;   
    std::wcout << L"_Spoilers_port :  " << _Spoilers_port << std::endl;
    std::wcout << L"_Spoilers_jnb  :  " << _Spoilers_jnb << std::endl;
    std::wcout << L"" << std::endl;     
    std::wcout << L"_Gears_port :  " << _Gears_port << std::endl;
    std::wcout << L"_Gears_jnb  :  " << _Gears_jnb << std::endl;
    std::wcout << L"" << std::endl;    
    std::wcout << L"_Brakes_port :  " << _Brakes_port << std::endl;
    std::wcout << L"_Brakes_jnb  :  " << _Brakes_jnb << std::endl;
    std::wcout << L"" << std::endl;
    
    _cockpit_data_out.f16_cockpit.fcu.ap_en = CockpitFederateThreadHla1516e::window->Get_Autopilot_AP();
	_cockpit_data_out.f16_cockpit.fcu.vt_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_spd();
	_cockpit_data_out.f16_cockpit.fcu.psi_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg();
	_cockpit_data_out.f16_cockpit.fcu.alt_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_alt();
	_cockpit_data_out.f16_cockpit.fcu.vs_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_vs();
	
	sendUpdateAttributesTrim();
    
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> initialization(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// 
void CockpitFederateHla1516e::calculateState() 
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> calculateState(): Start" << std::endl;
	#endif
	Inputs = _Joystick.getInputs();

	// Get axis inputs
	_Aileron          = Inputs[_Aileron_jnb       *MAX_INPUTS + _Aileron_port];
	_Elevator         = Inputs[_Elevator_jnb      *MAX_INPUTS + _Elevator_port];
	_Rudder           = Inputs[_Rudder_jnb        *MAX_INPUTS + _Rudder_port];
	_Throttle_Left    = Inputs[_Throttle_Left_jnb *MAX_INPUTS + _Throttle_Left_port];
	_Throttle_Right   = Inputs[_Throttle_Right_jnb*MAX_INPUTS + _Throttle_Right_port];

	if ( Inputs[_Flaps_Up_jnb  *MAX_INPUTS + _Flaps_Up_port] &&  _Flaps_Up_released)
	{
		// Go to next flaps position
		_Flaps += 0.34; 
		if(_Flaps >1) _Flaps =1;
		_Flaps_Up_released = false;
	}
	else if (!Inputs[_Flaps_Up_jnb  *MAX_INPUTS + _Flaps_Up_port] && !_Flaps_Up_released)
	{
		_Flaps_Up_released = true;
	}
	
	if ( Inputs[_Flaps_Down_jnb*MAX_INPUTS + _Flaps_Down_port] &&  _Flaps_Down_released)
	{
		// Go to previous flaps position
		_Flaps -= 0.34; 
		if(_Flaps <0) _Flaps =0;
		_Flaps_Down_released = false;
	}
	else if (!Inputs[_Flaps_Down_jnb*MAX_INPUTS + _Flaps_Down_port] && !_Flaps_Down_released)
	{
		_Flaps_Down_released = true;
	}
	
	if ( Inputs[_Spoilers_jnb  *MAX_INPUTS + _Spoilers_port] &&  _Spoilers_released)
	{
		// Switch spoilers position
		_Spoilers = 1 - _Spoilers; 
		_Spoilers_released = false;
	}
	else if (!Inputs[_Spoilers_jnb  *MAX_INPUTS + _Spoilers_port] && !_Spoilers_released)
	{
		_Spoilers_released = true;
	}
	
	if ( Inputs[_Gears_jnb  *MAX_INPUTS + _Gears_port] &&  _Gears_released)
	{
		// Switch Gears position
		_Gears = 1 - _Gears; 
		_Gears_released = false;
	}
	else if (!Inputs[_Gears_jnb  *MAX_INPUTS + _Gears_port] && !_Gears_released)
	{
		_Gears_released = true;
	}
	
	if ( Inputs[_Brakes_jnb  *MAX_INPUTS + _Brakes_port] &&  _Brakes_released)
	{
		// Switch Brakes position
		_Brakes = 1 - _Brakes; 
		_Brakes_released = false;
	}
	else if (!Inputs[_Brakes_jnb  *MAX_INPUTS + _Brakes_port] && !_Brakes_released)
	{
		_Brakes_released = true;
	}

	// Adjust sensitivity
	_Aileron        = _Aileron*_Aileron_sens;
	_Elevator       = _Elevator*_Elevator_sens;
	_Rudder         = _Rudder*_Rudder_sens;
	_Throttle_Left  = _Throttle_Left*_Throttle_Left_sens;
	_Throttle_Right = _Throttle_Right*_Throttle_Right_sens;

	// Throttles need to be in [0,1]
	_Throttle_Left  = (1 + _Throttle_Left )/2;
	_Throttle_Right = (1 + _Throttle_Right)/2;
	
	std::wcout << L"Joystick (Elevator) :  " << _Elevator << std::endl;
	std::wcout << L"Joystick (Aileron) :   " << _Aileron << std::endl;
	std::wcout << L"Joystick (Rudder) :    " << _Rudder << std::endl;
	std::wcout << L"Joystick (Throttle Left) :  " << _Throttle_Left << std::endl;
	std::wcout << L"Joystick (Throttle Right) :  " << _Throttle_Right << std::endl;
	std::wcout << L"Joystick (Flaps) :     " << _Flaps << std::endl;
	std::wcout << L"Joystick (Spoilers) :  " << _Spoilers << std::endl;
	std::wcout << L"Joystick (Gears) :     " << _Gears << std::endl;
	std::wcout << L"Joystick (Brakes) :    " << _Brakes << std::endl;
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> calculateState(): End" << std::endl;
	#endif

}

//----------------------------------------------------------------------
// 
void CockpitFederateHla1516e::calculateOutput() 
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> calculateOutput(): Start" << std::endl;
	#endif

	_cockpit_data_out.f16_cockpit.sst.pitch_cmd = _Elevator;
	_cockpit_data_out.f16_cockpit.sst.roll_cmd = _Aileron;
	_cockpit_data_out.f16_cockpit.sst.thr_cmd = _Throttle_Right;
	_cockpit_data_out.f16_cockpit.sst.yaw_cmd = _Rudder;
	
	_cockpit_data_out.f16_cockpit.fcu.ap_en = CockpitFederateThreadHla1516e::window->Get_Autopilot_AP();
	_cockpit_data_out.f16_cockpit.fcu.vt_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_spd();
	_cockpit_data_out.f16_cockpit.fcu.psi_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg();
	_cockpit_data_out.f16_cockpit.fcu.alt_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_alt();
	_cockpit_data_out.f16_cockpit.fcu.vs_ref = (float) CockpitFederateThreadHla1516e::window->Get_Autopilot_vs();
    
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> calculateOutput(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Send Updates for all attributes
void CockpitFederateHla1516e::sendUpdateAttributesTrim()
{ 
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> sendUpdateAttributes(): Start" << std::endl;
	#endif
	//create AttributeHandleValueMap
	rti1516e::AttributeHandleValueMap ahvpsCockpitModelData;
	//Encode Data
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_bytes((char*)&_cockpit_data_out, sizeof(_cockpit_data_out)) ;
    _OutputMessagebuffer.updateReservedBytes();
    
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    ahvpsCockpitModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullCockpitModelData,attrValue1)); 
    std::wstring tag{L""}; // useless ... or maybe ...
    try 
    {

         _RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullCockpitModelData, ahvpsCockpitModelData, {tag.c_str(), tag.size()});

    }
    catch ( rti1516e::Exception &e ) 
	{
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
        std::wcout  << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> sendUpdateAttributes(): End" << std::endl;
	#endif
}


//----------------------------------------------------------------------
// Send Updates for all attributes
void CockpitFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{ 
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> sendUpdateAttributes(): Start" << std::endl;
	#endif
	//create AttributeHandleValueMap
	rti1516e::AttributeHandleValueMap ahvpsCockpitModelData;
	//Encode Data
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_bytes((char*)&_cockpit_data_out, sizeof(_cockpit_data_out)) ;
    _OutputMessagebuffer.updateReservedBytes();
    
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    ahvpsCockpitModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullCockpitModelData,attrValue1)); 
    std::wstring tag{L""}; // useless ... or maybe ...
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullCockpitModelData, ahvpsCockpitModelData, {tag.c_str(), tag.size()});
		}
		else
        {
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullCockpitModelData, ahvpsCockpitModelData, {tag.c_str(), tag.size()}, UpdateTime);
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
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> sendUpdateAttributes(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Callback : reflect attribute values with time
void CockpitFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void CockpitFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
                                             throw ( rti1516e::FederateInternalError) 
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> reflectAttributeValues(): Start" << std::endl;
	#endif
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
				std::wcout << L"CockpitFederateHla1516e.cc: RAV ==> _sim_time_data_in " << std::endl;
				_InputMessagebuffer.read_bytes((char*)&_sim_time_data_in, sizeof(_sim_time_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"ManagerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullSimulationTimeModelData = true;
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
				std::wcout << L"CockpitFederateHla1516e.cc: RAV ==> _efcs_data_in " << std::endl; 
				_InputMessagebuffer.read_bytes((char*)&_efcs_data_in, sizeof(_efcs_data_in));
				//_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"ManagerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for attribute " 
				     << parmHandle 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullEfcsModelData = true;
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
				std::wcout << L"CockpitFederateHla1516e.cc: RAV ==> _sensors_data_in " << std::endl;
				_InputMessagebuffer.read_bytes((char*)&_sensors_data_in, sizeof(_sensors_data_in));
				CockpitFederateThreadHla1516e::window->set_PFD_altitude(_sensors_data_in.f16_fdm_debug.x.alt); // Feet (conversion if * 0.3048)
				CockpitFederateThreadHla1516e::window->set_PFD_roll(_sensors_data_in.f16_fdm_debug.x.phi * COCKPIT_RAD2DEG); // Degrees Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_PFD_pitch(_sensors_data_in.f16_fdm_debug.x.theta * COCKPIT_RAD2DEG); // Degrees Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_PFD_heading(_sensors_data_in.f16_fdm_debug.x.psi * COCKPIT_RAD2DEG); // Degrees Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_ND_heading(_sensors_data_in.f16_fdm_debug.x.psi * COCKPIT_RAD2DEG); // Degrees Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_ND_headingBug(CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg()); // Degrees Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_PFD_airspeed(_sensors_data_in.f16_fdm_debug.x.vt); // Knots?? From m/s to knots is * 1.94
				CockpitFederateThreadHla1516e::window->set_PFD_FlightPath(_sensors_data_in.f16_fdm_debug.x.alpha * COCKPIT_RAD2DEG, _sensors_data_in.f16_fdm_debug.x.beta * COCKPIT_RAD2DEG); 
				CockpitFederateThreadHla1516e::window->set_PFD_mach(_sensors_data_in.f16_fdm_debug.x.mach);
				CockpitFederateThreadHla1516e::window->set_ps(_sensors_data_in.f16_fdm_debug.x.ps);  
				CockpitFederateThreadHla1516e::window->set_mach(_sensors_data_in.f16_fdm_debug.x.mach);
				CockpitFederateThreadHla1516e::window->set_thr(_sensors_data_in.f16_fdm_debug.fcs.dth);
			}
			_NewAttributeFullSensorModelData = true;
        }
	}
	#ifdef DEBUG_COCKPIT_FED_REFLECT
    std::wcout << L"CockpitFederateHla1516e.cc -> reflectAttributeValues(): End" << std::endl;
	#endif

}

//----------------------------------------------------------------------
// Callback : timeRegulationEnabled
void CockpitFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void CockpitFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void CockpitFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

//----------------------------------------------------------------------
void CockpitFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

//----------------------------------------------------------------------
void CockpitFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
//----------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void CockpitFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void CockpitFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

//----------------------------------------------------------------------
// Callback : federationSynchronized
void CockpitFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

//----------------------------------------------------------------------
// function : run
void CockpitFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes(updateTime);
		timeAdvanceRequest(_TimeStep);
	}
} 

//----------------------------------------------------------------------
// function : run
void CockpitFederateHla1516e::runOneStep()
{
	// Model calculations
	RTI1516fedTime updateTime(0.0);
	updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
	calculateState();
	calculateOutput();
	sendUpdateAttributes(updateTime);
	timeAdvanceRequest(_TimeStep);
} 

bool CockpitFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
