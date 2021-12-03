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
#include "ConfigurationParametersParser.hh"
using namespace ConfigurationParser;

//----------------------------------------------------------------------
// TemplateFederateHla13 Constructor
FlightDynamicsFederateHla1516e::FlightDynamicsFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	
    _DiscovObjectInstanceFullApModelData = false;
    _NewAttributeFullApModelData     = false;
	_DiscovObjectInstanceFullEngModelData = false;
    _NewAttributeFullEngModelData     = false;
    
    memset(&_fdm_out, 0, sizeof(_fdm_out));
    memset(&_cockpit_in, 0, sizeof(_cockpit_in));
    memset(&_eng_in, 0, sizeof(_eng_in));
	//memset(&_SimToSensorsMessage, 0, sizeof(sim_to_sensors_message_t));
	//memset(&_FcsToSimMessage, 0, sizeof(fcs_to_sim_fdm_t));

	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void FlightDynamicsFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void FlightDynamicsFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
		std::wstring ClassFullApModel (L"ClassFullApModel");
		std::wstring AttrFullApModelData (L"AttrFullApModelData");
		_ObjectClassHandleClassFullApModel = _RtiAmb->getObjectClassHandle(ClassFullApModel);
		_AttributeHandleFullApModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullApModel, AttrFullApModelData);
		
		std::wstring ClassFullEngModel (L"ClassFullEngModel");
		std::wstring AttrFullEngModelData (L"AttrFullEngModelData");
		_ObjectClassHandleClassFullEngModel = _RtiAmb->getObjectClassHandle(ClassFullEngModel);
		_AttributeHandleFullEngModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullEngModel, AttrFullEngModelData);
		
		// Published
		std::wstring ClassFullFdmModel (L"ClassFullFdmModel");
		std::wstring AttrFullFdmModelData (L"AttrFullFdmModelData");
		_ObjectClassHandleClassFullFdmModel = _RtiAmb->getObjectClassHandle(ClassFullFdmModel);
		_AttributeHandleFullFdmModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleClassFullFdmModel, AttrFullFdmModelData);
		
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void FlightDynamicsFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		_attrFullApModelData.insert(_AttributeHandleFullApModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullApModel,_attrFullApModelData);
		_attrFullEngModelData.insert(_AttributeHandleFullEngModelData);       
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleClassFullEngModel,_attrFullEngModelData);

		// For Class/Attributes published
		_attrFullFdmModelData.insert(_AttributeHandleFullFdmModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleClassFullFdmModel,_attrFullFdmModelData);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring FlightDynamicsF16 (L"FlightDynamicsF16");
        _ObjectInstanceHandleFullFdmModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleClassFullFdmModel,FlightDynamicsF16);

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void FlightDynamicsFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullApModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleClassFullEngModel);
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
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleClassFullFdmModel);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_DiscovObjectInstanceFullApModelData ||
			!_DiscovObjectInstanceFullEngModelData)
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
	_DiscovObjectInstanceFullApModelData = false;
	_DiscovObjectInstanceFullEngModelData = false;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_NewAttributeFullApModelData ||
		   !_NewAttributeFullEngModelData)
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
    _NewAttributeFullApModelData = false;
    _NewAttributeFullEngModelData = false;
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
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
    std::wstring AutopilotF16 (L"AutopilotF16");
    std::wstring EngineF16 (L"EngineF16");
    if ( (theObjectClass == _ObjectClassHandleClassFullApModel) &&  (!wcscmp(theObjectInstanceName.c_str(),AutopilotF16.c_str())) ) 
	{
		_DiscovObjectInstanceFullApModelData = true;
		_ObjectInstanceHandleFullApModelData = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"< AutopilotF16 > Object Instance has been discovered" << std::endl;
		#endif
	}
	else if ( (theObjectClass == _ObjectClassHandleClassFullApModel) &&  (!wcscmp(theObjectInstanceName.c_str(),EngineF16.c_str())) ) 
	{
		_DiscovObjectInstanceFullApModelData = true;
		_ObjectInstanceHandleFullApModelData = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"< EngineF16 > Object Instance has been discovered" << std::endl;
		#endif
	} 
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void FlightDynamicsFederateHla1516e::pauseInitTrim()
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
void FlightDynamicsFederateHla1516e::pauseInitSim()
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
	_f16_fdm_entity.load_tables_aero_f();
	_f16_fdm_entity.set_trim_conditions(vt_trim, 0.0, 0.0, alt_trim, p_trim, q_trim, r_trim, psi_trim);
	_f16_fdm_entity.set_trim_init(20000, 0.3, -0.03 * 57.29577951, 0.0, 0.0, 0.3);
	_f16_fdm_entity.trim_aircraft();
	_f16_fdm_entity.compute_xdot(0.02);
	_f16_fdm_entity.u2fcs();
	_f16_fdm_entity.u2cs();
	_f16_ap.load_tables();
	_f16_ap.set_fcs_cmd(_f16_fdm_entity.get_fcs_cmd());
	_f16_ap.set_state(_f16_fdm_entity.get_x());
	
	sendInitialHlaAttributes();  
	
	// Log
	std::string csv_log = "test_log.csv";
	_f16_ap.enable_log();
	_f16_ap.open_csv(csv_log.c_str());
    
}

//----------------------------------------------------------------------
// Calculate State values for Efcs
void FlightDynamicsFederateHla1516e::calculateState() 
{
	// Model State calculation
	_f16_fdm_entity.compute_xdot(0.02);
	_f16_ap.set_state(_f16_fdm_entity.get_x());
	_f16_ap.ap_mode();
	_f16_ap.ap_laws(0.02);
	_f16_ap.fbw_lef(0.02);
	_f16_ap.fbw_tef(0.02);
	_f16_ap.fbw_roll(0.02);
	_f16_ap.fbw_pitch(0.02);
	_f16_ap.fbw_yaw(0.02);
	_f16_ap.write_csv();
	_f16_fdm_entity.set_fcs_cmd(_f16_ap.get_fcs_cmd());
}

//----------------------------------------------------------------------
// Calculate Ouput values for Efcs
void FlightDynamicsFederateHla1516e::calculateOutput() 
{
	
}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendInitialHlaAttributesTrim()
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;

}

//----------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendInitialHlaAttributes()
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;

    rti1516e::VariableLengthData MyTag1;
	std::string testTag ("Aircraft_F16-Trim");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    
    _OutputMessagebuffer.reset() ;
    _fdm_out = _f16_fdm_entity.get_fdm();
    _OutputMessagebuffer.write_bytes((char*)&_fdm_out, sizeof(_fdm_out)) ;
    _OutputMessagebuffer.updateReservedBytes();
    
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFdmModelData,attrValue1)); 
    
    try 
    {
		_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1);
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
void FlightDynamicsFederateHla1516e::sendHlaAttributes(RTI1516fedTime UpdateTime)
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvpsFlightDynamicsModelData;

    rti1516e::VariableLengthData MyTag1;
	std::string testTag ("Aircraft_F16");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
    
    _OutputMessagebuffer.reset() ;
    _fdm_out = _f16_fdm_entity.get_fdm();
    _OutputMessagebuffer.write_bytes((char*)&_fdm_out, sizeof(_fdm_out)) ;
    _OutputMessagebuffer.updateReservedBytes();
    
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvpsFlightDynamicsModelData.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AttributeHandleFullFdmModelData,attrValue1)); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1);
		}
		else
        {
			_RtiAmb->updateAttributeValues(_ObjectInstanceHandleFullFdmModel, ahvpsFlightDynamicsModelData, MyTag1, UpdateTime);
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
    
    if (theObject == _ObjectInstanceHandleFullApModelData)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			//assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
			buffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _AttributeHandleFullApModelData)  
            { 
				buffer.read_bytes((char*)&_cockpit_in, sizeof(_cockpit_in));
				_f16_ap.set_cockpit(_cockpit_in);
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _NewAttributeFullApModelData = true;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
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
		std::wcout << L"_TotalRealTimeMs = " 
				     << _TotalRealTimeMs
				     << std::endl;
		if (_TotalRealTimeMs < 20.0)
		{
			usleep((_TimeStep.getFedTime()-_TotalRealTimeMs)*1000);
		}
		cnt++;
		if (cnt == 5000) _f16_ap.close_csv();
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
	std::wcout << L"_TotalRealTimeMs = " 
				     << _TotalRealTimeMs
				     << std::endl;
	if (_TotalRealTimeMs < 20.0)
	{
		usleep((_TimeStep.getFedTime()-_TotalRealTimeMs)*1000);
	}
} 

bool FlightDynamicsFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
