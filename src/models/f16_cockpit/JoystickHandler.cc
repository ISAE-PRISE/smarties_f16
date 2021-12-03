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

#include "JoystickHandler.hh"

//----------------------------------------------------------------------
// TemplateFederateHla13 Constructor
JoystickHandler::JoystickHandler( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_TotalRealTimeMs = 0.0;
	_IsRtTimerEnabled = false;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false ;
	_IsOutMesTimespamped = true;
	
	// Init Local values to 0.0
	_Aileron = 0.0;
	_Elevator= 0.0; 
	_Rudder= 0.0;
	_Throttle_Left= 0.0; 
	_Throttle_Right= 0.0; 
	_Flaps= 0.0; 
	_Spoilers= 0.0; 
	_Gears= 0.0;
	_Brakes= 0.0;
	
	memset(&_TimeStamp, 0, sizeof(timespec));
	memset(&_TimeStamp_old, 0, sizeof(timespec));
	memset(&_ExecutionTime, 0, sizeof(timespec));


	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> Constructor(): End" << std::endl;
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
JoystickHandler::~JoystickHandler()
{
}

//----------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void JoystickHandler::createFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
// Destroy a federation from Federation Name
void JoystickHandler::destroyFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void JoystickHandler::joinFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
                                                         , L"joystick"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickHandler.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

//----------------------------------------------------------------------
void JoystickHandler::resignFederationExecution() 
{
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle JoystickHandler::getFederateHandle() const
{
    return _FederateHandle;
}


//----------------------------------------------------------------------
// Set all Time management settings for federate
void JoystickHandler::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// get handles of objet/interaction classes
void JoystickHandler::getAllHandles()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring JOYSTICK (L"JOYSTICK");
		std::wstring AILERON (L"AILERON");
		std::wstring ELEVATOR (L"ELEVATOR");
		std::wstring RUDDER (L"RUDDER");
		std::wstring THROTTLE_LEFT (L"THROTTLE_LEFT");
		std::wstring THROTTLE_RIGHT (L"THROTTLE_RIGHT");
		std::wstring FLAPS (L"FLAPS");
		std::wstring SPOILERS (L"SPOILERS");
		std::wstring GEARS (L"GEARS");
		std::wstring BRAKES (L"BRAKES");
		
		// Published
		_JOYSTICK_ClassHandle = _RtiAmb->getObjectClassHandle(JOYSTICK);
        _AILERON_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, AILERON);
        _ELEVATOR_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, ELEVATOR);
        _RUDDER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, RUDDER);
        _THROTTLE_LEFT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_LEFT);
        _THROTTLE_RIGHT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_RIGHT);
        _FLAPS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, FLAPS);
        _SPOILERS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, SPOILERS);
        _GEARS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, GEARS);
        _BRAKES_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, BRAKES);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> getAllHandles(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publish and subscribe
void JoystickHandler::publishAndSubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes published
		attr_JOYSTICK.insert(_AILERON_ATTRIBUTE);
		attr_JOYSTICK.insert(_ELEVATOR_ATTRIBUTE);
		attr_JOYSTICK.insert(_RUDDER_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_LEFT_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_RIGHT_ATTRIBUTE);
		attr_JOYSTICK.insert(_FLAPS_ATTRIBUTE);
		attr_JOYSTICK.insert(_SPOILERS_ATTRIBUTE);
		attr_JOYSTICK.insert(_GEARS_ATTRIBUTE);
		attr_JOYSTICK.insert(_BRAKES_ATTRIBUTE);
		
        _RtiAmb->publishObjectClassAttributes(_JOYSTICK_ClassHandle,attr_JOYSTICK);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void JoystickHandler::registerObjectInstances()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Joystick (L"Joystick");
        _JOYSTICK_ObjectHandle = _RtiAmb->registerObjectInstance(_JOYSTICK_ClassHandle,Joystick);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Carry out publications and subscriptions
void JoystickHandler::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unpublishObjectClass(_JOYSTICK_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void JoystickHandler::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Register an Object instance
void JoystickHandler::waitForAllAttributesReceived()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Callback : discover object instance
void JoystickHandler::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void JoystickHandler::pauseInitTrim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void JoystickHandler::pauseInitSim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> pauseInitSim(): Start" << std::endl;
    #endif
	std::wstring SimString (L"Simulating");
    if (_IsCreator) 
    {
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
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickHandler.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

//----------------------------------------------------------------------
// Init phase for Efcs
void JoystickHandler::initialization() 
{
// Parse the init XML file 
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
	
	std::cout << "Side Stick System created with following configuration: " << std::endl;
	std::cout << "Configuration Joystick:  " << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Aileron_port :  " << _Aileron_port << std::endl;
    std::cout << "_Aileron_jnb  :  " << _Aileron_jnb << std::endl;
    std::cout << "_Aileron_sens :  " << _Aileron_sens << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Elevator_port :  " << _Elevator_port << std::endl;
    std::cout << "_Elevator_jnb  :  " << _Elevator_jnb << std::endl;
    std::cout << "_Elevator_sens :  " << _Elevator_sens << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Rudder_port :  " << _Rudder_port << std::endl;
    std::cout << "_Rudder_jnb  :  " << _Rudder_jnb << std::endl;
    std::cout << "_Rudder_sens :  " << _Rudder_sens << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Throttle_Left_port :  " << _Throttle_Left_port << std::endl;
    std::cout << "_Throttle_Left_jnb  :  " << _Throttle_Left_jnb << std::endl;
    std::cout << "_Throttle_Left_sens :  " << _Throttle_Left_sens << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Throttle_Right_port :  " << _Throttle_Right_port << std::endl;
    std::cout << "_Throttle_Right_jnb  :  " << _Throttle_Right_jnb << std::endl;
    std::cout << "_Throttle_Right_sens :  " << _Throttle_Right_sens << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Flaps_Up_port :  " << _Flaps_Up_port << std::endl;
    std::cout << "_Flaps_Up_jnb  :  " << _Flaps_Up_jnb << std::endl;
    std::cout << "" << std::endl;
    std::cout << "_Flaps_Down_port :  " << _Flaps_Down_port << std::endl;
    std::cout << "_Flaps_Down_jnb  :  " << _Flaps_Down_jnb << std::endl;
    std::cout << "" << std::endl;   
    std::cout << "_Spoilers_port :  " << _Spoilers_port << std::endl;
    std::cout << "_Spoilers_jnb  :  " << _Spoilers_jnb << std::endl;
    std::cout << "" << std::endl;     
    std::cout << "_Gears_port :  " << _Gears_port << std::endl;
    std::cout << "_Gears_jnb  :  " << _Gears_jnb << std::endl;
    std::cout << "" << std::endl;    
    std::cout << "_Brakes_port :  " << _Brakes_port << std::endl;
    std::cout << "_Brakes_jnb  :  " << _Brakes_jnb << std::endl;
    std::cout << "" << std::endl;
	
	//std::cout << "Side Stick System Main loop started: " << std::endl;
}

//----------------------------------------------------------------------
// Calculate State values for Efcs
void JoystickHandler::calculateState() 
{
	// No implementation here
}

//----------------------------------------------------------------------
// Calculate Ouput values for Efcs
void JoystickHandler::calculateOutput() 
{
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

	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"Joystick (Elevator) :  " << _Elevator << std::endl;
	std::wcout << L"Joystick (Aileron) :   " << _Aileron << std::endl;
	std::wcout << L"Joystick (Rudder) :    " << _Rudder << std::endl;
	std::wcout << L"Joystick (Throttle Left) :  " << _Throttle_Left << std::endl;
	std::wcout << L"Joystick (Throttle Right) :  " << _Throttle_Right << std::endl;
	std::wcout << L"Joystick (Flaps) :     " << _Flaps << std::endl;
	std::wcout << L"Joystick (Spoilers) :  " << _Spoilers << std::endl;
	std::wcout << L"Joystick (Gears) :     " << _Gears << std::endl;
	std::wcout << L"Joystick (Brakes) :    " << _Brakes << std::endl;
	#endif
}

