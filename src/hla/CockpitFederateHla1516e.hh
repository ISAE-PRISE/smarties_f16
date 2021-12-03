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

#ifndef __COCKPIT_FEDERATE_HLA1516E_HH_DEF__
#define __COCKPIT_FEDERATE_HLA1516E_HH_DEF__

#define DEBUG_COCKPIT_FED

// System includes
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>
#include <iterator>
#include <assert.h>
#include <time.h>
#include <limits>

// udp sockets
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

// RTI includes
#include <RTI/RTI1516.h>
#include <RTI/Enums.h>
#include <RTI/NullFederateAmbassador.h>
#include <RTI/RTI1516fedTime.h>
#include <RTI/LogicalTime.h>
#include <RTI/encoding/HLAfixedRecord.h>
#include <RTI/encoding/HLAfixedArray.h>
#include <RTI/encoding/BasicDataElements.h>

// Specific includes
#include <MessageBuffer.hh>
#include "Cockpit.hh"
#include "Joystick.hh"
#include "tinyxml2.h"
#include <f16_data_types_f.h>

//json handling
#include <json.hh>

// Datatypes
#include <hla_data_types_f.h>

using namespace rti1516e;

#define COCKPIT_DEG2RAD           0.017453293
#define COCKPIT_RAD2DEG           57.29577951

class CockpitFederateHla1516e : public rti1516e::NullFederateAmbassador 
{
	// Private  elements
	private:
	
		// Internal model
		Cockpit _Cockpit;
		// Internal model for Joystick
		Joystick _Joystick;
		
		// data input/ouput
		efcs_data_hla_t _efcs_data_in;
		sensors_data_hla_t _sensors_data_in;
		config_data_hla_t _config_data_in;
		sim_time_data_hla_t _sim_time_data_in;
		cockpit_data_hla_t _cockpit_data_out;
		
		// JSON socket for data display
		int _sock_curve;
		struct sockaddr_in _curve_servaddr; 
		struct sockaddr_in _curve_remaddr;
		socklen_t _curve_addrlen;
		int _length;
		nlohmann::json _json_element;
		std::string _json_string_dump;
		
		// This is to be sure that the object is only assigned once!
		// i.e. one controller per flightdynamics
		// this is a woraround and it has to change
		bool _ObjectControlCommandAffected; 
		
		double *Inputs;
		
		// Local variables
		double _Aileron, 
		_Elevator, 
		_Rudder, 
		_Throttle_Left, 
		_Throttle_Right, 
		_Flaps, 
		_Spoilers, 
		_Gears, 
		_Brakes,
		_Stabilizer,
		_ParkingBrakes;

		// Port numbers of the joystick (called input in config file) 
		int _Aileron_port, 
		_Elevator_port, 
		_Rudder_port, 
		_Throttle_Left_port, 
		_Throttle_Right_port, 
		_Flaps_Up_port, 
		_Flaps_Down_port, 
		_Spoilers_port, 
		_Gears_port, 
		_Brakes_port;

		// Joystick numbers (called port in config file)
		int _Aileron_jnb, 
		_Elevator_jnb, 
		_Rudder_jnb, 
		_Throttle_Left_jnb, 
		_Throttle_Right_jnb,  
		_Flaps_Up_jnb, 
		_Flaps_Down_jnb, 
		_Spoilers_jnb, 
		_Gears_jnb, 
		_Brakes_jnb;

		// Joystick sensitivity
		double _Aileron_sens, 
		_Elevator_sens, 
		_Rudder_sens, 
		_Throttle_Left_sens, 
		_Throttle_Right_sens;

		// Booleans for buttons handling
		bool _Flaps_Up_released, 
		_Flaps_Down_released, 
		_Spoilers_released, 
		_Gears_released, 
		_Brakes_released;
		
		// DataTypes to transmit the data
		
		
		// DataTypes to receive data from others
	
		rti1516e::RTIambassador*    _RtiAmb;
		rti1516e::FederateHandle _FedHandle;
	
	    std::wstring          _FederateName;
        std::wstring          _FederationName;
        std::wstring          _FomFileName;	    
	    std::wstring          _TrimSyncPointName;
		std::wstring          _SimSyncPointName;
	    
	    rti1516e::FederateHandle _FederateHandle ;
	
		rti1516e::VariableLengthData _MyTag;
		rti1516e::VariableLengthData _MySyncTag;
		
		bool _IsTimeReg;
		bool _IsTimeConst;
		bool _IsTimeAdvanceGrant;
		bool _IsOutMesTimespamped;

		RTI1516fedTime _TimeStep;   
		RTI1516fedTime _Lookahead;   
		RTI1516fedTime _RestOfTimeStep; 			
		RTI1516fedTime _LocalTime;
		RTI1516fedTime _SimulationEndTime;

		bool _SyncRegSuccess;
		bool _SyncRegFailed;
		bool _InPause;
		bool _IsCreator;
		bool _IsSyncAnnonced;
		
		bool _OneAircraftConnected;
		std::wstring _ConnectedAircraftStringId;
		
		// Message Buffers
		MessageBuffer _OutputMessagebuffer;
		MessageBuffer _InputMessagebuffer;
		
		// Handles and Flags for subscribed datas
		// Simulation time
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullSimulationTimeModel;
		rti1516e::AttributeHandle _AttributeHandleFullSimulationTimeModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullSimulationTimeModelData;
		bool _DiscovObjectInstanceFullSimulationTimeModelData;
		bool _NewAttributeFullSimulationTimeModelData;
		rti1516e::AttributeHandleSet _attrFullSimulationTimeModelData;
		
		// Efcs
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullEfcsModel;
		rti1516e::AttributeHandle _AttributeHandleFullEfcsModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullEfcsModelData;
		bool _DiscovObjectInstanceFullEfcsModelData;
		bool _NewAttributeFullEfcsModelData;
		rti1516e::AttributeHandleSet _attrFullEfcsModelData;
		
		// Configuration data
		rti1516e::ObjectClassHandle _ObjectClassFullConfiguration; 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullConfiguration;
		rti1516e::AttributeHandle _AttributeHandleFullConfigurationData; 
		bool _DiscovObjectInstanceFullConfigurationData;
		bool _NewAttributeFullConfigurationData;
		rti1516e::AttributeHandleSet _attrFullConfigurationData; 
		
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullSensorModel;
		rti1516e::AttributeHandle _AttributeHandleFullSensorModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullSensorModelData;
		bool _DiscovObjectInstanceFullSensorModelData;
		bool _NewAttributeFullSensorModelData;
		rti1516e::AttributeHandleSet _attrFullSensorModelData;
		
		// Handles and Flags for published datas
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullCockpitModel;
		rti1516e::AttributeHandle _AttributeHandleFullCockpitModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullCockpitModelData;
		rti1516e::AttributeHandleSet _attrFullCockpitModelData;

				
    // Public elements
	public:
	
		CockpitFederateHla1516e ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~CockpitFederateHla1516e();
		
		// wstring handling
		static const std::string getString(const std::wstring& wstr) {
			return std::string(wstr.begin(),wstr.end());
		};

		static const std::wstring getWString(const char* cstr) {
			std::wstringstream ss;
			ss << cstr;
			return std::wstring(ss.str());
		};
		
		// Federation Management
		void createFederationExecution();
		void destroyFederationExecution();
		void joinFederationExecution();
		void resignFederationExecution();
		rti1516e::FederateHandle getFederateHandle() const ;
		void getAllHandles();
		void publishAndSubscribe();
		void registerObjectInstances();
		void unpublishAndUnsubscribe();
		void waitForAllObjectDiscovered();
		void waitForAllAttributesReceived();
		void setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                         , RTI1516fedTime Lookahead
                                         , RTI1516fedTime LocalTime
                                         , RTI1516fedTime SimulationEndTime
                                         );
		void initialization();
		void initHlaStructs();
		void calculateState();
        void calculateOutput();
        void sendUpdateAttributesTrim();
        void sendUpdateAttributes(RTI1516fedTime UpdateTime);
		void run();    
		void runOneStep();                                      
		void pauseInitTrim();
		void pauseInitSim();
		void pause();
		bool getEndOfSimulation();
		
		// Callback : discover object instance
		void discoverObjectInstance (rti1516e::ObjectInstanceHandle theObject,
		                             rti1516e::ObjectClassHandle theObjectClass,
		                             std::wstring const &theObjectInstanceName)
                                     throw (rti1516e::FederateInternalError);

		// Callback : reflect attribute values without time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::SupplementalReflectInfo theReflectInfo)
							  throw (rti1516e::FederateInternalError) ;

		// Callback : reflect attribute values with time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::LogicalTime const &theTime,
									rti1516e::OrderType receivedOrdering,
									rti1516e::MessageRetractionHandle theHandle,
									rti1516e::SupplementalReflectInfo theReflectInfo
									)
							  throw ( rti1516e::FederateInternalError) ;


		// HLA specific methods : TIME MANAGEMENT 
		// Callback : timeRegulationEnabled
		/*virtual void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeRegulationWasPending,
			rti1516e::FederateInternalError) ;*/
		void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;

		// Callback : timeConstrainedEnabled
		/*virtual void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeConstrainedWasPending,
			rti1516e::FederateInternalError) ;*/
		void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
		
		// Callback : timeAdvanceGrant
		/*virtual void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::JoinedFederateIsNotInTimeAdvancingState,
			rti1516e::FederateInternalError) ;*/
		void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
			
		void enableTimeRegulation();
		void enableTimeConstrained();
		void enableAsynchronousDelivery();
		void disableTimeRegulation();
		void disableTimeConstrained();
		void disableAsynchronousDelivery();
		void timeAdvanceRequest(RTI1516fedTime NextLogicalTime);
		void nextEventRequest(RTI1516fedTime NextLogicalTime);
		void timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime);
		void nextEventAvailable(RTI1516fedTime NextLogicalTime);

		// HLA specific methods : SYNCHRONISATION 
		// Callback : synchronizationPointRegistrationSucceeded
		void synchronizationPointRegistrationSucceeded(std::wstring const &label )
		throw (rti1516e::FederateInternalError) ;

		// Callback : synchronizationPointRegistrationFailed
		void synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
		throw (rti1516e::FederateInternalError) ;

		// Callback : announceSynchronizationPoint
		void announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag )
		throw (rti1516e::FederateInternalError) ;

		// Callback : federationSynchronized
		virtual void federationSynchronized( std::wstring const &label, rti1516e::FederateHandleSet const& failedToSyncSet)
			throw( rti1516e::FederateInternalError );
		
};
#endif 
