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

#ifndef __ENGINE_FEDERATE_HLA1516E_HH_DEF__
#define __ENGINE_FEDERATE_HLA1516E_HH_DEF__

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
#include <assert.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

// RTI includes
#include <RTI/certiRTI1516.h>
#include <RTI/RTI1516.h>
#include <RTI/Enums.h>
#include <RTI/NullFederateAmbassador.h>
#include <RTI/RTI1516fedTime.h>
#include <RTI/LogicalTime.h>

// Specific includes
#include <MessageBuffer.hh>

// Models
#include <f16_engine_daep_f.hh>

// Datatypes
#include <hla_data_types_f.h>

#define DEBUG_ENGINE_FEDERATE_1516E

class EngineFederateHla1516e : public rti1516e::NullFederateAmbassador 
{
	// Private  elements
	private:
	
		//internal model
		f16_engine_daep_f _f16_engine_daep;
		
		// data input/ouput
		efcs_data_hla_t _efcs_data_in;
		config_data_hla_t _config_data_in;
		sim_time_data_hla_t _sim_time_data_in;
		env_data_hla_t    _env_data_in;
		fdm_data_hla_t _fdm_data_in;
		engine_data_hla_t _eng_data_daep_out;
		engine_data_hla_t _eng_data_nasa_in;

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
		
		bool _IsModelTrimmed;
		
		//---------------------------------------------------------------
		// SUBSCRIBED
		//---------------------------------------------------------------
		// Simulation time
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullSimulationTimeModel;
		rti1516e::AttributeHandle _AttributeHandleFullSimulationTimeModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullSimulationTimeModelData;
		bool _DiscovObjectInstanceFullSimulationTimeModelData;
		bool _NewAttributeFullSimulationTimeModelData;
		rti1516e::AttributeHandleSet _attrFullSimulationTimeModelData;
		
		// FDM
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullFlightDynamicsModel;
		rti1516e::AttributeHandle _AttributeHandleFullFlightDynamicsModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullFlightDynamicsModelData;
		bool _DiscovObjectInstanceFullFlightDynamicsModelData;
		bool _NewAttributeFullFlightDynamicsModelData;
		rti1516e::AttributeHandleSet _attrFullFlightDynamicsModelData;
		
		// Env
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullEnvironmentModel;
		rti1516e::AttributeHandle _AttributeHandleFullEnvironmentModelData;  
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullEnvironmentModelData;
		bool _DiscovObjectInstanceFullEnvironmentModelData;
		bool _NewAttributeFullEnvironmentModelData;
		rti1516e::AttributeHandleSet _attrFullEnvironmentModelData;
			
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
		
		//---------------------------------------------------------------
		// PUBLISHED + SUBSCRIBED
		//---------------------------------------------------------------
		// Engine 
		rti1516e::ObjectClassHandle _ObjectClassHandleClassFullEngineModel;
		rti1516e::AttributeHandle _AttributeHandleFullEngineModelData; 
		// Engine Subscribed (Nasa Model) 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullEngineModelData_nasa;
		rti1516e::AttributeHandleSet _attrFullEngineModelData_nasa;
		bool _DiscovObjectInstanceFullEngineModelData_nasa;
		bool _NewAttributeFullEngineModelData_nasa;
		// Published engine model (DAEP)
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleFullEngineModelData_daep;
		rti1516e::AttributeHandleSet _attrFullEngineModelData_daep;
		
		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		MessageBuffer _InputMessagebuffer;
		
		// For simulation versus realtime measurements
		timespec _TimeStamp, _TimeStamp_old;
		timespec _ExecutionTime;
		double _TotalRealTimeMs;
				
    // Public elements
	public:
	
		EngineFederateHla1516e ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~EngineFederateHla1516e();
		
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
		void calculateState();
        void calculateOutput();
		void sendHlaAttributes(RTI1516fedTime UpdateTime);
		void sendInitialHlaAttributes();
        void sendInitialHlaAttributesTrim();
		void run();  
		void runOneStep();                                       
		void pauseInitTrim();
		void pauseInitSim();
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


#endif //__FLIGHT_DYNAMICS_FEDERATE_HLA1516E_HH_DEF__
