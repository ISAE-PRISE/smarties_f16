#----------------------------------------------------------------------
#----------------------------------------------------------------------
#
# SMARTIES 
# Simulation Modules for Aircraft Real-Time Embedded Systems
# 
# Copyright (C) 2018-2021  ISAE-SUPAERO
#
# Author: Jean-Baptiste Chaudron
# email: jean-baptiste.chaudron@isae.fr
#
#----------------------------------------------------------------------
#----------------------------------------------------------------------

PROJECT(SMARTIES C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
SET(PACKAGE_NAME SMARTIES_F16)
SET(DESCRIPTION_SUMMARY "SMARTIES F16 Project .")
SET(TARGET_NAME smarties_f16)

SET(ABI_VERSION_MAJOR "0")
SET(ABI_VERSION_MINOR "0")
SET(ABI_VERSION_PATCH "1")
SET(ABI_VERSION "${ABI_VERSION_MAJOR}.${ABI_VERSION_MINOR}.${ABI_VERSION_PATCH}")
SET(PACKAGE_BUGREPORT "jean-baptiste.chaudron@isae.fr")
SET(CMAKE_CXX_FLAGS "-std=c++11")

#----------------------------------------------------------------------
#                Use CERTI specific CMake modules first  
#        You should set CERTI_HOME env var to appropriate value 
#  or put here the full path to the directory containing FindRTI.cmake 
#             Use CERTI_HOME env var if it is defined 
#----------------------------------------------------------------------
IF ("$ENV{CERTI_HOME}" STRGREATER "")
    FILE(TO_CMAKE_PATH "$ENV{CERTI_HOME}" CERTI_HOME_TEMP)
    SET(CERTI_HOME ${CERTI_HOME_TEMP} CACHE PATH "The CERTI install directory")
    LIST(APPEND CMAKE_MODULE_PATH "${CERTI_HOME}/share/scripts")

# Or try to use CERTI possible installed at the same INSTALL_PREFIX
ELSE ("$ENV{CERTI_HOME}" STRGREATER "")
    LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/scripts")    
ENDIF ("$ENV{CERTI_HOME}" STRGREATER "")
#####
#SET(CMAKE_MODULE_PATH $ENV{CERTI_HOME}/share/scripts)

#----------------------------------------------------------------------
# For configuration of library for root    
OPTION(USE_FULL_RPATH
             "Use the full RPATH" OFF)
             
IF (USE_FULL_RPATH)       
    # see http://www.cmake.org/Wiki/CMake_RPATH_handling#Always_full_RPATH
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    
    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    
    # the RPATH to be used when installing
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    
    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDIF(USE_FULL_RPATH)

#----------------------------------------------------------------------
# You should uncomment the following line if you want VERBOSE build 
#SET(CMAKE_VERBOSE_MAKEFILE ON)

# The preferred (default) behaviour is to build library 
# as shared on all platform 
OPTION(BUILD_SHARED
  "Build libraries as shared library" ON)
IF (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS ON)
ELSE (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS OFF)
ENDIF (BUILD_SHARED)

#----------------------------------------------------------------------
#SET(CMAKE_BUILD_TYPE "Debug")
###
SET(CMAKE_BUILD_TYPE "Release")

#----------------------------------------------------------------------
# Call i.e. FIND RTI   
FIND_PACKAGE(RTI REQUIRED)
INCLUDE_DIRECTORIES(${RTI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${RTI1516e_INCLUDE_DIR})
ADD_DEFINITIONS(${RTI_DEFINITIONS})
MESSAGE(STATUS "* RTI FOUND *")


#----------------------------------------------------------------------
#  Call i.e. FIND Qt5  
SET(QT_USE_QTOPENGL TRUE) 
FIND_PACKAGE(Qt5 COMPONENTS Widgets WebKitWidgets Svg OpenGL QUIET)
IF (Qt5_FOUND)
  MESSAGE(STATUS "* Qt${Qt5_VERSION} Found *" )
  find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
ELSE ()
  MESSAGE(STATUS "* Qt5 NOT FOUND: GUI NOT ACTIVATED *")
ENDIF (Qt5_FOUND)
find_package(GLUT)


#----------------------------------------------------------------------
## Includes Models
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/hla)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/common)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/datatypes)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/configs)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/entity)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_eng)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_efcs)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_fdm)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_cockpit)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_cockpit/ECAM)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_cockpit/FCU)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND)

#----------------------------------------------------------------------
# fcs_entity
SET(EXE_NAME fcs_entity)
SET(SRC_LIST 	${CMAKE_SOURCE_DIR}/src/entity/f16_fcs_entity_main.cc
				${CMAKE_SOURCE_DIR}/src/entity/f16_fcs_entity.cc
				${CMAKE_SOURCE_DIR}/src/common/ReceivingSocketUDP.cc
				${CMAKE_SOURCE_DIR}/src/common/SendingSocketUDP.cc
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/f16_efcs_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/fof_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/llf_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/pid_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/sof_f.cc
				)

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} pthread rt)
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)


#----------------------------------------------------------------------
# Efcs Bridge Component (HLA FEDERATE)
SET(EXE_NAME EfcsBridgeFederateHla1516e)
SET(SRC_LIST 	${CMAKE_SOURCE_DIR}/src/hla/EfcsBridgeFederateHla1516eMain.cc
				${CMAKE_SOURCE_DIR}/src/hla/EfcsBridgeFederateHla1516e.cc
				${CMAKE_SOURCE_DIR}/src/common/ReceivingSocketUDP.cc
				${CMAKE_SOURCE_DIR}/src/common/SendingSocketUDP.cc
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/f16_efcs_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/fof_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/llf_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/pid_f.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_efcs/sof_f.cc
				)

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} pthread rt)
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

#----------------------------------------------------------------------
# Engine Component for DAEP Model (HLA FEDERATE)
SET(EXE_NAME EngineFederateHla1516e)
SET(SRC_LIST 	${CMAKE_SOURCE_DIR}/src/hla/EngineFederateHla1516eMain.cc
				${CMAKE_SOURCE_DIR}/src/hla/EngineFederateHla1516e.cc
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_eng/f16_engine_daep_f.cc
				)

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

#----------------------------------------------------------------------
# F16 Flight dynamic Component (HLA FEDERATE)
SET(EXE_NAME FlightDynamicsFederateHla1516e)
SET(SRC_LIST 	${CMAKE_SOURCE_DIR}/src/hla/FlightDynamicsFederateHla1516eMain.cc
				${CMAKE_SOURCE_DIR}/src/hla/FlightDynamicsFederateHla1516e.cc
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
				${CMAKE_SOURCE_DIR}/src/models/f16_fdm/f16_fdm_f.cc
				)

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

#----------------------------------------------------------------------
# ManagerFederateHla1516e
SET(EXE_NAME ManagerFederateHla1516e)
SET(SRC_LIST 	${CMAKE_SOURCE_DIR}/src/hla/ManagerFederateHla1516eMain.cc
				${CMAKE_SOURCE_DIR}/src/hla/ManagerFederateHla1516e.cc
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
				)

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

#----------------------------------------------------------------------
# Visualization Component
SET(EXE_NAME VisualizationFederateHla1516e)
SET(SRC_LIST ${CMAKE_SOURCE_DIR}/src/hla/VisualizationFederateMainHla1516e.cc 
             ${CMAKE_SOURCE_DIR}/src/hla/VisualizationFederateHla1516e.cc
             ${CMAKE_SOURCE_DIR}/src/common/FlightGearNativeFdm.cc
             ${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} pthread rt ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

#----------------------------------------------------------------------
# Install run script
INSTALL(FILES ${CMAKE_SOURCE_DIR}/scripts/run_demo_f16.sh DESTINATION bin)

#----------------------------------------------------------------------
# F16 Cockpit (HLA FEDERATE)
IF (Qt5_FOUND)               
SET(EXE_NAME CockpitFederateHla1516e)
SET(SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/src/hla/CockpitFederateThreadHla1516e.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/hla/CockpitFederateHla1516e.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/Cockpit.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/hla/CockpitMainHla1516e.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/CockpitMainWindow.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/UnitConversion.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/FCU/FCUwindow.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ECAM/ECAMwindow.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ECAM/N1display.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetPFD.cpp
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetNAV.cpp
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/qfi_PFD.cpp
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/qfi_NAV.cpp
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/LayoutSquare.cpp
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/Joystick.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ConfigurationParametersParser.cc
			 ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/tinyxml2.cpp
			 ${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.cc
			 )

# Qt5 specific functions #
Set(HDR_LIST  	${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/CockpitMainWindow.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/hla/CockpitFederateThreadHla1516e.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/UnitConversion.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/FCU/FCUwindow.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ECAM/ECAMwindow.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetPFD.h
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetNAV.h
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/qfi_PFD.h
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/qfi_NAV.h
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/LayoutSquare.h
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/Joystick.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ConfigurationParametersParser.hh
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/tinyxml2.h
				${CMAKE_SOURCE_DIR}/src/common/MessageBuffer.hh
				)

SET(FORMS_LIST 	${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/CockpitMainWindow.ui
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/FCU/FCUwindow.ui
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ECAM/ECAMwindow.ui
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetNAV.ui
				${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/WidgetPFD.ui)

SET(RESSOURCES_LIST ${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/Ressources.qrc
					${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/FCU/fcu_rc.qrc
					${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/ECAM/ecam_rc.qrc
					${CMAKE_CURRENT_SOURCE_DIR}/src/models/f16_cockpit/PFD_ND/qfi.qrc)

QT5_WRAP_CPP(09_COCKPIT_HEADERS_MOC ${HDR_LIST})
QT5_WRAP_UI(09_COCKPIT_FORMS_HEADERS ${FORMS_LIST})
QT5_ADD_RESOURCES(09_COCKPIT_RESSOURCES_RCC ${RESSOURCES_LIST})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST} ${09_COCKPIT_HEADERS_MOC} ${09_COCKPIT_FORMS_HEADERS} ${09_COCKPIT_RESSOURCES_RCC})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT} Qt5::Widgets Qt5::WebKitWidgets Qt5::Svg)
INSTALL(TARGETS ${EXE_NAME}  RUNTIME DESTINATION bin)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/src/models/f16_cockpit/SideStick1Config.xml DESTINATION bin)
endif (Qt5_FOUND)

if (NOT Qt5_FOUND)
MESSAGE(STATUS "Qt5 not found! Please check your settings - Cockpit federate not configured for compilation")
endif (NOT Qt5_FOUND)
