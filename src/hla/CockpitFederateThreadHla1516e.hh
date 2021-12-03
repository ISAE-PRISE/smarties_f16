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


#ifndef __COCKPIT_FEDERATE_THREAD_HLA1516e_HH__
#define __COCKPIT_FEDERATE_THREAD_HLA1516e_HH__

#include "CockpitMainWindow.hh"
#include <QThread>


class CockpitFederateThreadHla1516e: public QThread
{
public:
    void run();
    static CockpitMainWindow *window;
    CockpitFederateThreadHla1516e(CockpitMainWindow*);
    
Q_OBJECT
    
signals:
	void gps_update_signal();
	void pfd_update_signal();
	void ecam_update_signal();
};

#endif 
