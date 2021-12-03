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

#include <QApplication>
#include <QDesktopWidget>
#include "CockpitMainWindow.hh"
#include "CockpitFederateThreadHla1516e.hh"


//----------------------------------------------------------------------
// Instantiate the CockpitFederateThread and CockpitMainWindow,
// then link them (Qt signal/slot) to display information from the simulation.
//----------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    CockpitMainWindow w;
    CockpitMainWindow *win;
    win=&w;


    // screenGeometry(0) is full screen
    QDesktopWidget *desktop = QApplication::desktop();
    //QRect rect = desktop->screenGeometry(0);
    //w.move(rect.topLeft());
    
    w.show();

    //if (rect.width() < 1900 || rect.height() < 1000) w.show();
    //else w.showFullScreen();
    
	CockpitFederateThreadHla1516e thread(&w);
	CockpitFederateThreadHla1516e* th;
	th=&thread;
	CockpitMainWindow::connect(th, SIGNAL(gps_update_signal()), win, SLOT(new_data_for_GPS()));
	CockpitMainWindow::connect(th, SIGNAL(pfd_update_signal()), win, SLOT(new_data_for_PFD()));
	CockpitMainWindow::connect(th, SIGNAL(ecam_update_signal()), win, SLOT(new_data_for_ECAM()));
	thread.start();
	
    return a.exec();
}
