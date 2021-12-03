//----------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//----------------------------------------------------------------------

#ifndef __COCKPIT_MAIN_WINDOW_HH__
#define __COCKPIT_MAIN_WINDOW_HH__

#include <QMainWindow>

#include "FCU/FCUwindow.hh"
#include "ECAM/ECAMwindow.hh"
#include "PFD_ND/WidgetNAV.h"
#include "PFD_ND/WidgetPFD.h"
#include "ui_CockpitMainWindow.h"

namespace Ui 
{
	class CockpitMainWindow;
}

//----------------------------------------------------------------------
// Main Window of the Cockpit. This class handle the GUI 
// of the Cockpit Instruments 
//----------------------------------------------------------------------
class CockpitMainWindow : public QMainWindow
{
    Q_OBJECT

	public:
	
		explicit CockpitMainWindow(QWidget *parent = 0);
		~CockpitMainWindow();

		FCUwindow *fcuw;
		ECAMwindow *ecamw;

		bool Get_Autopilot_AP()     {return fcuw->Get_Autopilot_AP();}
		bool Get_Autopilot_AP2()     {return fcuw->Get_Autopilot_AP2();}
		bool Get_Autopilot_athr()   {return fcuw->Get_Autopilot_athr();}
		int Get_Autopilot_spd()     {return fcuw->Get_Autopilot_spd();}
		int Get_Autopilot_hdg()     {return fcuw->Get_Autopilot_hdg();}
		int Get_Autopilot_alt()     {return fcuw->Get_Autopilot_alt();}
		int Get_Autopilot_vs()      {return fcuw->Get_Autopilot_vs();}

		void set_PFD_pitch(double pit) {ui->widgetPFD->setPitch(pit); }
		void set_PFD_roll(double rol) {ui->widgetPFD->setRoll(rol); }
		void set_PFD_heading(double hed) {ui->widgetPFD->setHeading(hed);}
		void set_PFD_altitude(double alt) {ui->widgetPFD->setAltitude(alt); } //used to update fcu alti for mach calculation
		void set_PFD_airspeed(double tas) {ui->widgetPFD->setAirspeed(tas); }
		void set_PFD_FlightPath(double alpha, double beta) {ui->widgetPFD->setFlightPathMarker( alpha, beta ); }
		void set_PFD_mach(double mch) {ui->widgetPFD->setMachNo(mch); }
		void set_PFD_dynamic_pressure(double dyn) {ui->widgetPFD->setPressure(dyn); }
		
		void set_ND_heading(double hed) {ui->widgetNAV->setHeading(hed);}
		void set_ND_headingBug(double hedbug) {ui->widgetNAV->setHeadingBug(hedbug);}

		void setN1Engine1ECAM(double N1Engine1) {ecamw->setN1forEngine1(N1Engine1); }
		void setN1Engine2ECAM(double N1Engine2) {ecamw->setN1forEngine2(N1Engine2); }
		void setN1Engine3ECAM(double N1Engine3) {ecamw->setN1forEngine3(N1Engine3); }
		void setN1Engine4ECAM(double N1Engine4) {ecamw->setN1forEngine4(N1Engine4); }
		
		void set_ps(float ps) {ecamw->set_ps(ps); }
		void set_mach(float mach) {ecamw->set_mach(mach); }
		void set_thr(float thr) {ecamw->set_thr(thr); }


		
	private slots:

		void new_data_for_GPS() {ui->widgetNAV->update();}
		void new_data_for_PFD() {ui->widgetPFD->update();}
		void new_data_for_ECAM() {ecamw->SetNewData();}

	private:

		Ui::CockpitMainWindow *ui;

};

#endif // __COCKPIT_MAIN_WINDOW_HH__
