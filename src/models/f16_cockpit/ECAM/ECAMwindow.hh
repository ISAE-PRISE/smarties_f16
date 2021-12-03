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

#ifndef __ECAM_WINDOW_HH_DEF__
#define __ECAM_WINDOW_HH_DEF__

#include <QWidget>

namespace Ui 
{
	class ECAMwindow;
}

//----------------------------------------------------------------------
// Provide a GUI for the Small Electronic Centralised Aircraft Monitor
//----------------------------------------------------------------------
class ECAMwindow : public QWidget
{
	Q_OBJECT

	public:
		explicit ECAMwindow(QWidget *parent = 0);
		~ECAMwindow();

		void setN1forEngine1(double N1Engine1) {_N1Engine1 = N1Engine1; }
		void setN1forEngine2(double N1Engine2) {_N1Engine2 = N1Engine2; }
		void setN1forEngine3(double N1Engine3) {_N1Engine3 = N1Engine3; }
		void setN1forEngine4(double N1Engine4) {_N1Engine4 = N1Engine4; }
		void set_ps(float ps) {_ps = ps; }
		void set_mach(float mach) {_mach = mach; }
		void set_thr(float thr) {_thr = thr; }
		void SetNewData();

	private:
	
		Ui::ECAMwindow *ui;
		float _N1Engine1;
		float _N1Engine2;
		float _N1Engine3;
		float _N1Engine4;
		float _max_thrust;
		float _pres_sl;
		float _ps;
		float _mach;
		float _machratio;
		float _pressureratio;
		float _thr;
};

#endif // __ECAM_WINDOW_HH_DEF__
