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

#ifndef __FCU_WINDOW_HH_DEF__
#define __FCU_WINDOW_HH_DEF__

#include <QWidget>
#include <QIcon>

namespace Ui {
class FCUwindow;
}

//----------------------------------------------------------------------
// Provide a GUI for the Flight Control Unit
//----------------------------------------------------------------------
class FCUwindow : public QWidget
{
    Q_OBJECT
    
	public:
		explicit FCUwindow(QWidget *parent = 0);
		~FCUwindow();

		bool Get_Autopilot_AP()     {return Autopilot_AP;}
		bool Get_Autopilot_AP2()     {return Autopilot_AP2;}
		bool Get_Autopilot_athr()   {return Autopilot_athr;}
		
		int Get_Autopilot_spd()     {return Autopilot_spd;}
		int Get_Autopilot_hdg()     {return Autopilot_hdg;}
		int Get_Autopilot_alt()     {return Autopilot_alt;}
		int Get_Autopilot_vs()      {return Autopilot_vs;}
		
		void set_Alti(double input);
		void set_Mach_from_spd();
		void set_spd_from_Mach();

		private slots:

		void on_dial_HDG_valueChanged(int value);

		void on_dial_SPD_valueChanged(int value);

		void on_slider_ALT_SELECT_valueChanged(int value);

		void on_dial_ALT_valueChanged(int value);

		void on_dial_VS_valueChanged(int value);

		void on_button_athr_clicked();

		void on_button_loc_clicked();

		void on_button_ap1_clicked();

		void on_button_ap2_clicked();

		void on_button_exped_clicked();

		void on_button_appr_clicked();

		void on_dial_switch_SPD_MACH_valueChanged(int value);

	private:
		Ui::FCUwindow *ui;

		bool Autopilot_AP;
		bool Autopilot_AP2;
		bool Autopilot_athr;

		int Autopilot_alt; // in ft
		int Autopilot_hdg; // in deg
		double Autopilot_spd; // in kt
		int Autopilot_vs;  // in ftps
		double Autopilot_mach;
		double Alti;

		void initialize();
		void create_icons();
		void update_all_disp();
		void update_SPD_disp();
		void update_HDG_disp();
		void update_ALT_disp();
		void update_VS_disp();
		void update_ATHR_disp();
		void update_AP1_disp();
		void update_AP2_disp();
		void update_APPR_disp();
		void update_LOC_disp();
		void update_EXPED_disp();
		void update_AP_state();

		static const int Min_alt = 0;
		static const int Max_alt = 50000;

		static const int Min_vs = -6000;
		static const int Max_vs = 6000;

		static const int Min_mach = 10;  //*100
		static const int Max_mach = 100;  //*100 - It was 80 Before

		bool Autopilot_ap1;
		bool Autopilot_ap2;
		bool Autopilot_exped;
		bool Autopilot_appr;
		bool Autopilot_loc;

		int HDG_dial_value;
		int SPD_dial_value;
		int ALT_dial_value;
		int ALT_select_sensitivity;
		int VS_dial_value;
		int SPD_MACH_dial_value;

		QIcon icon_loc_on;
		QIcon icon_loc_off;
		QIcon icon_ap1_on;
		QIcon icon_ap1_off;
		QIcon icon_ap2_on;
		QIcon icon_ap2_off;
		QIcon icon_athr_on;
		QIcon icon_athr_off;
		QIcon icon_exped_on;
		QIcon icon_exped_off;
		QIcon icon_appr_on;
		QIcon icon_appr_off;
};

#endif // __FCU_WINDOW_HH_DEF__
