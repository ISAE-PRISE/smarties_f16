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

#include "ECAMwindow.hh"
#include "ui_ECAMwindow.h"
#include "math.h"

//----------------------------------------------------------------------
// Constructor 
ECAMwindow::ECAMwindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ECAMwindow)
{
    ui->setupUi(this);
    ui->N1left->init();
    ui->N1right->init();
    _N1Engine1 = 0;
    _N1Engine2 = 0;
    _N1Engine3 = 0;
    _N1Engine4 = 0;
    _max_thrust=19000*4.4482216;
    _pres_sl=101325.0;
    _machratio = 1;
    _pressureratio = 1;
}

//----------------------------------------------------------------------
// Destructor 
ECAMwindow::~ECAMwindow()
{
    delete ui;
}

//----------------------------------------------------------------------
// 
void ECAMwindow::SetNewData()
{
	if (_mach < 0.5) _machratio = 1 - 0.8 * _mach;
    else _machratio = 0.7 - 0.2 * _mach;

    _pressureratio = _ps/_pres_sl * pow(1+0.2*_mach*_mach,3.5);

    _N1Engine1 = 100*_thr/(_max_thrust*_machratio*_pressureratio);
    _N1Engine4 = 100*_thr/(_max_thrust*_machratio*_pressureratio);

    ui->N1left->set(_N1Engine1);
    ui->N1right->set(_N1Engine4);
}
