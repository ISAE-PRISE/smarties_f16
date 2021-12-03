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

#include "CockpitMainWindow.hh"
#include "ui_CockpitMainWindow.h"
#include <iostream>
#include <QDebug>
#include <QDesktopWidget>

using namespace std;

//----------------------------------------------------------------------
// Constructor 
CockpitMainWindow::CockpitMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CockpitMainWindow)
{
    ui->setupUi(this);
    fcuw=new FCUwindow(ui->FCU_widget);
    ecamw=new ECAMwindow(ui->ECAM_widget);

    QDesktopWidget *desktop = QApplication::desktop();
    QRect rect = desktop->screenGeometry(0);

    if (rect.width() < 1900 || rect.height() < 1000) 
    { // Screen too small -> add a scrollbar

        QWidget *ghostwidget = new QWidget();   // Ghost container widget
        ghostwidget->resize(QSize(1920,1080));

        QScrollArea *sa = new QScrollArea(this);// scrollable zone
            sa->setWidgetResizable( false );
            sa->setAcceptDrops(0);
            sa->setWidget(ghostwidget);
            sa->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
            sa->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
            sa->viewport()->setAutoFillBackground(true);

        ui->centralWidget->setParent(ghostwidget);
        this->setCentralWidget(sa);

    }

}

//----------------------------------------------------------------------
// Destructor 
CockpitMainWindow::~CockpitMainWindow()
{
  delete ui;
}
