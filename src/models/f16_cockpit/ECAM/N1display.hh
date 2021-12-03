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

#ifndef __N1_DISPLAY_HH_DEF__
#define __N1_DISPLAY_HH_DEF__

#include <QGraphicsView>
#include <QtSvg/QGraphicsSvgItem>
#include <QtSvg/QSvgGenerator>

//----------------------------------------------------------------------
// Provide a GUI for arrow indicators
//----------------------------------------------------------------------
class N1display : public QGraphicsView 
{

	public:

		N1display( QWidget *parent = 0 );
		~N1display( void );
		void    init( void );
		void    set( double power );
		enum RendererType { Native, OpenGL, Image };

	private:

		void    clean( void );
		void    update_view( void );
		void    switch_color_mode( void );

		double  actual_pwr;         // current power in %
		double  previous_pwr;       // previous power in %

		double red_step;            // red/green step in %

		RendererType m_renderer;

		QGraphicsSvgItem *item_background;
		QGraphicsSvgItem *item_scale;
		QGraphicsSvgItem *item_window_green;
		QGraphicsSvgItem *item_window_red;
		QGraphicsTextItem *item_window_text;
		QGraphicsSvgItem *item_arrow_green;
		QGraphicsSvgItem *item_arrow_red;
		bool color_mode; // 0=red, 1=green
    
};

#endif //  __N1_DISPLAY_HH_DEF__

