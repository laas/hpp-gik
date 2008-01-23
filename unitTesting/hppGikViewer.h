#ifndef HPP_GIK_VIEWER_H
#define HPP_GIK_VIEWER_H

#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>		/* getenv(), etc. */
#include <unistd.h>		/* sleep(), etc.  */
#include "robot/hppGik2DShape.h"

class ChppGikViewer
{
    public :
        ChppGikViewer( bool& outInitOk );
        
        void showframe( );
        
        void erase();
        
        void draw2DShape(const ChppGik2DShape& inShape, double posX, double posY, double inOrientation);
        
        void flush();
        
        KeySym* nextKey();

        
        ~ChppGikViewer( );
        
    private:
        
        Display* display;
        Window win;			/* pointer to the newly created window.      */
        GC gc;
        int screen_num;		/* number of screen to place the window on.  */
        
        unsigned int display_width,
        display_height;	/* height and width of the X display.        */
        unsigned int width, height;	/* height and width for the new window.      */
        unsigned int win_x, win_y;	/* location of the window's top-left corner. */
        unsigned int win_border_width; /* width of window's border.                */
        char *display_name;
        double window_scale;
        int centerX, centerY;
        
        XEvent attEvent;
        KeySym attKeySymbol;
        
        ChppGik2DShape attShape;
        ChppGik2DVertex attVertex;
        
        std::vector<int> attXs;
        std::vector<int> attYs;
        
        void equivalentPixel(double inX, double inY, int& pX, int& pY);
};

#endif
