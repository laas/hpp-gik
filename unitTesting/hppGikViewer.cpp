#include "hppGikViewer.h"
#include <math.h>


ChppGikViewer::ChppGikViewer(bool& outInitOk)
{
    display_name = getenv("DISPLAY");  /* address of the X display.      */
    display = XOpenDisplay(display_name);
    if (display == NULL)
    {
        fprintf(stderr, "Cannot connect to X server '%s'\n",
                display_name);
        outInitOk = false;
        return;
    }

    /* get the geometry of the default screen for our display. */
    screen_num = DefaultScreen(display);
    display_width = DisplayWidth(display, screen_num);
    display_height = DisplayHeight(display, screen_num);

    /* make the new window occupy 1/9 of the screen's size. */
    width = (display_width / 2);
    height = (display_height / 2);

    window_scale = ((double)width)/4.0;
    centerX = height /2;
    centerY = width /2;

    /* the window should be placed at the top-left corner of the screen. */
    win_x = 0;
    win_y = 0;

    /* the window's border shall be 2 pixels wide. */
    win_border_width = 2;

    /* create a simple window, as a direct child of the screen's   */
    /* root window. Use the screen's white color as the background */
    /* color of the window. Place the new window's top-left corner */
    /* at the given 'x,y' coordinates.                             */
    win = XCreateSimpleWindow(display, RootWindow(display, screen_num),
                              win_x, win_y, width, height, win_border_width,
                              BlackPixel(display, screen_num),
                              WhitePixel(display, screen_num));

    /* these variables are used to specify various attributes for the GC. */
    /* initial values for the GC. */
    //XGCValues values = CapButt | JoinBevel;
    XGCValues values;
    values.join_style = JoinBevel;
    values.cap_style = CapButt;
    /* which values in 'values' to check when creating the GC. */
    unsigned long valuemask = GCCapStyle | GCJoinStyle;

    /* create a new graphical context. */
    gc = XCreateGC(display, win, valuemask, &values);
    if (gc < 0)
    {
        fprintf(stderr, "XCreateGC: \n");
        outInitOk = false;
    }

    /* subscribe to the given set of event types. */
    XSelectInput(display, win, KeyPressMask);// ExposureMask | KeyPressMask | ButtonPressMask |  Button1MotionMask | Button2MotionMask | StructureNotifyMask);
    
    outInitOk = true;
}

void ChppGikViewer::showframe()
{
    /* make the window actually appear on the screen. */
    XMapWindow(display, win);

    /* flush all pending requests to the X server, and wait until */
    /* they are processed by the X server.                        */
    XSync(display, False);
}

void ChppGikViewer::erase()
{
    XClearWindow(display, win);
}


void ChppGikViewer::flush()
{
    XFlush(display);
    XSync(display, False);
}


void ChppGikViewer::draw2DShape(const ChppGik2DShape& inShape, double posX, double posY, double inOrientation)
{
    //transform the shape in real world
    attShape.vertices.clear();
    double co = cos(inOrientation);
    double si = sin(inOrientation);

    for (unsigned int i=0;i<inShape.vertices.size();i++)
    {
        attVertex.x = posX + co * inShape.vertices[i].x - si * inShape.vertices[i].y ;
        attVertex.y = posY + si * inShape.vertices[i].x + co * inShape.vertices[i].y ;
        attShape.vertices.push_back(attVertex);
    }

    if (!(attShape.vertices.empty()))
    {
        attShape.vertices.push_back(attShape.vertices[0]);
    }

    //express vertices in pixels
    attXs.clear();
    attYs.clear();
    int X, Y;
    for (unsigned int i=0;i<attShape.vertices.size();i++)
    {
        equivalentPixel( attShape.vertices[i].x, attShape.vertices[i].y, X, Y );
        attXs.push_back(X);
        attYs.push_back(Y);
    }

    for (unsigned int i=0;i<attXs.size()-1;i++)
    {
        XDrawLine(display, win, gc, attXs[i], attYs[i], attXs[i+1], attYs[i+1]);
    }
}

void ChppGikViewer::equivalentPixel(double inX, double inY, int& pX, int& pY)
{
    pY = centerX - (int)floor(inX*window_scale);
    pX = centerY - (int)floor(inY*window_scale);
}




KeySym* ChppGikViewer::nextKey( )
{
    int done = 0;
    XNextEvent(display, &attEvent);
    if (attEvent.type == KeyPress)
    {
        /* translate the key code to a key symbol. */
        attKeySymbol = XKeycodeToKeysym(display, attEvent.xkey.keycode, 0);
        return &attKeySymbol;
    }
    return 0;
}

ChppGikViewer::~ChppGikViewer( )
{
    /* free the GCs. */
    XFreeGC(display, gc);
    /* close the connection to the X server. */
    XCloseDisplay(display);
}
