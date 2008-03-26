/*
 * Copyright (c) 2008
 *      IS/AIST-ST2I/CNRS Joint Japanese-French Robotics Laboratory (JRL).
 * All rights reserved.
 */

#include <string>

#include "hppGikTest.h"
#include "hppGikBasicExample.h"


int main(int argc, char *argv[])
{
    ChppGikTest gikTest;

    //ChppGikBasicExample gikExample;
    //gikExample.planExample();

    //gikTest.locoPlannerTest();

#if defined(HAVE_X11_XLIB_H) && defined(HAVE_X11_XUTIL_H)
    //gikTest.locoPlannerTestInteractive();
#endif

    gikTest.interprete();
}

