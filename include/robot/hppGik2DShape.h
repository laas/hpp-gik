#ifndef HPP_GIK_2DSHAPE_H
#define HPP_GIK_2DSHAPE_H

#include <vector>


class ChppGik2DVertex
{
    public:
        double x;
        double y;
};

class ChppGik2DShape
{
    public:
        std::vector<ChppGik2DVertex> vertices;
};


class ChppGikLinkedVertex;
/**
\brief Chained list element
 */
class ChppGikLinkedVertex
{
    public:
        ChppGikLinkedVertex* next;
        ChppGikLinkedVertex* prev;
        ChppGik2DVertex world;
        ChppGik2DVertex local;
        ChppGik2DVertex ntonext;
        ChppGik2DVertex nout;
        double dtonext;
        unsigned int foot;
};

#endif
