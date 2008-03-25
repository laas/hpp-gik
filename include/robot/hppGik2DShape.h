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
#endif
