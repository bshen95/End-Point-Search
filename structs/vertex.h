#pragma once
#include "point.h"
#include <vector>

namespace polyanya
{

// A point in the polygon mesh.
struct Vertex
{
    Point p;
    // "int" here means an array index.
    std::vector<int> polygons;
    bool is_corner;
    bool is_ambig;
    std::vector<int> obstacle_edge;
    // since all the obstacle edges connect to the vertex itself, we only record the id of connected vertex;
    // obstacle edge : (this,Vertex[i]);
    bool is_turning_vertex = false;
    int cpd_id;
};

}
