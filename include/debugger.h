#pragma once
#include <GeoIO.h>
#include <string>

class Mdebugger
{
public:
    static void debuggerPolygon(
        const GeometryIO::Polygon &polygon,
        const std::string &path = "../debug_file/debug_polygon.ply");

    static void debuggerMesh(
        const std::string &path,
        const std::vector<Vector3i> &F,
        const std::vector<Vector2r> &V,
        const bool Inversed = false
    );
};