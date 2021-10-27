#pragma once
#include <common.h>
#include <string>
#include <vector>
#include <HeightMap.h>



class GeometryIO
{
public:
    typedef std::vector<Vector2r> Loop;
    typedef std::vector<Loop> Polygon;

    static void loadGeoFromJSON(const char *path, std::vector<Polygon>& Polygons);
    static void loadHeightMapFromTIFF(const char *path, HeightMap& heightMap);

    static void writeObj(const std::string &path, const std::vector<Vector3r>& V, const std::vector<Vector3i>& F);
    static void writeCsv(const std::string &path, const std::vector<Vector3r>& V);
};
