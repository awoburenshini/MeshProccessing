#include <GeoIO.h>
#include <iostream>
#include <debugger.h>
#include <CDT.h>
#include <fstream>
#include <isotropicRemesher.h>

using namespace std;
using GeoIO = GeometryIO;
using MD = Mdebugger;

GeoIO::Polygon getPolygon(int i)
{
    std::string polys_path = "D:/Mesh_process/data/system_greengrass_damo_G001133220021011.geojson";
    std::vector<GeoIO::Polygon> polygons;
    GeometryIO::loadGeoFromJSON(polys_path.c_str(), polygons);
    for (int pl = 0; pl < polygons.size(); ++pl)
    {
        auto &polygon = polygons[pl];
        if (polygon.size() != 1)
        {
            continue;
        }

        std::cout << pl << " number of vertex" << polygon[0].size() << " " << std::endl;
    }
    return polygons[i];
}

void polygonNormalize(GeoIO::Polygon &polygon)
{
    for (auto &loop : polygon)
    {
        for (Vector2r &p : loop)
        {
            p -= MAPLEFTBOTTOM;
        }
    }
}

void polygonUnNormalize(GeoIO::Polygon &polygon)
{

    for (auto &loop : polygon)
    {
        for (Vector2r &p : loop)
        {
            p += MAPLEFTBOTTOM;
        }
    }
}

void main()
{

    GeoIO::Polygon plyg = getPolygon(0);
    // GeoIO::Polygon plyg = getPolygon(0);
    polygonNormalize(plyg);
    MD::debuggerPolygon(plyg);

    auto triangle = CDT::triangulateLoop(
        plyg[0]);
    plyg[0].resize(plyg[0].size() - 1);
    MD::debuggerMesh("../debug_file/a.ply", triangle, plyg[0], false);

    IsotropicRemesher remesh(plyg[0], triangle, 0.5);
    remesh.init();
    remesh.run();
    auto mesh = remesh.getMesh();

    IsotropicRemesher remesh2(mesh.first, mesh.second,0.5);
    remesh2.init();
    remesh2.run();
    remesh.flip_to_delauney();



    mesh = remesh2.getMesh();
    MD::debuggerMesh("../debug_file/b.ply", mesh.second, mesh.first, false);

    std::cout << "hello" << std::endl;
}
