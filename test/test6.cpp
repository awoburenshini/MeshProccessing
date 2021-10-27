#include <common.h>
#include <iostream>
#include <CDT.h>
#include <debugger.h>
#include <HeightMap.h>
#include <GeoIO.h>
#include <Sample.h>

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

std::vector<Vector3r> addZComponent(const std::vector<Vector2r> &plane, const HeightMap &hmap)
{

    vector<Vector3r> V;
    V.resize(plane.size(), Vector3r({0, 0, 0}));

    for (int i = 0; i < plane.size(); ++i)
    {
        Real z = hmap.getHeight(plane[i] + MAPLEFTBOTTOM);
        V[i] = Vector3r({plane[i].x(), plane[i].y(), z});
    }

    return std::move(V);
}

int main()
{
    GeoIO::Polygon plyg = getPolygon(51);
    // GeoIO::Polygon plyg = getPolygon(0);
    polygonNormalize(plyg);
    MD::debuggerPolygon(plyg);

    auto triangle = CDT::triangulateLoop(
        plyg[0]);
    plyg[0].resize(plyg[0].size() - 1);
    // MD::debuggerMesh("../debug_file/a.ply", triangle, plyg[0], false);

    auto mesh = CDT::isotropicRemesh(plyg[0], triangle, 0.5);

    MD::debuggerMesh("../debug_file/b.ply", mesh.second, mesh.first, false);

    std::string hmap_path = "../data/terrain.tiff";
    HeightMap hmap;
    GeometryIO::loadHeightMapFromTIFF(hmap_path.c_str(), hmap);


    std::pair<std::vector<Vector3r>, std::vector<Vector3i>> mesh3;

    mesh3.first = addZComponent(mesh.first, hmap);
    mesh3.second = mesh.second;

    GeoIO::writeObj("../debug_file/b.obj",mesh3.first, mesh3.second);

    Real radius = 1.0;
    int n_samples = 300;

    std::vector<Vector3r> samples;

    PoissonDiskOnMesh::sample(mesh3.first,mesh3.second,radius,n_samples,samples);

    GeoIO::writeCsv("../debug_file/sample.csv", samples);

    return 0;
}