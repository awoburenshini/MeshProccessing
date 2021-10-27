#include <common.h>
#include <iostream>
#include <CDT.h>
#include <debugger.h>
#include <HeightMap.h>
#include <GeoIO.h>
#include <Sample.h>

#pragma execution_character_set("utf-8")
using namespace std;
using GeoIO = GeometryIO;
using MD = Mdebugger;

GeoIO::Polygon getPolygon(int i)
{
    std::string polys_path = "./data/system_greengrass_damo_G001133220021011.geojson";
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
    int polygon_ind = 0;
    std::cout << "Enter the index you want to process(1-200)" << std::endl;
    std::cin >> polygon_ind;

    Real mesh_density = 1.0;
    std::cout << "Enter the mesh density( > 0)" << std::endl;
    std::cin >> mesh_density;
    if (mesh_density < 0.)
    {
        std::cout << "mesh density < 0,****" << std::endl;
        return -1;
    }

    Real radius = 1.0;
    std::cout << "Enter the sampling radius( >0 )" << std::endl;
    std::cin >> radius;
    if (radius < 0.)
    {
        std::cout << "sampling radius < 0, ****" << std::endl;
        return -1;
    }

    std::string polygon_path = "./debug_file/input_polygon"+to_string(polygon_ind)+".ply";
    std::string ear_clipping_path = "./debug_file/ear_clipping"+to_string(polygon_ind)+".ply";
    std::string remesh_path = "./debug_file/isotropic_remesh"+to_string(polygon_ind)+".ply";
    std::string terran_path = "./debug_file/terran"+to_string(polygon_ind)+".obj";
    std::string sample_path = "./debug_file/sample"+to_string(polygon_ind)+".csv";

    GeoIO::Polygon plyg = getPolygon(polygon_ind);
    polygonNormalize(plyg);
    MD::debuggerPolygon(plyg, polygon_path);

    auto triangle = CDT::triangulateLoop(
        plyg[0]);
    plyg[0].resize(plyg[0].size() - 1);
    MD::debuggerMesh(ear_clipping_path, triangle, plyg[0], false);

    auto mesh = CDT::isotropicRemesh(plyg[0], triangle, mesh_density);

    MD::debuggerMesh(remesh_path, mesh.second, mesh.first, false);

    std::string hmap_path = "./data/terrain.tiff";
    HeightMap hmap;
    GeometryIO::loadHeightMapFromTIFF(hmap_path.c_str(), hmap);

    std::pair<std::vector<Vector3r>, std::vector<Vector3i>> mesh3;

    mesh3.first = addZComponent(mesh.first, hmap);
    mesh3.second = mesh.second;

    GeoIO::writeObj(terran_path, mesh3.first, mesh3.second);

    std::vector<Vector3r> samples;

    PoissonDiskOnMesh::sample(mesh3.first, mesh3.second, radius, 100, samples);

    GeoIO::writeCsv(sample_path, samples);

    return 0;
}