#include <GeoIO.h>
#include <triangulator.h>
#include <iostream>
#include <debugger.h>
#include <CDT.h>
#include <fstream>

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

void debuggerMeshAsPLY(
    const std::string &path,
    const std::vector<std::array<size_t, 3>> &F,
    const std::vector<std::array<double, 2>> &V,
    const bool Inversed)
{

    std::ofstream os(path);
    if (os.fail())
    {
        throw std::runtime_error("Unable to open OBJ file \"" + path + "\"!");
    }

    // ply header
    {
        os << "ply" << std::endl;
        os << "format ascii 1.0" << std::endl;
    }

    // attribute vertice header
    {
        os << "element vertex " << V.size() << std::endl;
        os << "property float x" << std::endl;
        os << "property float y" << std::endl;
        os << "property float z" << std::endl;
    }

    // attribute vertice header
    {
        os << "element face " << F.size() << std::endl;
        os << "property list "
           << "uchar "
           << "int "
           << "vertex_indices" << std::endl;
    }

    // header end
    {
        os << "end_header" << std::endl;
    }

    for (int i = 0; i < V.size(); ++i)
    {
        const auto &vec = V[i];
        os << vec[0] << " " << vec[1] << " " << 0. << std::endl;
    }

    if (Inversed)
    {
        for (const auto &f : F)
        {
            os << 3 << " " << f[0] << " " << f[2] << " " << f[1] << std::endl;
        }
    }
    else
    {
        for (const auto &f : F)
        {
            os << 3 << " " << f[0] << " " << f[1] << " " << f[2] << std::endl;
        }
    }
}





void main()
{
    // GeoIO::Polygon plyg = getPolygon(162);
    GeoIO::Polygon plyg = getPolygon(0);
    polygonNormalize(plyg);
    MD::debuggerPolygon(plyg);

    Triangulator::Diagnostics diag;
    std::vector<std::array<double, 2>> contour;
    for (const auto &loop : plyg)
    {
        contour.reserve(loop.size());
        for (int i = 0 ;i < loop.size() - 1; ++i)
        {
            auto& p = loop[i];
            std::array<double, 2> p_ = {p.x(), p.y()};
            contour.push_back(p_);
        }
    }

    auto triangle = Triangulator::triangulate1(
        contour, diag);

    debuggerMeshAsPLY("../debug_file/a.ply",triangle,contour,false);
    std::cout << "hello" << std::endl;
} 
