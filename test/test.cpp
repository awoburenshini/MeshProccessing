#include <GeoIO.h>
#include <iostream>

using namespace std;
using GeoIO = GeometryIO;

void main()
{
    std::string polys_path = "D:/Mesh_process/data/system_greengrass_damo_G001133220021011.geojson";
    std::vector<GeoIO::Polygon> polygons;
    GeometryIO::loadGeoFromJSON(polys_path.c_str(), polygons);

    std::string hmap_path = "D:/Mesh_process/data/terrain.tiff";
    HeightMap hmap;
    GeometryIO::loadHeightMapFromTIFF(hmap_path.c_str(), hmap);

    std::cout << hmap.getHeight(MAPLEFTBOTTOM)<< std::endl;
    std::cout << MAPLEFTBOTTOM<< std::endl;
    std::cout << MAPRIGHTTOP<< std::endl;
    std::cout<< "hello" << std::endl;
}