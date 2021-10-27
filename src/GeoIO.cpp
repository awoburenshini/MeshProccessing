#include <GeoIO.h>

#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <tinytiffreader.h>

#include <fstream>
#include <sstream>

using namespace std;
using namespace rapidjson;

static void readFile(const char *fileName, std::string &result)
{
    std::ifstream is(fileName);

    std::ostringstream sin;

    sin << is.rdbuf();

    result = sin.str();

    is.close();
    is.clear();
}

static Vector2r proj(double longtitude, double latitude)
{
    constexpr double PI = 3.141592653589793;

    double x = ((longtitude * PI) / 180.) * 6378137.0;
    double y = ((latitude * PI) / 180.) * 6378137.0;

    return Vector2r(x, y);
}

static void parsePolygon(Value &polygonJSON, GeometryIO::Polygon &polygon)
{
    int nLoops = polygonJSON.Size();

    for (int i = 0; i < nLoops; ++i)
    {
        GeometryIO::Loop loop;
        Value &loopJSON = polygonJSON[i];
        int deg = loopJSON.Size();
        loop.reserve(deg);
        for (int k = 0; k < deg; ++k)
        {
            Value &coord = loopJSON[k];
            Vector2r xy = proj(coord[0].GetDouble(), coord[1].GetDouble());
            // Vector2r xy = Vector2r(coord[0].GetDouble(), coord[1].GetDouble());
            loop.push_back(xy);
        }
        polygon.push_back(loop);
    }
}

static bool parseJSON(const char *jsonStr, std::vector<GeometryIO::Polygon> &polygons)
{

    Document doc;

    if (doc.Parse(jsonStr).HasParseError())
    {
        std::cerr << "parse error" << std::endl;
        return false;
    }

    if (!doc.IsObject())
    {
        std::cerr << "should be a object" << std::endl;
        return false;
    }

    if (doc.HasMember("features"))
    {
        Value &features = doc["features"];

        if (features.IsArray())
        {
            int N = features.Size();
            for (int i = 0; i < N; ++i)
            {
                Value &feature = features[i];
                if (feature.HasMember("geometry"))
                {
                    Value &geo = feature["geometry"];
                    std::string type = geo["type"].GetString();

                    if (type == "Polygon")
                    {
                        Value &polygonJSON = geo["coordinates"];
                        GeometryIO::Polygon polygon;
                        parsePolygon(polygonJSON, polygon);
                        polygons.push_back(polygon);
                    }
                    else if (
                        type == "MultiPolygon")
                    {
                        Value &multiPolygon = geo["coordinates"];

                        // for (int poly_ind = 0; poly_ind < multiPolygon.Size(); ++poly_ind)
                        // {
                        //     Value &polygonJSON = multiPolygon[poly_ind][0];
                        //     GeometryIO::Polygon polygon;
                        //     parsePolygon(polygonJSON, polygon);
                        //     polygons.push_back(polygon);
                        // }
                    }
                }
            }
        }
    }

    return true;
}

void GeometryIO::loadGeoFromJSON(const char *path, std::vector<Polygon> &Polygons)
{
    string res;
    readFile(path, res);
    parseJSON(res.c_str(), Polygons);
}

void GeometryIO::loadHeightMapFromTIFF(const char *path, HeightMap &heightMap)
{

    TinyTIFFReaderFile *tiffr = TinyTIFFReader_open(path);
    if (!tiffr)
    {
        std::cout << "    ERROR reading (not existent, not accessible or no TIFF file)\n";
        return;
    }

    const uint32_t width = TinyTIFFReader_getWidth(tiffr);
    const uint32_t height = TinyTIFFReader_getHeight(tiffr);
    const uint16_t samples = TinyTIFFReader_getSamplesPerPixel(tiffr);
    const uint16_t bitsPerSample = TinyTIFFReader_getBitsPerSample(tiffr, 0);

    string descrption = TinyTIFFReader_getImageDescription(tiffr);

    MatrixXR16u &hmap = heightMap.m_data;
    hmap.resize(height, width);
    TinyTIFFReader_getSampleData(tiffr, hmap.data(), 0);
    heightMap.init();
}

void GeometryIO::writeObj(const std::string &path, const std::vector<Vector3r> &V, const std::vector<Vector3i> &F)
{
    std::ofstream os(path);
    if (os.fail())
    {
        throw std::runtime_error("Unable to open OBJ file \"" + path + "\"!");
    }
    for (int i = 0; i < V.size(); ++i)
    {
        os << "v " << V[i].x() << " " << V[i].y() << " " << V[i].z() << endl;
    }

    for (int i = 0; i < F.size(); ++i)
    {
        os << "f " << F[i].x() + 1 << " " << F[i].y() + 1 << " " << F[i].z() + 1 << endl;
    }

    os.close();
}

void GeometryIO::writeCsv(const std::string &path, const std::vector<Vector3r> &V)
{

    std::ofstream os(path);

    if (os.fail())
    {
        throw std::runtime_error("Unable to open OBJ file \"" + path + "\"!");
    }
    for (int i = 0; i < V.size(); ++i)
    {
        os << V[i].x() << ", " << V[i].y() << ", " << V[i].z() << endl;
    }


}
