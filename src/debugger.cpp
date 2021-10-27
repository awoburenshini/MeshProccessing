#include <common.h>
#include <debugger.h>
#include <fstream>
#include <vector>
#include <string>

void Mdebugger::debuggerPolygon(
    const GeometryIO::Polygon &polygon,
    const std::string &path)
{

    int vert_cnt = 0;

    for (auto &loop : polygon)
    {
        vert_cnt += (loop.size() - 1);
    }

    int face_cnt = polygon.size();

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
        os << "element vertex " << vert_cnt << std::endl;
        os << "property float x" << std::endl;
        os << "property float y" << std::endl;
        os << "property float z" << std::endl;
    }

    // attribute vertice header
    {
        os << "element face " << face_cnt << std::endl;
        os << "property list "
           << "uchar "
           << "int "
           << "vertex_indices" << std::endl;
    }

    // header end
    {
        os << "end_header" << std::endl;
    }

    for (const auto &loop : polygon)
    {
        for (int i = 0; i < loop.size() - 1; ++i)
        {
            const Vector2r &p = loop[i];
            os << p.x() << " " << p.y() << " " << 0. << std::endl;
        }
    }

    int flag = 0;

    for (const auto &loop : polygon)
    {
        os << loop.size() - 1 << " ";

        for (int i = 0; i < loop.size(); ++i)
        {
            os << flag++ << " ";
        }
        os << std::endl;
    }
};

void Mdebugger::debuggerMesh(
    const std::string &path,
    const std::vector<Vector3i> &F,
    const std::vector<Vector2r> &V,
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