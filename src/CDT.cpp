#include <common.h>
#include <GeoIO.h>
#include <vector>
#include <queue>
#include <CDT.h>
#include <iostream>
#include <unordered_set>
#include <dedge.h>
#include <isotropicRemesher.h>
using namespace std;
using GeoIO = GeometryIO;

enum CrossTest
{
    EXTERIOR = 0,
    ONEDGE = 1,
    ONVERTEX = 2,
    INTERIOR = 3
};
// Ear struct is used in circular list for ear-clipping
struct Ear
{
    Vector2r p;
    int originalIndex;
    int prev;
    int next;
    bool operator<(const Ear &other) { return this->originalIndex < other.originalIndex; }
};

constexpr Real EPS = 1e-12;

static inline double signedArea(const Vector2r &p, const Vector2r &q, const Vector2r &r)
{
    return 0.5 * ((p[0] * q[1] - p[1] * q[0]) + (q[0] * r[1] - q[1] * r[0]) + (r[0] * p[1] - r[1] * p[0]));
}

static CrossTest pointInTriangle(const Vector2r &a, const Vector2r &p, const Vector2r &q, const Vector2r &r)
{
    std::array<double, 3> test;
    test[0] = (a[0] - r[0]) * (q[1] - r[1]) - (q[0] - r[0]) * (a[1] - r[1]);
    test[1] = (a[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (a[1] - p[1]);
    test[2] = (a[0] - q[0]) * (p[1] - q[1]) - (p[0] - q[0]) * (a[1] - q[1]);
    std::array<int, 3> intTest;
    intTest[0] = (test[0] > EPS ? 1 : (test[0] < -EPS ? -1 : 0));
    intTest[1] = (test[1] > EPS ? 1 : (test[1] < -EPS ? -1 : 0));
    intTest[2] = (test[2] > EPS ? 1 : (test[2] < -EPS ? -1 : 0));
    int sum = intTest[0] + intTest[1] + intTest[2];
    int prod = intTest[0] * intTest[1] * intTest[2];

    if (abs(sum) == 3)
    {
        return CrossTest::INTERIOR;
    }

    if (abs(sum) == 2)
    {
        return CrossTest::ONEDGE;
    }

    if (abs(sum) == 1 && prod == 0)
    {
        return CrossTest::ONVERTEX;
    }

    return CrossTest::EXTERIOR;
}

static bool clippable(const Ear &ear, const vector<Ear> &poly)
{
    const Vector2r &p = ear.p;
    int prev = ear.prev;
    const Vector2r &q = poly[prev].p;
    int next = ear.next;
    const Vector2r &r = poly[next].p;
    double signA = signedArea(q, p, r);
    if (signA <= EPS)
    {
        return false;
    }

    int current_Vertex = poly[next].next;

    while (current_Vertex != prev)
    {
        const Vector2r &a = poly[current_Vertex].p;

        auto result = pointInTriangle(a, q, p, r);

        if (result != CrossTest::EXTERIOR)
        {
            return false;
        }

        current_Vertex = poly[current_Vertex].next;
    }

    return true;
}

std::vector<Vector3i> CDT::triangulateLoop(
    const GeoIO::Loop &loop)
{
    size_t N = loop.size();
    bool anti_clockwise = true;
    {
        // emm.... some stupid input
        const Vector2r &first = loop.front();
        const Vector2r &end = loop.back();

        const bool firstEqualEnd = (std::abs(first(0) - end(0)) <= EPS && std::abs(first[1] - end[1]) <= EPS);

        N = firstEqualEnd ? loop.size() - 1 : loop.size();
        Real signedArea = 0;
        for (int i = 0; i < N; ++i)
        {
            const Vector2r &q = loop[i];
            const Vector2r &p = loop[(i + 1) % N];
            signedArea += (q[0] * p[1] - q[1] * p[0]);
        }

        anti_clockwise = signedArea > 0;
    }

    std::vector<Ear> poly(N); // use poly to represent loop, since loop is dirty
    {
        for (int i = 0; i < N; ++i)
        {
            poly[i].p = loop[i];
            poly[i].originalIndex = i;
        }

        for (int i = 0; i < N; ++i)
        {
            // connect as clock-wise
            if (anti_clockwise)
            {
                poly[i].prev = (poly[i].originalIndex + N - 1) % N;
                poly[i].next = (poly[i].originalIndex + 1) % N;
            }
            else
            {
                poly[i].prev = (poly[i].originalIndex + 1) % N;
                poly[i].next = (poly[i].originalIndex + N - 1) % N;
            }
        }
    }

    std::vector<Vector3i> triangles;
    triangles.reserve(N);

    queue<Ear *> earQ;
    for (auto &ear : poly)
    {
        earQ.push(&ear);
    }

    while (earQ.size() >= 3)
    {
        Ear *ear = earQ.front();
        earQ.pop();
        if (!clippable(*ear, poly))
        {
            earQ.push(ear);
            continue;
        }

        // if clippable
        int ear_prev = ear->prev;
        int ear_next = ear->next;
        triangles.push_back({ear_prev, ear->originalIndex, ear->next});

        ear->next = -1;
        ear->prev = -1;

        poly[ear_prev].next = ear_next;
        poly[ear_next].prev = ear_prev;
    }

    return std::move(triangles);
}

std::pair<std::vector<Vector2r>, std::vector<Vector3i>> CDT::isotropicRemesh(std::vector<Vector2r> &V_, std::vector<Vector3i> &F_, const Real &target_length_ratio)
{

    typedef std::pair<std::vector<Vector2r>, std::vector<Vector3i>> Mesh;
    Mesh mesh;
    {
        IsotropicRemesher remesh(V_, F_, 1.);
        remesh.init();
        remesh.run();
        mesh = remesh.getMesh();
    }

    if (target_length_ratio > 1.)
    {
        return mesh;
    }

    std::vector<Real> ratio_table;

    int n = 0;
    Real r = 1.;
    while (0.5 * r >= target_length_ratio)
    {
        r *= 0.5;
        n++;
    }

    Real last = target_length_ratio / r;

    ratio_table.resize(n + 1, 0.5);
    ratio_table.back() = last;

    for (auto rt : ratio_table)
    {
        IsotropicRemesher remesh(mesh.first, mesh.second, rt);
        remesh.init();
        remesh.run();
        mesh = remesh.getMesh();
    }

    return mesh;
}
