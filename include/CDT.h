#pragma once
#include <GeoIO.h>
#include <vector>



class CDT
{
public:
    static std::vector<Vector3i> triangulateLoop(const GeometryIO::Loop &loop);

    static std::pair<std::vector<Vector2r>, std::vector<Vector3i>> isotropicRemesh(std::vector<Vector2r>& V, std::vector<Vector3i>& F, const Real& target_length_ratio);

};