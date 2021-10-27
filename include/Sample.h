#pragma once
#include <common.h>


class PoissonDiskOnMesh{

public:
    
    static void sample(const std::vector<Vector3r>& points,const std::vector<Vector3i>& faces, Real radius, int n_samples, std::vector<Vector3r>& sample_points);

};





