#include <Sample.h>

#include <assert.h>
#include <functional>
#include <vector>
#include <unordered_map>
#include <map>
#include <algorithm>

#include <iostream>

using namespace std;

const Real PI = 3.14159265359;

static Real random_Real(Real maximum)
{
    return ((Real)rand() / (Real)(RAND_MAX / maximum));
}

static Real easy_geodesic_dist(
    const Vector3r &p1, const Vector3r &p2,
    const Vector3r &n1, const Vector3r &n2)
{
    Vector3r v = p2 - p1;
    v.normalize();
    // n1.normalize();
    // n2.normalize();

    Real c1 = n1.dot(v);
    Real c2 = n2.dot(v);

    Real res = (p2 - p1).squaredNorm();

    if (abs(c1 - c2) > 0.001)
    {
        res *= (asin(c1) - asin(c2)) / (c1 - c2);
    }

    return res;
}

struct raw_points
{

    int face_id;
    int cell_id;
    Vector3r p;
    Vector3r n;
};

static Real easy_sample(
    int num_samples,
    const vector<Vector3r> &points,
    const vector<Vector3r> &normals,
    const vector<Vector3i> &faces,
    const vector<Real> &areas,
    vector<raw_points> &sampled_raw)
{

    map<Real, int> area_sum_to_ind;
    Real max_area_sum = 0.;

    for (int i = 0; i < areas.size(); ++i)
    {
        area_sum_to_ind[max_area_sum] = i;
        max_area_sum += areas[i];
    }

    for (int i = 0; i < num_samples; ++i)
    {
        Real r = random_Real(max_area_sum);
        auto iter = area_sum_to_ind.upper_bound(r);

        assert(iter != area_sum_to_ind.begin());

        --iter;

        int f = iter->second;

        int v0 = faces[f](0);
        int v1 = faces[f](1);
        int v2 = faces[f](2);

        const Vector3r &p0 = points[v0];
        const Vector3r &p1 = points[v1];
        const Vector3r &p2 = points[v2];

        const Vector3r &n0 = normals[v0];
        const Vector3r &n1 = normals[v1];
        const Vector3r &n2 = normals[v2];

        Real u = random_Real(1);
        Real v = random_Real(1);

        // barycenter random sampling
        Vector3r p = p0 * (1 - sqrt(u)) + p1 * (sqrt(u) * (1 - v)) + p2 * (v * sqrt(u));
        Vector3r n = n0 * (1 - sqrt(u)) + n1 * (sqrt(u) * (1 - v)) + n2 * (v * sqrt(u));

        n.normalize();

        sampled_raw.emplace_back();

        raw_points &rp = sampled_raw.back();

        rp.cell_id = -1;
        rp.p = p;
        rp.n = n;
        rp.face_id = f;
    }

    return max_area_sum;
}

static void make_poission_from_raws(
    Real radius,
    vector<raw_points> &sampled_raw,
    vector<Vector3r> &sampled_p,
    vector<Vector3r> &sampled_n)
{
    Vector3r left_bottom = sampled_raw[0].p;
    Vector3r right_top = left_bottom;

    for (int i = 0; i < sampled_raw.size(); ++i)
    {
        const Vector3r &p = sampled_raw[i].p;
        left_bottom(0) = min(left_bottom(0), p(0));
        left_bottom(1) = min(left_bottom(1), p(1));
        left_bottom(2) = min(left_bottom(2), p(2));

        right_top(0) = max(right_top(0), p(0));
        right_top(1) = max(right_top(1), p(1));
        right_top(2) = max(right_top(2), p(2));
    }

    Vector3r bbx_size = right_top - left_bottom;

    const Real radius_sq = radius * radius;

    Vector3r grid_size = bbx_size / radius;

    Vector3i grid_size_int({lrint(grid_size(0)),
                            lrint(grid_size(1)),
                            lrint(grid_size(2))});

    grid_size_int(0) = max(grid_size_int(0), 1);
    grid_size_int(1) = max(grid_size_int(1), 1);
    grid_size_int(2) = max(grid_size_int(2), 1);

    auto index_grid_cell = [&](Vector3r p)
    {
        Vector3r cell_length = bbx_size.cwiseQuotient(grid_size_int.cast<Real>());
        Vector3r lcoord = p - left_bottom;
        Vector3r idx = lcoord.cwiseQuotient(cell_length);
        Vector3i int_idx = idx.cast<int>();
        return int_idx;
    };

    auto to_linear = [&](Vector3i idx)
    {
        return idx.x() + grid_size_int.x() * (idx.y() + grid_size_int.y() * idx.z());
    };

    for (auto &pos : sampled_raw)
    {
        Vector3i idx = index_grid_cell(pos.p);
        int id = to_linear(idx);
        pos.cell_id = id;
    }

    sort(sampled_raw.begin(), sampled_raw.end(), [](const raw_points &lhs, const raw_points &rhs)
         { return lhs.cell_id < rhs.cell_id; });

    struct poisson_sample
    {
        Vector3r p;
        Vector3r n;
    };

    struct hash_data
    {
        vector<poisson_sample> poisson_samples;
        int first_sample_id;
        int sample_cnt;
    };

    unordered_map<int, hash_data> cells; // spatial hash

    {
        int last_id = -1;
        unordered_map<int, hash_data>::iterator last_id_iter;

        for (int i = 0; i < sampled_raw.size(); ++i)
        {
            const auto &sample = sampled_raw[i];
            if (sample.cell_id == last_id)
            {
                ++last_id_iter->second.sample_cnt;
                continue;
            }

            hash_data data;

            data.first_sample_id = i;
            data.sample_cnt = 1;

            auto result = cells.insert({sample.cell_id, data});
            last_id = sample.cell_id;
            last_id_iter = result.first;
        }
    }

    vector<int> neighbor_cell_offsets;

    {
        for (int x = -1; x <= +1; ++x)
        {
            for (int y = -1; y <= +1; ++y)
            {
                for (int z = -1; z <= +1; ++z)
                {
                    Vector3i off({x, y, z});
                    int id = to_linear(off);
                    neighbor_cell_offsets.push_back(id);
                }
            }
        }
    }

    int max_trial = 5;

    for (int trial = 0; trial < max_trial; ++trial)
    {
        for (auto &it : cells)
        {
            int cell_id = it.first;
            hash_data &data = it.second;

            int next_sample_id = data.first_sample_id + trial;
            if (trial >= data.sample_cnt)
            {
                continue;
            }

            const auto &candidate = sampled_raw[next_sample_id];
            bool conflict = false;

            for (int neighbor_offset : neighbor_cell_offsets)
            {
                int neighbor_cell_id = cell_id + neighbor_offset;
                const auto &it = cells.find(neighbor_cell_id);
                if (it == cells.end())
                {
                    continue;
                }

                const hash_data &neighbor = it->second;

                for (const auto &sample : neighbor.poisson_samples)
                {
                    Real distance = easy_geodesic_dist(sample.p, candidate.p, sample.n, candidate.n);
                    if (distance < radius_sq)
                    {
                        conflict = true;
                        break;
                    }
                }

                if (conflict)
                {
                    break;
                }
            }

            if (conflict)
            {
                continue;
            }

            data.poisson_samples.emplace_back();
            poisson_sample &new_sample = data.poisson_samples.back();
            new_sample.p = candidate.p;
            new_sample.n = candidate.n;
        }
    }
    for (const auto it : cells)
    {
        for (const auto &sample : it.second.poisson_samples)
        {
            sampled_p.push_back(sample.p);
            sampled_n.push_back(sample.n);
        }
    }
}

static void compute_area_normal(
    const vector<Vector3r> &points,
    const vector<Vector3i> &faces,
    vector<Vector3r> &normals,
    vector<Real> &areas)
{
    normals.resize(points.size(), Vector3r(0, 0, 0));
    areas.resize(faces.size(), 0);

    for (int f = 0; f < faces.size(); ++f)
    {
        const Vector3i &face = faces[f];

        int v0 = face[0];
        int v1 = face[1];
        int v2 = face[2];

        Vector3r p0 = points[v0];
        Vector3r p1 = points[v1];
        Vector3r p2 = points[v2];

        Vector3r e01 = p1 - p0;
        Vector3r e02 = p2 - p0;

        Vector3r areaNorm = e01.cross(e02);

        Real area = areaNorm.norm();

        areas[f] = area;

        normals[v0] += areaNorm;
        normals[v1] += areaNorm;
        normals[v2] += areaNorm;
    }

    for (int i = 0; i < normals.size(); ++i)
    {
        normals[i].normalize();
    }
}

void PoissonDiskOnMesh::sample(
    const std::vector<Vector3r> &points,
    const std::vector<Vector3i> &faces,
    Real radius, int n_samples,
    std::vector<Vector3r> &sample_points)
{
    if(radius < 0.){
        std::cout<< "radius < 0"<< std::endl;
        return;
    }

    std::vector<Real> areas;
    std::vector<Vector3r> normals;

    compute_area_normal(points, faces, normals, areas);

    vector<raw_points> sample_raw;
    Real surface_area = easy_sample(
        100000, points, normals, faces, areas, sample_raw);


    vector<Vector3r> sample_n;
    make_poission_from_raws(
        radius,sample_raw,sample_points,sample_n
    );

}
