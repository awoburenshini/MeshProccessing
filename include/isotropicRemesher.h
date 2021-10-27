#pragma once
#include <common.h>
#include <vector>
#include <queue>

class IsotropicRemesher
{

public:
    IsotropicRemesher(
        const std::vector<Vector2r> &V_,
        const std::vector<Vector3i> &F_,
        const Real &target_length_ratio_)
        : target_length_ratio(target_length_ratio_), avg_length(0.)
    {
        V = Eigen::Map<const MatrixXr>(&V_[0][0], 2, V_.size());
        F = Eigen::Map<const MatrixXi>(&F_[0][0], 3, F_.size());
        nV = V.cols();
        nF = F.cols();
        activeFace.resize(F.cols());
        activeFace.setConstant(true);
    }

    std::pair<std::vector<Vector2r>, std::vector<Vector3i>> getMesh();

    void init();

    void run();

public:
    void split_long_edges();

    void flip_to_delauney();

    void collapse_short_edge();

    void renewTopology();

    void barycenter_smooth();

private:

    void getEdgeList(std::vector<std::pair<int,int>>& edges);
    void getEdgeList(std::queue<std::pair<int,int>>& edges);

    // return inserted vertex
    int split_edge(int e);

    // return flipped halfedge
    int flip_edge(int e);

    // return merged vertex, actually is from
    int collapse_edge(int e);

    bool isCollapsible(int e) const;

    bool delauneyFlipCondition(int e) const;

    bool needToFlip(int e) const;

    int findFromToHalfedge(int from, int to) const;

private:
    Real target_length_ratio;
    Real target_length;
    Real avg_length;
    Real low;
    Real high;

private:
    int nV;
    int nF;

private:
    VectorXb activeFace;

private:
    MatrixXr V;
    MatrixXi F;

    VectorXi V2E;
    VectorXi E2E;
    VectorXb isBoundaryVertex;
    VectorXb NonManiFold;
};