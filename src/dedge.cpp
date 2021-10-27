#include "dedge.h"
#include <vector>

void build_dedge(const MatrixXi &F, const MatrixXr &V, VectorXi &V2E,
                         VectorXi &E2E, VectorXb &boundary, VectorXb &nonManifold) {

    V2E.resize(V.cols());
    V2E.setConstant(INVALID);

    int deg = F.rows();
    std::vector<std::pair<int, int>> tmp(F.size());


    for (int f = 0; f < F.cols(); ++f) {
        for (int i = 0; i < deg; ++i) {
            int idx_cur = F(i, f);
            int idx_next = F((i+1)%deg, f);
            int edge_id = deg * f + i;

            if (idx_cur >= V.cols() || idx_next >= V.cols()) {
                throw std::runtime_error("Mesh data contains an out-of-bounds vertex reference!");
            }
            if (idx_cur == idx_next) {
                continue;
            }

            tmp[edge_id] = std::make_pair(idx_next, INVALID);

            
            if (V2E[idx_cur] == INVALID) {
                V2E[idx_cur] = edge_id;
            }
            else {
                int idx = V2E[idx_cur];
                while (tmp[idx].second != INVALID) {
                    idx = tmp[idx].second;
                }
                tmp[idx].second = edge_id;
            }

        }
    }


    nonManifold.resize(V.cols());
    nonManifold.setConstant(false);

    E2E.resize(F.cols() * deg);
    E2E.setConstant(INVALID);

    for (int f = 0; f < F.cols(); ++f) {
        for (int i = 0; i < deg; ++i) {
            int v_cur = F(i, f);
            int v_next = F((i + 1) % deg, f);
            int cur_he = f * deg + i;

            // untrival trangle
            if (v_cur == v_next) {
                continue;
            }
            int cur_he_opp = INVALID;
            int he = V2E[v_next];
            while (he != INVALID) {
                if (tmp[he].first == v_cur) {
                    if (cur_he_opp == INVALID) {
                        cur_he_opp = he;
                    }
                    else {
                        nonManifold[v_cur] = true;
                        nonManifold[v_next] = true;
                        cur_he_opp = INVALID;
                        break;
                    }
                }
                he = tmp[he].second;
            }
            if (cur_he_opp != INVALID && cur_he < cur_he_opp) {
                E2E[cur_he] = cur_he_opp;
                E2E[cur_he_opp] = cur_he;
            }
        }
    }

    int nonManifoldVertexCounter(0), boundaryVertexCounter(0), isolatedVertexCounter(0);

    boundary.resize(V.cols());
    boundary.setConstant(false);


    for (int v = 0; v < V.cols(); v++) {
        int he = V2E[v];
        if (he == INVALID) {
            isolatedVertexCounter++;
            continue;
        }
        if (nonManifold[v]) {
            nonManifoldVertexCounter++;
            V2E[v] = INVALID;
            continue;
        }

        int he_start = he;
        int v2e = INT_MAX;
        do {
            v2e = std::min(v2e, he);
            int v_prevHe = E2E(dedge_prev(he, deg));
            if (v_prevHe == INVALID) {
                v2e = he; 
                boundary[v] = true;
                boundaryVertexCounter++;
                break;
            }
            he = v_prevHe;
        } while (he != he_start);
        V2E[v] = v2e;
    }
}