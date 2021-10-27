
#pragma once

#include "common.h"

static constexpr int INVALID = -1;

inline int dedge_prev_3(int e) { return (e % 3 == 0) ? e + 2 : e - 1; }
inline int dedge_next_3(int e) { return (e % 3 == 2) ? e - 2 : e + 1; }

inline int dedge_from_vertex(int e, const MatrixXi &F)
{
    return F(e % 3, e / 3);
}

inline int dedge_to_vertex(int e, const MatrixXi &F)
{
    int e_next = dedge_next_3(e);
    return dedge_from_vertex(e_next, F);
}

inline int dedge_prev(int e, int deg) { return (e % deg == 0u) ? e + (deg - 1) : e - 1; }
inline int dedge_next(int e, int deg) { return (e % deg == deg - 1) ? e - (deg - 1) : e + 1; }

extern void build_dedge(const MatrixXi &F, const MatrixXr &V, VectorXi &V2E,
                        VectorXi &E2E, VectorXb &boundary, VectorXb &nonManifold);