#include <isotropicRemesher.h>
#include <dedge.h>
#include <unordered_set>
#include <set>
#include <iostream>
#include <queue>
#include <map>
#include <list>

using namespace std;
void IsotropicRemesher::init()
{
    build_dedge(F, V, V2E, E2E, isBoundaryVertex, NonManiFold);
    avg_length = 0;
    for (int e = 0; e < E2E.size(); ++e)
    {
        int to = dedge_to_vertex(e, F);
        int from = dedge_from_vertex(e, F);

        avg_length += (V.col(to) - V.col(from)).norm();
    }
    avg_length = avg_length / (Real)E2E.size();
    target_length = avg_length * target_length_ratio;
    low = 0.4 * target_length;
    high = target_length * 4. / 3.;

    int nV = V.cols();
    int nF = F.cols();
}

// update sequence
// 1) V, 2) F 3) activeFace, 4) E2E, 5) isBoundaryVertex, 6) V2E
int IsotropicRemesher::split_edge(int e)
{
    // the neccesary of split these edge have been determined
    int from = dedge_from_vertex(e, F);
    int to = dedge_to_vertex(e, F);

    // Real e_length = (V.col(to) - V.col(from)).norm();

    if (E2E(e) == -1)
    {

        int new_v_idx = nV++;
        if (nV > V.cols())
        {
            V.conservativeResize(V.rows(), 2 * V.cols());
            int nV2E = V2E.size();
            V2E.conservativeResize(V.cols());
            for (int i = nV2E; i < V2E.size(); ++i)
            {
                V2E(i) = -2;
            }

            isBoundaryVertex.conservativeResize(V.cols());
        }
        Vector2r mid = (V.col(from) + V.col(to)) * .5;

        int e_next = dedge_next_3(e);
        int up = dedge_to_vertex(e_next, F);

        int f0 = e / 3;
        int f1 = nF++;
        if (nF > F.cols())
        {
            F.conservativeResize(F.rows(), 2 * F.cols());
            E2E.conservativeResize(3 * F.cols());
            int nAct = activeFace.size();
            activeFace.conservativeResize(F.cols());
            for (int i = nAct; i < activeFace.size(); ++i)
            {
                activeFace(i) = false;
            }
        }

        int e_from_up = -1;
        int e_up_to = -1;
        {
            int e_prev = dedge_prev_3(e);
            e_from_up = E2E(e_prev);
            e_up_to = E2E(e_next);
        }

        V.col(new_v_idx) = mid;

        F.col(f0) = Vector3i({up, from, new_v_idx});
        F.col(f1) = Vector3i({up, new_v_idx, to});

        activeFace(f0) = true;
        activeFace(f1) = true;

        E2E(3 * f0 + 0) = e_from_up;
        E2E(3 * f0 + 1) = -1;
        E2E(3 * f0 + 2) = 3 * f1 + 0;
        if (e_from_up != -1)
        {
            E2E(e_from_up) = 3 * f0 + 0;
        }

        E2E(3 * f1 + 0) = 3 * f0 + 2;
        E2E(3 * f1 + 1) = -1;
        E2E(3 * f1 + 2) = e_up_to;
        if (e_up_to != -1)
        {
            E2E(e_up_to) = 3 * f1 + 2;
        }

        isBoundaryVertex(new_v_idx) = true;

        V2E(from) = 3 * f0 + 1;
        V2E(up) = 3 * f0 + 0;
        V2E(to) = 3 * f1 + 2;
        V2E(new_v_idx) = 3 * f1 + 1;

        // return {3 * f0 + 1, 3 * f0 + 2, 3 * f1 + 1};
        return new_v_idx;
    }
    else
    {

        int new_v_idx = nV++;
        if (nV > V.cols())
        {
            V.conservativeResize(V.rows(), 2 * V.cols());
            int nV2E = V2E.size();
            V2E.conservativeResize(V.cols());
            for (int i = nV2E; i < V2E.size(); ++i)
            {
                V2E(i) = -2;
            }

            isBoundaryVertex.conservativeResize(V.cols());
        }

        V.col(new_v_idx) = (V.col(from) + V.col(to)) * .5;

        int f = e / 3;
        int e_opp = E2E(e);
        int new_f0_idx = e_opp / 3;
        int new_f1_idx = nF++;
        int new_f2_idx = nF++;
        if (nF > F.cols())
        {
            F.conservativeResize(F.rows(), 2 * F.cols());
            E2E.conservativeResize(3 * F.cols());

            int nAct = activeFace.size();
            activeFace.conservativeResize(F.cols());
            for (int i = nAct; i < activeFace.size(); ++i)
            {
                activeFace(i) = false;
            }
        }

        int e_next = dedge_next_3(e);
        int e_opp_next = dedge_next_3(e_opp);

        int e_from_up = -1;
        int e_down_from = -1;
        int e_to_down = -1;
        int e_up_to = -1;
        {
            e_up_to = E2E(e_next);
            int e_prev = dedge_prev_3(e);
            e_from_up = E2E(e_prev);

            int e_opp_next = dedge_next_3(e_opp);
            e_down_from = E2E(e_opp_next);

            int e_opp_prev = dedge_prev_3(e_opp);
            e_to_down = E2E(e_opp_prev);
        }

        int from = dedge_from_vertex(e, F);
        int to = dedge_to_vertex(e, F);
        int up = dedge_to_vertex(e_next, F);
        int down = dedge_to_vertex(e_opp_next, F);

        F.col(f) = Vector3i({from, new_v_idx, up});
        F.col(new_f0_idx) = Vector3i({new_v_idx, to, up});
        F.col(new_f1_idx) = Vector3i({new_v_idx, down, to});
        F.col(new_f2_idx) = Vector3i({new_v_idx, from, down});
        activeFace(f) = true;
        activeFace(new_f0_idx) = true;
        activeFace(new_f1_idx) = true;
        activeFace(new_f2_idx) = true;

        E2E(3 * f + 0) = 3 * new_f2_idx + 0;
        E2E(3 * f + 1) = 3 * new_f0_idx + 2;
        E2E(3 * f + 2) = e_from_up;
        if (e_from_up != -1)
        {
            E2E(e_from_up) = 3 * f + 2;
        }

        E2E(3 * new_f0_idx + 0) = 3 * new_f1_idx + 2;
        E2E(3 * new_f0_idx + 1) = e_up_to;
        E2E(3 * new_f0_idx + 2) = 3 * f + 1;
        if (e_up_to != -1)
        {
            E2E(e_up_to) = 3 * new_f0_idx + 1;
        }

        E2E(3 * new_f1_idx + 0) = 3 * new_f2_idx + 2;
        E2E(3 * new_f1_idx + 1) = e_to_down;
        E2E(3 * new_f1_idx + 2) = 3 * new_f0_idx + 0;
        if (e_to_down != -1)
        {
            E2E(e_to_down) = 3 * new_f1_idx + 1;
        }

        E2E(3 * new_f2_idx + 0) = 3 * f + 0;
        E2E(3 * new_f2_idx + 1) = e_down_from;
        E2E(3 * new_f2_idx + 2) = 3 * new_f1_idx + 0;
        if (e_down_from != -1)
        {
            E2E(e_down_from) = 3 * new_f2_idx + 1;
        }

        isBoundaryVertex(new_v_idx) = false;

        V2E(up) = 3 * f + 2;
        V2E(to) = 3 * new_f0_idx + 1;
        V2E(down) = 3 * new_f1_idx + 1;
        V2E(from) = 3 * new_f2_idx + 1;

        V2E(new_v_idx) = 3 * f + 1;

        // return {3 * f + 0, 3 * f + 1, 3 * new_f0_idx + 0, 3 * new_f1_idx + 0};
        return new_v_idx;
    }
}

int IsotropicRemesher::flip_edge(int e)
{
    if (e == -1 || E2E(e) == -1)
    {
        return -1;
    }

    int from = -1;
    int to = -1;
    int up = -1;
    int down = -1;

    int e_from_up = -1;
    int e_down_from = -1;
    int e_to_down = -1;
    int e_up_to = -1;

    int f0 = -1;
    int f1 = -1;
    {
        from = dedge_from_vertex(e, F);
        to = dedge_to_vertex(e, F);
        int e_next = dedge_next_3(e);
        up = dedge_to_vertex(e_next, F);
        int e_opp = E2E(e);
        int e_opp_next = dedge_next_3(e_opp);
        down = dedge_to_vertex(e_opp_next, F);

        int e_prev = dedge_prev_3(e);
        e_from_up = E2E(e_prev);
        e_down_from = E2E(e_opp_next);
        int e_opp_prev = dedge_prev_3(e_opp);
        e_to_down = E2E(e_opp_prev);
        e_up_to = E2E(e_next);

        f0 = e / 3;
        f1 = e_opp / 3;
    }

    F.col(f0) = Vector3i({from, down, up});
    F.col(f1) = Vector3i({up, down, to});

    E2E(3 * f0 + 0) = e_down_from;
    E2E(3 * f0 + 1) = 3 * f1 + 0;
    E2E(3 * f0 + 2) = e_from_up;
    if (e_down_from != -1)
    {
        E2E(e_down_from) = 3 * f0 + 0;
    }
    if (e_from_up != -1)
    {
        E2E(e_from_up) = 3 * f0 + 2;
    }

    E2E(3 * f1 + 0) = 3 * f0 + 1;
    E2E(3 * f1 + 1) = e_to_down;
    E2E(3 * f1 + 2) = e_up_to;
    if (e_to_down != -1)
    {
        E2E(e_to_down) = 3 * f1 + 1;
    }
    if (e_up_to != -1)
    {
        E2E(e_up_to) = 3 * f1 + 2;
    }

    V2E(from) = 3 * f0 + 0;
    V2E(down) = 3 * f0 + 1;
    V2E(to) = 3 * f1 + 2;
    V2E(up) = 3 * f1 + 0;

    return 3 * f0 + 1;
}

bool IsotropicRemesher::isCollapsible(int e) const
{

    int from = dedge_from_vertex(e, F);
    int to = dedge_to_vertex(e, F);

    if (isBoundaryVertex(from) || isBoundaryVertex(to))
    {
        return false;
    }
    unordered_set<int> adj_from;
    adj_from.reserve(16);
    {
        int current_e = e;
        do
        {

            int c_to = dedge_to_vertex(current_e, F);
            adj_from.insert(c_to);
            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);
        } while (current_e != e);
    }

    int adj_vertex_cnt = 0;
    {
        int current_e = E2E(e);
        do
        {
            int c_to = dedge_to_vertex(current_e, F);

            if (adj_from.find(c_to) != adj_from.end())
            {
                adj_vertex_cnt++;
            }
            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);
        } while (current_e != E2E(e));
    }
    if (adj_vertex_cnt == 2)
    {
        return true;
    }
    return false;
}

int IsotropicRemesher::collapse_edge(int e)
{
    int from = -1;
    int to = -1;
    int up = -1;
    int down = -1;

    int e_from_up = -1;
    int e_down_from = -1;
    int e_to_down = -1;
    int e_up_to = -1;

    int f0 = -1;
    int f1 = -1;
    {
        from = dedge_from_vertex(e, F);
        to = dedge_to_vertex(e, F);
        int e_next = dedge_next_3(e);
        up = dedge_to_vertex(e_next, F);
        int e_opp = E2E(e);
        int e_opp_next = dedge_next_3(e_opp);
        down = dedge_to_vertex(e_opp_next, F);

        int e_prev = dedge_prev_3(e);
        e_from_up = E2E(e_prev);
        e_down_from = E2E(e_opp_next);
        int e_opp_prev = dedge_prev_3(e_opp);
        e_to_down = E2E(e_opp_prev);
        e_up_to = E2E(e_next);

        f0 = e / 3;
        f1 = e_opp / 3;
    }

    std::set<int> e_one_ring_face;
    {

        int current_e = e;
        do
        {
            int f = current_e / 3;
            if (f != f0 && f != f1)
            {
                e_one_ring_face.insert(f);
            }
            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);
        } while (current_e != e);

        current_e = E2E(e);
        do
        {
            int f = current_e / 3;
            if (f != f0 && f != f1)
            {
                e_one_ring_face.insert(f);
            }
            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);
        } while (current_e != E2E(e));
    }

    activeFace(f0) = false;
    activeFace(f1) = false;
    nV -= 1;
    nF -= 2;

    Vector2r mid = .5 * (V.col(from) + V.col(to));
    V.col(from) = mid;
    V.col(to) = mid;

    for (int f : e_one_ring_face)
    {
        for (int i = 0; i < 3; ++i)
        {
            int v_id = F(i, f);
            if (v_id == from || v_id == to)
            {
                F(i, f) = from;
            }
        }
    }

    E2E(e_from_up) = e_up_to;
    E2E(e_up_to) = e_from_up;
    E2E(e_down_from) = e_to_down;
    E2E(e_to_down) = e_down_from;

    V2E(up) = e_up_to;
    V2E(to) = -2;
    V2E(down) = e_down_from;
    V2E(from) = e_from_up;

    return from;
}

static bool MinMaxSwap2D(
    const Vector2r &pt0,
    const Vector2r &pt1,
    const Vector2r &pt2,
    const Vector2r &pt3)
{
    Real wesp = .001;

    Vector2r v01 = pt1 - pt0;
    Vector2r v02 = pt2 - pt0;
    Vector2r v03 = pt3 - pt0;

    Real area032 = v03[0] * v02[1] - v03[1] * v02[0];

    if (area032 < 1e-12)
    {
        return false;
    }

    Vector2r v12 = pt2 - pt1;
    Vector2r v13 = pt3 - pt1;

    Real area123 = v12[0] * v13[1] - v12[1] * v13[0];

    if (area123 < 1e-12)
    {
        return false;
    }

    v02.normalize();
    v12.normalize();
    v13.normalize();
    v03.normalize();

    Real w012 = v02.dot(v12);
    Real w031 = v13.dot(v03);
    Real w0 = w012 < w031 ? w012 : w031;

    Real w032 = v03.dot(v02);
    Real w123 = v12.dot(v13);
    Real w1 = w032 < w123 ? w032 : w123;
    Real dw = w1 - w0;

    return dw >= wesp;
}

bool IsotropicRemesher::delauneyFlipCondition(int e) const
{
    if (E2E(e) == -1 || e == -1)
    {
        return false;
    }

    int e_n = -1;
    int e_o = -1;
    int e_o_n = -1;

    int va0 = -1;
    int va1 = -1;
    int vb0 = -1;
    int vb1 = -1;

    {
        va1 = dedge_from_vertex(e, F);
        va0 = dedge_to_vertex(e, F);

        e_n = dedge_next_3(e);
        vb0 = dedge_to_vertex(e_n, F);

        e_o = E2E(e);
        e_o_n = dedge_next_3(e_o);
        vb1 = dedge_to_vertex(e_o_n, F);
    }

    Vector2r pa0 = V.col(va0);
    Vector2r pa1 = V.col(va1);
    Vector2r pb0 = V.col(vb0);
    Vector2r pb1 = V.col(vb1);

    if (!MinMaxSwap2D(pa0, pa1, pb1, pb0))
    {
        return false;
    }

    return true;
}

int IsotropicRemesher::findFromToHalfedge(int from, int to) const
{
    if (isBoundaryVertex(from))
    {
        int current_e = V2E(from);

        do
        {
            int c_to = dedge_to_vertex(current_e, F);

            if (c_to == to)
            {
                return current_e;
            }

            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);

        } while (current_e != -1);

        current_e = V2E(from);
        do
        {

            int c_to = dedge_to_vertex(current_e, F);

            if (c_to == to)
            {
                return current_e;
            }

            int c_e_opp = E2E(current_e);
            if (c_e_opp == -1)
            {
                break;
            }
            else
            {
                current_e = dedge_next_3(c_e_opp);
            }
        } while (true);
    }
    else
    {
        int start = V2E(from);
        int current_e = start;
        do
        {

            int c_to = dedge_to_vertex(current_e, F);
            if (c_to == to)
            {
                return current_e;
            }

            int c_e_prev = dedge_prev_3(current_e);
            current_e = E2E(c_e_prev);
        } while (current_e != start);
    }

    return -1;
}

void IsotropicRemesher::run()
{
    flip_to_delauney();
    for (int i = 0; i < 3; ++i)
    {
        split_long_edges();
        collapse_short_edge();
        barycenter_smooth();
        flip_to_delauney();
        renewTopology();// 不renew拓扑有bug, 没时间修了。。。
    }
    barycenter_smooth();
}

static void clearQueue(queue<pair<int, int>> &edges, const pair<int, int> &edge, int v)
{
    std::vector<std::pair<int, int>> garbage;
    garbage.reserve(edges.size());

    while (!edges.empty())
    {
        std::pair<int, int> e = edges.front();
        edges.pop();
        if (e == edge)
        {
            continue;
        }
        garbage.push_back(e);
    }

    for (auto &e : garbage)
    {
        if (e.first == edge.second)
        {
            e.first = v;
        }

        if (e.second == edge.second)
        {
            e.second = v;
        }
        edges.push(e);
    }
}

void IsotropicRemesher::renewTopology()
{
    MatrixXr V_;
    V_.resize(2, nV);
    MatrixXi F_;
    F_.resize(3, nF);

    std::unordered_map<int, int> old_to_new;
    old_to_new.reserve(nV);

    int cnt_v = 0;
    int cnt_f = 0;
    for (int f = 0; f < F.cols(); ++f)
    {
        if (!activeFace(f))
        {
            continue;
        }

        int v0 = F(0, f);
        int v1 = F(1, f);
        int v2 = F(2, f);

        if (v0 < 0 || v1 < 0 || v2 < 0)
        {
            continue;
        }

        int nv0 = -1;
        int nv1 = -1;
        int nv2 = -1;

        if (old_to_new.find(v0) == old_to_new.end())
        {
            nv0 = cnt_v++;
            V_.col(nv0) = V.col(v0);
            old_to_new[v0] = nv0;
        }
        else
        {
            nv0 = old_to_new[v0];
        }

        if (old_to_new.find(v1) == old_to_new.end())
        {
            nv1 = cnt_v++;
            V_.col(nv1) = V.col(v1);
            old_to_new[v1] = nv1;
        }
        else
        {
            nv1 = old_to_new[v1];
        }

        if (old_to_new.find(v2) == old_to_new.end())
        {
            nv2 = cnt_v++;
            V_.col(nv2) = V.col(v2);
            old_to_new[v2] = nv2;
        }
        else
        {
            nv2 = old_to_new[v2];
        }

        F_.col(cnt_f) = Vector3i({nv0, nv1, nv2});
        cnt_f++;
    }
    V_.conservativeResize(2, cnt_v);
    F_.conservativeResize(3, cnt_f);

    F = F_;
    V = V_;

    activeFace.conservativeResize(cnt_f);

    activeFace.setConstant(true);

    build_dedge(F, V, V2E, E2E, isBoundaryVertex, NonManiFold);
}

// bool debug = false;

void IsotropicRemesher::getEdgeList(std::vector<std::pair<int, int>> &edges)
{

    // if (debug)
    // {
    //     std::cout << "splitting check--------------------" << std::endl;
    //     std::cout << F << "\n"
    //               << std::endl;

    //     std::cout << activeFace << "\n"
    //               << std::endl;

    //     std::cout << V << "\n"
    //               << std::endl;
    //     std::cout << E2E << "\n"
    //               << std::endl;
    //     std::cout << V2E << "\n"
    //               << std::endl;
    // }

    std::set<std::pair<int, int>> edges_pre;

    for (int f = 0; f < F.cols(); ++f)
    {
        if (!activeFace(f))
        {
            continue;
        }

        for (int i = 0; i < 3; ++i)
        {

            std::pair<int, int> edge_ordered;

            int head = F(i, f);
            int tail = F((i + 1) % 3, f);

            if (head < tail)
            {
                edge_ordered.first = head;
                edge_ordered.second = tail;
            }
            else
            {
                edge_ordered.first = tail;
                edge_ordered.second = head;
            }

            edges_pre.insert(edge_ordered);
        }
    }

    edges.reserve(edges_pre.size());
    for (const std::pair<int, int> &e : edges_pre)
    {
        int he = findFromToHalfedge(e.first, e.second);
        if (he == -1)
        {
            edges.push_back({e.second, e.first});
        }
        else
        {
            edges.push_back(e);
        }
    }
}

void IsotropicRemesher::getEdgeList(std::queue<std::pair<int, int>> &edges)
{
    std::set<std::pair<int, int>> edges_pre;

    for (int f = 0; f < F.cols(); ++f)
    {
        if (!activeFace(f))
        {
            continue;
        }

        for (int i = 0; i < 3; ++i)
        {

            std::pair<int, int> edge_ordered;

            int head = F(i, f);
            int tail = F((i + 1) % 3, f);

            if (head < tail)
            {
                edge_ordered.first = head;
                edge_ordered.second = tail;
            }
            else
            {
                edge_ordered.first = tail;
                edge_ordered.second = head;
            }

            edges_pre.insert(edge_ordered);
        }
    }

    for (const std::pair<int, int> &e : edges_pre)
    {
        int he = findFromToHalfedge(e.first, e.second);
        if (he == -1)
        {
            edges.push({e.second, e.first});
        }
        else
        {
            edges.push(e);
        }
    }
}

std::pair<std::vector<Vector2r>, std::vector<Vector3i>> IsotropicRemesher::getMesh()
{
    std::pair<std::vector<Vector2r>, std::vector<Vector3i>> mesh;

    std::vector<Vector3i> &Face = mesh.second;
    std::vector<Vector2r> &Vert = mesh.first;

    Face.reserve(nF);
    Vert.reserve(nV);

    std::unordered_map<int, int> old_to_new;
    old_to_new.reserve(nV);

    for (int f = 0; f < F.cols(); ++f)
    {
        if (!activeFace(f))
        {
            continue;
        }

        int v0 = F(0, f);
        int v1 = F(1, f);
        int v2 = F(2, f);

        if (v0 < 0 || v1 < 0 || v2 < 0)
        {
            continue;
        }

        int nv0 = -1;
        int nv1 = -1;
        int nv2 = -1;

        if (old_to_new.find(v0) == old_to_new.end())
        {
            nv0 = Vert.size();
            Vert.push_back(V.col(v0));
            old_to_new[v0] = nv0;
        }
        else
        {
            nv0 = old_to_new[v0];
        }

        if (old_to_new.find(v1) == old_to_new.end())
        {
            nv1 = Vert.size();
            Vert.push_back(V.col(v1));
            old_to_new[v1] = nv1;
        }
        else
        {
            nv1 = old_to_new[v1];
        }

        if (old_to_new.find(v2) == old_to_new.end())
        {
            nv2 = Vert.size();
            Vert.push_back(V.col(v2));
            old_to_new[v2] = nv2;
        }
        else
        {
            nv2 = old_to_new[v2];
        }

        Face.push_back({nv0, nv1, nv2});
    }

    return mesh;
}

void IsotropicRemesher::split_long_edges()
{

    queue<pair<int, int>> edges;
    getEdgeList(edges);

    while (!edges.empty())
    {
        pair<int, int> edge = edges.front();
        edges.pop();

        int from = edge.first;
        int to = edge.second;

        Real e_len = (V.col(to) - V.col(from)).squaredNorm();

        if (e_len > high * high)
        {
            int e = findFromToHalfedge(from, to);
            int new_vert = split_edge(e);

            {
                int current_e = V2E(new_vert);
                int c_e_prev = -1;
                do
                {
                    int c_e_opp = E2E(current_e);

                    int prep = max(c_e_opp, current_e);
                    pair<int, int> edge;
                    edge.first = dedge_from_vertex(prep, F);
                    edge.second = dedge_to_vertex(prep, F);
                    edges.push(edge);

                    c_e_prev = dedge_prev_3(current_e);
                    current_e = E2E(c_e_prev);
                } while (current_e != V2E(new_vert) && current_e != -1);

                if (current_e == -1)
                {
                    pair<int, int> edge;
                    int prep = c_e_prev;
                    edge.first = dedge_from_vertex(prep, F);
                    edge.second = dedge_to_vertex(prep, F);
                    edges.push(edge);
                }
            }
        }
    }
}

void IsotropicRemesher::flip_to_delauney()
{

    std::vector<std::pair<int, int>> edges;
    edges.reserve(E2E.rows());
    getEdgeList(edges);

    int n = (int)edges.size();

    n = n * n;
    int i = 0;

    bool has_flip = false;
    do
    {
        bool flipped_in_this_loop = false;
        for (auto &edge : edges)
        {
            int e = findFromToHalfedge(edge.first, edge.second);
            bool flipped = delauneyFlipCondition(e);
            if (flipped)
            {
                int e_after_flip = flip_edge(e);
                int from = dedge_from_vertex(e_after_flip, F);
                int to = dedge_to_vertex(e_after_flip, F);
                edge.first = from;
                edge.second = to;
            }

            flipped_in_this_loop |= flipped;

            ++i;
        }

        has_flip = flipped_in_this_loop;
    } while (has_flip && i < n);
}

void IsotropicRemesher::collapse_short_edge()
{
    queue<pair<int, int>> edges;
    getEdgeList(edges);

    while (!edges.empty())
    {
        pair<int, int> edge = edges.front();
        edges.pop();
        int e = findFromToHalfedge(edge.first, edge.second);
        if (!isCollapsible(e))
        {
            continue;
        }
        int from = edge.first;
        int to = edge.second;

        Real e_len = (V.col(to) - V.col(from)).norm();

        if (e_len < low)
        {
            int v = collapse_edge(e);
            clearQueue(edges, edge, v);
        }
    }
}

void IsotropicRemesher::barycenter_smooth()
{

    for (int v = 0; v < V.cols(); ++v)
    {
        if (isBoundaryVertex(v) || V2E(v) == -2)
        {
            continue;
        }

        std::vector<int> adj;
        adj.reserve(16);
        {
            int current_e = V2E(v);
            do
            {

                int c_to = dedge_to_vertex(current_e, F);
                adj.push_back(c_to);
                int c_e_prev = dedge_prev_3(current_e);
                current_e = E2E(c_e_prev);
            } while (current_e != V2E(v));
        }

        Vector2r center({0, 0});

        for (auto &v_adj : adj)
        {
            center += V.col(v_adj);
        }
        center /= (Real)adj.size();

        V.col(v) = center;
    }
}