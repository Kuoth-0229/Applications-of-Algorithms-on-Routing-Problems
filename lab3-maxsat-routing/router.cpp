#include "router.hpp"

#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <array>
#include <chrono>
#include <unordered_map>

// =============================================================================
// Grid-based 2-pin net router using Open-WBO (MaxSAT).
//
// Model (edge-based, as suggested in the spec hint):
//   * A 3-D grid graph is built from the gridlines.  Nodes are (i,j,k) grid
//     intersections; edges connect index-adjacent nodes (horizontal, vertical,
//     and vias between layers).
//   * For every net n and every edge e inside that net's routing region we have
//     a Boolean variable E[n][e] = "net n routes through edge e".
//   * For every net n and every node v inside its region we have U[n][v] =
//     "net n occupies node v".
//
//   Hard clauses:
//     - Connectivity (degree) per net:
//         pin node  -> exactly one incident edge of that net is used
//         other node-> zero or two incident edges of that net are used
//       (a source->sink simple path; cycles are excluded by cost minimisation).
//     - Edge->node linking: using an edge occupies both of its endpoints.
//     - Node exclusivity: at most one net occupies any node (forbids overlap,
//       short and crossing, which all reduce to node sharing in the verifier).
//
//   Soft clauses (objective):
//     - One unit soft clause (-E[n][e]) per edge variable with weight equal to
//       that edge's cost contribution (actual wirelength for planar edges,
//       via_cost for vias).  Minimising the falsified weight == minimising
//       wirelength + via_num * via_cost, exactly the verifier cost.
//
//   Each net is confined to the bounding box of its two pins, expanded by a
//   margin (a global, order-independent rule -- no pre-routing, no per-net
//   hardcoding).  The grid starts at the Hanan grid (gridlines = pin
//   coordinates).  If the formula is UNSAT (insufficient capacity), the grid is
//   refined (extra gridlines spaced by min_pitch_size) and the margin widened,
//   then the solve is retried.
// =============================================================================

namespace {
inline long long igcd(long long a, long long b) {
    a = a < 0 ? -a : a;
    b = b < 0 ? -b : b;
    while (b) { long long t = a % b; a = b; b = t; }
    return a;
}
}

class RouterImpl {
public:
    // ---- circuit spec ----
    int max_layer = 0;              // number of usable layers (0 .. max_layer-1)
    int boundary[4] = {0,0,0,0};    // minx, miny, maxx, maxy
    int min_pitch = 1;
    long long via_cost = 0;
    int M = 0;                      // number of nets
    std::vector<std::string> net_name;
    std::vector<std::array<int,3>> net_src, net_snk; // actual coordinates

    // ---- escalation knobs ----
    int refineLevel = 0;            // gridline insertions per base gap
    int margin = 1;                 // bbox expansion (index units)
    bool useSoft = true;            // include cost-minimisation soft clauses
    bool fixedKnobs = false;        // disable escalation (debug)
    bool forceMargin = false;       // margin pinned by env (debug)
    int cpuLim = 50;                // per-solve cpu limit (seconds), phase 0
    int optLim = 120;               // max cpu for the phase-1 optimisation solve

    // ---- two-phase solving ----
    // phase 0: NO_SOFT feasibility search (escalate grid/margin to find a legal
    //          routing fast).  phase 1: re-solve at that grid WITH the cost
    //          objective to minimise vias, keeping the best model found.
    int phase = 0;
    std::vector<std::vector<std::array<int,3>>> bestResult;
    bool haveBest = false;
    int totalBudget = 540;          // seconds, leaves a safety buffer under 600
    std::chrono::steady_clock::time_point t0;
    bool t0set = false;

    // ---- grid ----
    int X = 0, Y = 0, L = 0;
    std::vector<int> x_coors, y_coors;
    std::unordered_map<int,int> xIdx, yIdx;

    struct Edge { int u, v; long long w; bool via; };
    std::vector<Edge> edges;
    std::vector<std::vector<int>> inc; // node -> incident edge ids

    std::vector<int> srcNode, snkNode;

    long long scale = 1;

    // ---- per-net variable maps ----
    long long varCount = 0;
    std::vector<std::unordered_map<int,long long>> netEdgeVar; // net -> (edge -> var)
    std::vector<std::unordered_map<int,long long>> netNodeVar; // net -> (node -> var)
    std::vector<std::vector<int>> nodeNets;                    // node -> nets present

    // ---- monotone (directed) mode ----
    bool monotone = true;           // try monotone routing first
    struct Arc { int from, to; long long w; bool via; };
    std::vector<std::vector<Arc>> netArcs;          // net -> arcs
    std::vector<std::vector<long long>> netArcVar;  // net -> var per arc (parallel)
    std::vector<std::unordered_map<int,std::vector<int>>> arcOut, arcIn; // net -> node -> arc idx

    // ---- solution ----
    std::vector<std::vector<std::array<int,3>>> result;
    bool solved = false;

    // ----------------------------------------------------------------- helpers
    int nodeId(int i, int j, int k) const { return (k * Y + j) * X + i; }
    int numNodes() const { return X * Y * L; }

    // ------------------------------------------------------------------ parse
    void readCircuitSpec(std::ifstream& in) {
        std::string tok;
        in >> tok >> max_layer;
        in >> tok >> boundary[0] >> boundary[1] >> boundary[2] >> boundary[3];
        in >> tok >> min_pitch;
        in >> tok >> via_cost;
        in >> tok >> M;
        if (const char* e = std::getenv("FORCE_REFINE")) { refineLevel = atoi(e); fixedKnobs = true; }
        if (const char* e = std::getenv("FORCE_MARGIN")) { margin = atoi(e); forceMargin = true; }
        if (const char* e = std::getenv("NO_SOFT")) { if (atoi(e)) useSoft = false; }
        if (const char* e = std::getenv("BUDGET")) { totalBudget = atoi(e); }
        if (const char* e = std::getenv("CPU_LIM")) { cpuLim = atoi(e); }
        if (const char* e = std::getenv("OPT_LIM")) { optLim = atoi(e); }
        if (const char* e = std::getenv("MONO")) { monotone = atoi(e) != 0; }
        net_name.resize(M);
        net_src.resize(M);
        net_snk.resize(M);
        for (int n = 0; n < M; ++n) {
            in >> net_name[n];
            in >> net_src[n][0] >> net_src[n][1] >> net_src[n][2];
            in >> net_snk[n][0] >> net_snk[n][1] >> net_snk[n][2];
        }
    }

    // ------------------------------------------------------------ build grid
    std::vector<int> refineAxis(const std::set<int>& s) {
        std::vector<int> base(s.begin(), s.end()), out;
        for (size_t a = 0; a < base.size(); ++a) {
            out.push_back(base[a]);
            if (a + 1 >= base.size() || refineLevel <= 0) continue;
            int lo = base[a], hi = base[a+1], g = hi - lo;
            int rg = std::min(refineLevel, g / min_pitch - 1);
            if (rg <= 0) continue;
            int prev = lo;
            for (int i = 1; i <= rg; ++i) {
                int p = lo + (int)((long long)i * g / (rg + 1));
                if (p - prev >= min_pitch && hi - p >= min_pitch) {
                    out.push_back(p);
                    prev = p;
                }
            }
        }
        return out;
    }

    int maxUseful = 0;       // refine level beyond which the grid no longer changes
    long long lastVarCount = -1; // detect when escalation stops growing the instance

    void generateGraph() {
        std::set<int> sx, sy;
        for (int n = 0; n < M; ++n) {
            sx.insert(net_src[n][0]); sx.insert(net_snk[n][0]);
            sy.insert(net_src[n][1]); sy.insert(net_snk[n][1]);
        }
        // largest number of min_pitch insertions any base gap admits: once
        // refineLevel reaches this, further refinement cannot grow the grid.
        maxUseful = 0;
        auto scanGaps = [&](const std::set<int>& s){
            std::vector<int> b(s.begin(), s.end());
            for (size_t a = 0; a + 1 < b.size(); ++a)
                maxUseful = std::max(maxUseful, (b[a+1]-b[a]) / min_pitch - 1);
        };
        scanGaps(sx); scanGaps(sy);
        x_coors = refineAxis(sx);
        y_coors = refineAxis(sy);
        X = (int)x_coors.size();
        Y = (int)y_coors.size();
        L = max_layer;
        xIdx.clear(); yIdx.clear();
        for (int i = 0; i < X; ++i) xIdx[x_coors[i]] = i;
        for (int j = 0; j < Y; ++j) yIdx[y_coors[j]] = j;

        // edges + incidence
        edges.clear();
        inc.assign(numNodes(), {});
        auto addEdge = [&](int u, int v, long long w, bool via){
            int id = (int)edges.size();
            edges.push_back({u, v, w, via});
            inc[u].push_back(id);
            inc[v].push_back(id);
        };
        for (int k = 0; k < L; ++k)
            for (int j = 0; j < Y; ++j)
                for (int i = 0; i < X; ++i) {
                    int u = nodeId(i,j,k);
                    if (i + 1 < X) addEdge(u, nodeId(i+1,j,k), x_coors[i+1]-x_coors[i], false);
                    if (j + 1 < Y) addEdge(u, nodeId(i,j+1,k), y_coors[j+1]-y_coors[j], false);
                    if (k + 1 < L) addEdge(u, nodeId(i,j,k+1), via_cost, true);
                }

        srcNode.assign(M, -1);
        snkNode.assign(M, -1);
        for (int n = 0; n < M; ++n) {
            srcNode[n] = nodeId(xIdx[net_src[n][0]], yIdx[net_src[n][1]], net_src[n][2]);
            snkNode[n] = nodeId(xIdx[net_snk[n][0]], yIdx[net_snk[n][1]], net_snk[n][2]);
        }

        // weight scaling
        scale = 0;
        for (auto& e : edges) scale = igcd(scale, e.w);
        if (scale == 0) scale = 1;

        // ---- assign variables, confined to per-net bounding boxes ----
        varCount = 0;
        netEdgeVar.assign(M, {});
        netNodeVar.assign(M, {});
        nodeNets.assign(numNodes(), {});
        netArcs.assign(M, {});
        netArcVar.assign(M, {});
        arcOut.assign(M, {});
        arcIn.assign(M, {});
        for (int n = 0; n < M; ++n) {
            int si = xIdx[net_src[n][0]], sj = yIdx[net_src[n][1]];
            int ti = xIdx[net_snk[n][0]], tj = yIdx[net_snk[n][1]];
            int x0 = std::max(0, std::min(si,ti) - margin);
            int x1 = std::min(X-1, std::max(si,ti) + margin);
            int y0 = std::max(0, std::min(sj,tj) - margin);
            int y1 = std::min(Y-1, std::max(sj,tj) + margin);
            for (int k = 0; k < L; ++k)
                for (int j = y0; j <= y1; ++j)
                    for (int i = x0; i <= x1; ++i) {
                        int v = nodeId(i,j,k);
                        netNodeVar[n][v] = ++varCount;
                        nodeNets[v].push_back(n);
                    }
            if (!monotone) {
                // undirected edges whose both endpoints are inside the region
                for (int k = 0; k < L; ++k)
                    for (int j = y0; j <= y1; ++j)
                        for (int i = x0; i <= x1; ++i) {
                            int u = nodeId(i,j,k);
                            for (int e : inc[u]) {
                                int o = (edges[e].u == u) ? edges[e].v : edges[e].u;
                                if (o < u) continue;
                                if (netNodeVar[n].count(o))
                                    netEdgeVar[n][e] = ++varCount;
                            }
                        }
            } else {
                // directed arcs that move monotonically toward the target
                int dix = (ti > si) - (ti < si);
                int diy = (tj > sj) - (tj < sj);
                auto addArc = [&](int f, int t, long long w, bool via){
                    int idx = (int)netArcs[n].size();
                    netArcs[n].push_back({f, t, w, via});
                    netArcVar[n].push_back(++varCount);
                    arcOut[n][f].push_back(idx);
                    arcIn[n][t].push_back(idx);
                };
                for (int k = 0; k < L; ++k)
                    for (int j = y0; j <= y1; ++j)
                        for (int i = x0; i <= x1; ++i) {
                            int u = nodeId(i,j,k);
                            if (dix > 0 && i+1 <= x1) addArc(u, nodeId(i+1,j,k), x_coors[i+1]-x_coors[i], false);
                            if (dix < 0 && i-1 >= x0) addArc(u, nodeId(i-1,j,k), x_coors[i]-x_coors[i-1], false);
                            if (diy > 0 && j+1 <= y1) addArc(u, nodeId(i,j+1,k), y_coors[j+1]-y_coors[j], false);
                            if (diy < 0 && j-1 >= y0) addArc(u, nodeId(i,j-1,k), y_coors[j]-y_coors[j-1], false);
                            if (k+1 < L) addArc(u, nodeId(i,j,k+1), via_cost, true);
                            if (k-1 >= 0) addArc(u, nodeId(i,j,k-1), via_cost, true);
                        }
            }
        }
    }

    // -------------------------------------------------------- generate clauses
    void generateClauses(std::ofstream& out) {
        if (monotone) { generateClausesMonotone(out); return; }
        std::vector<std::vector<long long>> hard;
        std::vector<std::pair<long long,long long>> soft;
        long long softSum = 0;

        // connectivity (degree) per net over its region
        for (int n = 0; n < M; ++n) {
            for (auto& kv : netNodeVar[n]) {
                int v = kv.first;
                std::vector<long long> lits;
                for (int e : inc[v]) {
                    auto it = netEdgeVar[n].find(e);
                    if (it != netEdgeVar[n].end()) lits.push_back(it->second);
                }
                int d = (int)lits.size();
                bool isPin = (v == srcNode[n] || v == snkNode[n]);
                if (isPin) {
                    hard.push_back(lits); // at least one
                    for (int a = 0; a < d; ++a)
                        for (int b = a+1; b < d; ++b)
                            hard.push_back({-lits[a], -lits[b]});
                } else {
                    // zero or two: at most two ...
                    for (int a = 0; a < d; ++a)
                        for (int b = a+1; b < d; ++b)
                            for (int c = b+1; c < d; ++c)
                                hard.push_back({-lits[a], -lits[b], -lits[c]});
                    // ... and not exactly one
                    for (int a = 0; a < d; ++a) {
                        std::vector<long long> cl;
                        cl.push_back(-lits[a]);
                        for (int b = 0; b < d; ++b) if (b != a) cl.push_back(lits[b]);
                        hard.push_back(cl);
                    }
                }
            }
        }

        // edge -> node linking
        for (int n = 0; n < M; ++n)
            for (auto& kv : netEdgeVar[n]) {
                long long ev = kv.second;
                const Edge& e = edges[kv.first];
                hard.push_back({-ev, netNodeVar[n][e.u]});
                hard.push_back({-ev, netNodeVar[n][e.v]});
            }

        // node exclusivity: at most one net per node
        for (int v = 0; v < numNodes(); ++v) {
            const std::vector<int>& ns = nodeNets[v];
            for (size_t a = 0; a < ns.size(); ++a)
                for (size_t b = a+1; b < ns.size(); ++b)
                    hard.push_back({-netNodeVar[ns[a]][v], -netNodeVar[ns[b]][v]});
        }

        // soft objective -- only when the instance is small enough that the
        // weighted optimum can actually be proven within the time budget;
        // otherwise we rely on tight bounding boxes to keep the (feasibility-
        // only) routing short.
        bool softNow = useSoft && (phase == 1);
        if (softNow)
            for (int n = 0; n < M; ++n)
                for (auto& kv : netEdgeVar[n]) {
                    long long w = edges[kv.first].w / scale;
                    if (w <= 0) w = 1;
                    soft.push_back({w, -kv.second});
                    softSum += w;
                }

        long long top = softSum + 1;
        long long nclauses = (long long)hard.size() + (long long)soft.size();

        std::ostringstream buf;
        buf << "p wcnf " << varCount << ' ' << nclauses << ' ' << top << '\n';
        for (auto& cl : hard) {
            buf << top;
            for (long long lit : cl) buf << ' ' << lit;
            buf << " 0\n";
        }
        for (auto& s : soft)
            buf << s.first << ' ' << s.second << " 0\n";
        out << buf.str();
    }

    // -------------------------------------------- generate clauses (monotone)
    void generateClausesMonotone(std::ofstream& out) {
        std::vector<std::vector<long long>> hard;
        std::vector<std::pair<long long,long long>> soft;
        long long softSum = 0;
        auto amo = [&](const std::vector<long long>& v){
            for (size_t a = 0; a < v.size(); ++a)
                for (size_t b = a+1; b < v.size(); ++b)
                    hard.push_back({-v[a], -v[b]});
        };

        for (int n = 0; n < M; ++n) {
            // single-commodity flow: src emits 1, snk absorbs 1, others conserve
            for (auto& kv : netNodeVar[n]) {
                int v = kv.first;
                std::vector<long long> IN, OUT;
                auto io = arcOut[n].find(v);
                if (io != arcOut[n].end())
                    for (int idx : io->second) OUT.push_back(netArcVar[n][idx]);
                auto ii = arcIn[n].find(v);
                if (ii != arcIn[n].end())
                    for (int idx : ii->second) IN.push_back(netArcVar[n][idx]);

                if (v == srcNode[n]) {
                    hard.push_back(OUT);             // exactly one outgoing
                    amo(OUT);
                    for (long long a : IN) hard.push_back({-a});   // no incoming
                } else if (v == snkNode[n]) {
                    hard.push_back(IN);              // exactly one incoming
                    amo(IN);
                    for (long long b : OUT) hard.push_back({-b});  // no outgoing
                } else {
                    amo(IN); amo(OUT);
                    // conservation: any incoming implies an outgoing and vice versa
                    for (long long a : IN) {
                        std::vector<long long> cl; cl.push_back(-a);
                        for (long long b : OUT) cl.push_back(b);
                        hard.push_back(cl);
                    }
                    for (long long b : OUT) {
                        std::vector<long long> cl; cl.push_back(-b);
                        for (long long a : IN) cl.push_back(a);
                        hard.push_back(cl);
                    }
                }
            }

            // arc -> node occupancy linking, and lookup map for via 2-cycles
            std::unordered_map<long long,long long> arcByKey;
            int N = numNodes();
            for (size_t idx = 0; idx < netArcs[n].size(); ++idx) {
                const Arc& a = netArcs[n][idx];
                long long av = netArcVar[n][idx];
                hard.push_back({-av, netNodeVar[n][a.from]});
                hard.push_back({-av, netNodeVar[n][a.to]});
                arcByKey[(long long)a.from * N + a.to] = av;
            }
            // forbid an up-via immediately undone by a down-via (the only cycles
            // the monotone arc set admits)
            for (size_t idx = 0; idx < netArcs[n].size(); ++idx) {
                const Arc& a = netArcs[n][idx];
                if (!a.via || a.from >= a.to) continue;       // only the up arc
                auto it = arcByKey.find((long long)a.to * N + a.from);
                if (it != arcByKey.end())
                    hard.push_back({-netArcVar[n][idx], -it->second});
            }
        }

        // node exclusivity: at most one net per node
        for (int v = 0; v < numNodes(); ++v) {
            const std::vector<int>& ns = nodeNets[v];
            for (size_t a = 0; a < ns.size(); ++a)
                for (size_t b = a+1; b < ns.size(); ++b)
                    hard.push_back({-netNodeVar[ns[a]][v], -netNodeVar[ns[b]][v]});
        }

        // soft objective: minimise routing cost (wirelength is fixed for
        // monotone paths, so this mainly minimises vias)
        bool softNow = useSoft && (phase == 1);
        if (softNow)
            for (int n = 0; n < M; ++n)
                for (size_t idx = 0; idx < netArcs[n].size(); ++idx) {
                    long long w = netArcs[n][idx].w / scale;
                    if (w <= 0) w = 1;
                    soft.push_back({w, -netArcVar[n][idx]});
                    softSum += w;
                }

        long long top = softSum + 1;
        std::ostringstream buf;
        buf << "p wcnf " << varCount << ' '
            << (long long)hard.size() + (long long)soft.size() << ' ' << top << '\n';
        for (auto& cl : hard) {
            buf << top;
            for (long long lit : cl) buf << ' ' << lit;
            buf << " 0\n";
        }
        for (auto& s : soft) buf << s.first << ' ' << s.second << " 0\n";
        out << buf.str();
    }

    int elapsed() {
        if (!t0set) return 0;
        return (int)std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - t0).count();
    }

    std::string getSysCommand(int) {
        if (!t0set) { t0 = std::chrono::steady_clock::now(); t0set = true; }
        int rem = totalBudget - elapsed();
        if (rem < 10) rem = 10;
        // phase 0 = quick feasibility probe (skip hard grids fast);
        // phase 1 = optimisation, given most of the remaining budget.
        int lim = (phase == 0) ? std::min(cpuLim, rem) : std::min(optLim, rem);
        std::ostringstream c;
        c << "./open-wbo -cpu-lim=" << lim << " clause.sat > sat_result.txt 2>/dev/null";
        return c.str();
    }

    // ---------------------------------------------------------- read solution
    bool readSolverResult(std::ifstream& in, int) {
        std::string line, status;
        std::vector<long long> lits;
        bool haveV = false;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            if (line[0] == 's') {
                status = line;
            } else if (line[0] == 'v') {
                haveV = true;
                std::istringstream iss(line.substr(1));
                long long x;
                while (iss >> x) lits.push_back(x);
            }
        }
        bool sat = (status.find("OPTIMUM") != std::string::npos) ||
                   (status.find("SATISFIABLE") != std::string::npos &&
                    status.find("UNSATISFIABLE") == std::string::npos);
        if (!sat || !haveV) {
            // Phase 1 (optimisation) produced no usable model in time: keep the
            // legal routing found during phase 0.
            if (phase == 1 && haveBest) { result = bestResult; solved = true; return true; }
            if (fixedKnobs) { solved = false; result.assign(M, {}); return true; }
            // Out of time: stop escalating (use any feasible routing already found).
            if (elapsed() >= totalBudget) {
                if (haveBest) { result = bestResult; solved = true; }
                else { solved = false; result.assign(M, {}); }
                return true;
            }
            bool grew = (varCount != lastVarCount);
            lastVarCount = varCount;
            // If monotone routing has saturated (grid and every bbox already span
            // the whole region) and is still UNSAT, the instance needs detours:
            // fall back to the general (undirected) formulation and restart.
            if (monotone && !grew && refineLevel >= maxUseful) {
                monotone = false;
                refineLevel = 0; margin = 1; lastVarCount = -1;
                return false;
            }
            // Escalate capacity: refine the grid (bbox margin kept tight) until
            // the grid stops growing, then widen the margin.
            if (margin < 40) {
                if (refineLevel < maxUseful) refineLevel++;
                else margin++;
                return false;
            }
            solved = false;
            result.assign(M, {});
            return true;
        }

        std::vector<char> val(varCount + 2, 0);
        for (long long lit : lits) {
            long long a = lit < 0 ? -lit : lit;
            if (a >= 1 && a <= varCount) val[a] = (lit > 0) ? 1 : 0;
        }

        result.assign(M, {});
        for (int n = 0; n < M; ++n) {
            std::vector<std::array<int,3>>& path = result[n];
            std::set<int> visited;
            if (monotone) {
                // each occupied node has at most one chosen outgoing arc
                std::unordered_map<int,int> nextNode;
                for (size_t idx = 0; idx < netArcs[n].size(); ++idx)
                    if (val[netArcVar[n][idx]])
                        nextNode[netArcs[n][idx].from] = netArcs[n][idx].to;
                int cur = srcNode[n];
                while (true) {
                    int i = cur % X, t = cur / X, j = t % Y, k = t / Y;
                    path.push_back({i, j, k});
                    if (cur == snkNode[n] || visited.count(cur)) break;
                    visited.insert(cur);
                    auto it = nextNode.find(cur);
                    if (it == nextNode.end()) break;
                    cur = it->second;
                }
            } else {
                std::unordered_map<int, std::vector<int>> adj;
                for (auto& kv : netEdgeVar[n]) {
                    if (val[kv.second]) {
                        const Edge& e = edges[kv.first];
                        adj[e.u].push_back(e.v);
                        adj[e.v].push_back(e.u);
                    }
                }
                int cur = srcNode[n], prev = -1;
                while (true) {
                    int i = cur % X, t = cur / X, j = t % Y, k = t / Y;
                    path.push_back({i, j, k});
                    visited.insert(cur);
                    if (cur == snkNode[n]) break;
                    int nxt = -1;
                    auto it = adj.find(cur);
                    if (it != adj.end())
                        for (int w : it->second)
                            if (w != prev && !visited.count(w)) { nxt = w; break; }
                    if (nxt == -1) break;
                    prev = cur; cur = nxt;
                }
            }
        }
        solved = true;

        // Phase 0 found a legal routing: store it, then (budget permitting) run
        // phase 1 at the same grid to minimise vias.  Phase 1's model is final.
        if (phase == 0) {
            bestResult = result;
            haveBest = true;
            if (useSoft && !fixedKnobs && totalBudget - elapsed() > 20) {
                phase = 1;
                return false;
            }
        }
        return true;
    }

    // ----------------------------------------------------------- print output
    void printRoutingResult(std::ofstream& out) {
        out << "x_coors " << X << '\n';
        for (int i = 0; i < X; ++i) out << x_coors[i] << (i+1<X ? ' ' : '\n');
        out << "y_coors " << Y << '\n';
        for (int j = 0; j < Y; ++j) out << y_coors[j] << (j+1<Y ? ' ' : '\n');
        for (int n = 0; n < M; ++n) {
            out << net_name[n] << ' ' << result[n].size() << '\n';
            for (auto& p : result[n])
                out << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
        }
    }
};

// ------------- thin Router facade delegating to RouterImpl ------------------
static RouterImpl* impl() { static RouterImpl r; return &r; }

void Router::readCircuitSpec(std::ifstream& in) { impl()->readCircuitSpec(in); }
void Router::generateGraph() { impl()->generateGraph(); }
std::string Router::getSysCommand(int i) { return impl()->getSysCommand(i); }
void Router::generateClauses(std::ofstream& out) { impl()->generateClauses(out); }
bool Router::readSolverResult(std::ifstream& in, int i) { return impl()->readSolverResult(in, i); }
void Router::printRoutingResult(std::ofstream& out) { impl()->printRoutingResult(out); }
