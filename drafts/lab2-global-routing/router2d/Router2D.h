#pragma once

#include "ispdData.h"

#include <chrono>
#include <cstdint>
#include <vector>

namespace Router {

struct EdgeInfo {
    int dem = 0;
    int cap = 0;
    float hist = 0.0f;
};

struct SubNet {
    int netIdx;
    int subIdx;
    int x1, y1, x2, y2;
    int hpwl;
    int lastLen;
    // Direction-aware bbox extension. Grows along the orthogonal direction
    // of where this subnet's path is hitting overflow edges.
    int xExtend = 6;
    int yExtend = 6;
    int stuckCount = 0;  // consecutive iters this subnet stayed OF
    bool overflow = false;
    bool ripup = false;
};

class Router2D {
public:
    Router2D(ISPDParser::ispdData* d, int timeLimitSec);
    void run();

private:
    ISPDParser::ispdData* m_d;
    int m_W, m_H;
    int m_demUnit;
    int m_timeLimit;
    std::chrono::steady_clock::time_point m_start;

    std::vector<EdgeInfo> m_h, m_v;
    std::vector<SubNet> m_subs;

    int m_iter = 0;
    double m_kHist = 2.0;
    double m_kCong = 4.0;
    double m_histInc = 1.5;

    std::vector<double> m_dist;
    std::vector<int> m_visit;
    std::vector<int8_t> m_par;
    std::vector<int> m_plen;
    int m_curGen = 0;

    // Shared scratch buffers for monotonicRoute / humRoute DPs. Allocated
    // once to W*H and reused per call (per-call fill of the [0..N) prefix
    // that the call uses, where N depends on bbox).
    std::vector<double> m_dpBuf;
    std::vector<int8_t> m_parBuf;

    // RSMT-aware routing: edges already used by other subnets of the
    // currently-routed net get cost discount (treat as already-built tree).
    std::vector<int> m_netEdgeGen;  // size 2 * W * H (h then v block)
    int m_netEdgeCurGen = 0;
    int m_curNetIdx = -1;

    inline int hIdx(int x, int y) const { return y * (m_W - 1) + x; }
    inline int vIdx(int x, int y) const { return y * m_W + x; }
    inline int cIdx(int x, int y) const { return y * m_W + x; }
    inline int hEdgeId(int x, int y) const { return y * (m_W - 1) + x; }
    inline int vEdgeId(int x, int y) const { return (m_W - 1) * m_H + y * m_W + x; }
    bool isSharedEdge(int eid) const {
        return m_netEdgeGen[eid] == m_netEdgeCurGen;
    }
    void markNetEdges(int netIdx, int excludeSubIdx);

    double cellCost(int dem, int cap, float hist) const;
    double cellCostShared(int dem, int cap, float hist, int eid) const;

    void buildCapacity();
    void decomposeNets();
    void initialRoute();
    void nrrLoop();
    void refinement();

    // Add Steiner points to pin set via iterated 1-Steiner heuristic.
    // For n in [4, maxN] only. Modifies pins in-place.
    void steinerizePins(std::vector<ISPDParser::Point>& pins,
                        int maxN = 9, int maxAdded = 4);

    // Demand counting is per-net-deduplicated: edges shared by multiple
    // subnets of the same net contribute demand only once (matches the
    // BFS dedup that LayerAssignment::convertGRtoLA does on output).
    void addDemandPath(const std::vector<ISPDParser::RPoint>& path);
    void removeDemandPath(const std::vector<ISPDParser::RPoint>& path);
    bool pathHasOverflow(const std::vector<ISPDParser::RPoint>& path) const;

    void patternL(int x1, int y1, int x2, int y2,
                  std::vector<ISPDParser::RPoint>& out);
    bool monotonicRoute(int x1, int y1, int x2, int y2,
                        std::vector<ISPDParser::RPoint>& out);
    bool mazeRoute(int sx, int sy, int tx, int ty, int lenBound,
                   std::vector<ISPDParser::RPoint>& out,
                   int xExtend = 0, int yExtend = 0);
    // HUM-style: bidirectional DP within bbox, find best meeting point.
    // Far cheaper than mazeRoute (no priority queue), used as primary
    // reroute method. mazeRoute kept as fallback for stubborn nets.
    bool humRoute(int sx, int sy, int tx, int ty,
                  int xExtend, int yExtend,
                  std::vector<ISPDParser::RPoint>& out);

    int64_t totalOverflow() const;
    int maxOverflow() const;
    int64_t totalEdges() const;
    void updateHistory();

    double elapsedSec() const;
    bool timeUp(double margin = 60.0) const;
};

} // namespace Router
