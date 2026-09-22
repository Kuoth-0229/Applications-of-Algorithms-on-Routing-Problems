#include "Router2D.h"

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <tuple>

namespace Router {

using ISPDParser::Point;
using ISPDParser::RPoint;
using ISPDParser::TwoPin;
using std::vector;

Router2D::Router2D(ISPDParser::ispdData* d, int timeLimitSec)
    : m_d(d), m_W(d->numXGrid), m_H(d->numYGrid), m_timeLimit(timeLimitSec) {
    m_start = std::chrono::steady_clock::now();
    int mw = d->minimumWidth.empty() ? 1 : d->minimumWidth[0];
    int ms = d->minimumSpacing.empty() ? 1 : d->minimumSpacing[0];
    m_demUnit = mw + ms;
    if (m_demUnit <= 0) m_demUnit = 1;
}

double Router2D::elapsedSec() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - m_start)
        .count();
}

bool Router2D::timeUp(double margin) const {
    return elapsedSec() > (double)m_timeLimit - margin;
}

void Router2D::buildCapacity() {
    m_h.assign((size_t)(m_W - 1) * m_H, EdgeInfo{});
    m_v.assign((size_t)m_W * (m_H - 1), EdgeInfo{});

    int totalH = 0, totalV = 0;
    for (int z = 0; z < m_d->numLayer; ++z) {
        totalH += m_d->horizontalCapacity[z];
        totalV += m_d->verticalCapacity[z];
    }
    for (auto& e : m_h) e.cap = totalH;
    for (auto& e : m_v) e.cap = totalV;

    for (auto* adj : m_d->capacityAdjs) {
        int x1 = std::get<0>(adj->grid1);
        int y1 = std::get<1>(adj->grid1);
        int z1 = std::get<2>(adj->grid1);
        int x2 = std::get<0>(adj->grid2);
        int y2 = std::get<1>(adj->grid2);
        int z2 = std::get<2>(adj->grid2);
        int newCap = adj->reducedCapacityLevel;
        z1 -= 1;
        z2 -= 1;
        if (x1 != x2) {
            int origCap = m_d->horizontalCapacity[z1];
            int xl = std::min(x1, x2);
            if (xl >= 0 && xl < m_W - 1 && y1 >= 0 && y1 < m_H)
                m_h[hIdx(xl, y1)].cap += (newCap - origCap);
        } else if (y1 != y2) {
            int origCap = m_d->verticalCapacity[z1];
            int yb = std::min(y1, y2);
            if (x1 >= 0 && x1 < m_W && yb >= 0 && yb < m_H - 1)
                m_v[vIdx(x1, yb)].cap += (newCap - origCap);
        }
    }

    m_dist.assign((size_t)m_W * m_H, 0.0);
    m_visit.assign((size_t)m_W * m_H, -1);
    m_par.assign((size_t)m_W * m_H, 0);
    m_plen.assign((size_t)m_W * m_H, 0);

    // Per-net edge marker, allocated once. Index range: [0, hEdges) for hori
    // and [(W-1)*H, (W-1)*H + W*(H-1)) for vert.
    m_netEdgeGen.assign((size_t)(m_W - 1) * m_H + (size_t)m_W * (m_H - 1), -1);
    m_netEdgeCurGen = 0;

    // Reusable DP scratch for monotonicRoute / humRoute.
    m_dpBuf.assign((size_t)m_W * m_H, 0.0);
    m_parBuf.assign((size_t)m_W * m_H, (int8_t)-1);
}

void Router2D::steinerizePins(std::vector<Point>& pins, int maxN, int maxAdded) {
    int n = (int)pins.size();
    if (n < 4 || n > maxN) return;

    auto mstCost = [](const std::vector<Point>& pts) -> int {
        int k = (int)pts.size();
        if (k <= 1) return 0;
        std::vector<bool> inT(k, false);
        std::vector<int> minD(k, INT_MAX);
        inT[0] = true;
        int total = 0;
        for (int j = 1; j < k; ++j)
            minD[j] = std::abs(pts[0].x - pts[j].x) + std::abs(pts[0].y - pts[j].y);
        for (int it = 1; it < k; ++it) {
            int u = -1, best = INT_MAX;
            for (int j = 0; j < k; ++j)
                if (!inT[j] && minD[j] < best) { best = minD[j]; u = j; }
            if (u == -1) break;
            inT[u] = true;
            total += best;
            for (int j = 0; j < k; ++j) if (!inT[j]) {
                int d = std::abs(pts[u].x - pts[j].x) + std::abs(pts[u].y - pts[j].y);
                if (d < minD[j]) minD[j] = d;
            }
        }
        return total;
    };

    int curCost = mstCost(pins);
    for (int round = 0; round < maxAdded; ++round) {
        // Hanan grid: all (xi, yj) for xi in pinXs, yj in pinYs
        std::vector<int> xs, ys;
        for (auto& p : pins) { xs.push_back(p.x); ys.push_back(p.y); }
        std::sort(xs.begin(), xs.end()); xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
        std::sort(ys.begin(), ys.end()); ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

        int bestSavings = 0;
        Point bestSteiner(-1, -1);
        std::vector<Point> aug;
        aug.reserve(pins.size() + 1);
        for (int x : xs) for (int y : ys) {
            bool isPin = false;
            for (auto& p : pins) if (p.x == x && p.y == y) { isPin = true; break; }
            if (isPin) continue;

            aug = pins;
            aug.emplace_back(x, y);
            int newCost = mstCost(aug);
            int savings = curCost - newCost;
            if (savings > bestSavings) {
                bestSavings = savings;
                bestSteiner = Point(x, y);
            }
        }
        if (bestSavings <= 0) break;
        pins.push_back(bestSteiner);
        curCost -= bestSavings;
    }
}

void Router2D::decomposeNets() {
    int totalSubs = 0;
    for (size_t ni = 0; ni < m_d->nets.size(); ++ni) {
        auto* net = m_d->nets[ni];
        auto& pins = net->pin2D;
        int n = (int)pins.size();
        if (n < 2) continue;

        net->twopin.clear();

        if (n == 2) {
            net->twopin.emplace_back();
            auto& tp = net->twopin.back();
            tp.from = Point(pins[0].x, pins[0].y);
            tp.to = Point(pins[1].x, pins[1].y);
            tp.parNet = net;
            tp.wlen = std::abs(pins[0].x - pins[1].x) +
                      std::abs(pins[0].y - pins[1].y);
        } else if (n == 3) {
            // Optimal RSMT for 3-pin: Steiner point at (medianX, medianY)
            int xs[3] = {pins[0].x, pins[1].x, pins[2].x};
            int ys[3] = {pins[0].y, pins[1].y, pins[2].y};
            std::sort(xs, xs + 3);
            std::sort(ys, ys + 3);
            int sx = xs[1], sy = ys[1];
            for (int j = 0; j < 3; ++j) {
                if (pins[j].x == sx && pins[j].y == sy) continue;
                net->twopin.emplace_back();
                auto& tp = net->twopin.back();
                tp.from = Point(pins[j].x, pins[j].y);
                tp.to = Point(sx, sy);
                tp.parNet = net;
                tp.wlen = std::abs(pins[j].x - sx) + std::abs(pins[j].y - sy);
            }
        } else {
            // Steiner-augmented decomposition was tried but consistently
            // worsened TOF (broke TOF=0 on adaptec4) for marginal WL win;
            // disabled here. Keeping function in case it's useful later.
            std::vector<Point> ePins(pins.begin(), pins.end());
            int en = (int)ePins.size();

            vector<bool> inTree(en, false);
            vector<int> minDist(en, INT_MAX);
            vector<int> parent(en, -1);
            inTree[0] = true;
            for (int j = 1; j < en; ++j) {
                int d = std::abs(ePins[0].x - ePins[j].x) +
                        std::abs(ePins[0].y - ePins[j].y);
                minDist[j] = d;
                parent[j] = 0;
            }
            for (int k = 1; k < en; ++k) {
                int u = -1, bestD = INT_MAX;
                for (int j = 0; j < en; ++j) {
                    if (!inTree[j] && minDist[j] < bestD) {
                        bestD = minDist[j];
                        u = j;
                    }
                }
                if (u == -1) break;
                inTree[u] = true;

                net->twopin.emplace_back();
                auto& tp = net->twopin.back();
                int p = parent[u];
                tp.from = Point(ePins[p].x, ePins[p].y);
                tp.to = Point(ePins[u].x, ePins[u].y);
                tp.parNet = net;
                tp.wlen = std::abs(ePins[p].x - ePins[u].x) +
                          std::abs(ePins[p].y - ePins[u].y);

                for (int j = 0; j < en; ++j) {
                    if (!inTree[j]) {
                        int d = std::abs(ePins[u].x - ePins[j].x) +
                                std::abs(ePins[u].y - ePins[j].y);
                        if (d < minDist[j]) {
                            minDist[j] = d;
                            parent[j] = u;
                        }
                    }
                }
            }
        }

        for (size_t si = 0; si < net->twopin.size(); ++si) {
            auto& tp = net->twopin[si];
            SubNet s;
            s.netIdx = (int)ni;
            s.subIdx = (int)si;
            s.x1 = tp.from.x;
            s.y1 = tp.from.y;
            s.x2 = tp.to.x;
            s.y2 = tp.to.y;
            s.hpwl = tp.HPWL();
            s.lastLen = s.hpwl;
            m_subs.push_back(s);
        }
        totalSubs += (int)net->twopin.size();
    }
    fprintf(stderr, "[Router2D] decomposed %zu nets -> %d subnets\n",
            m_d->nets.size(), totalSubs);
}

double Router2D::cellCost(int dem, int cap, float hist) const {
    // base reflects accumulated history of being overflow
    double base = 1.0 + (double)hist * m_kHist;
    if (cap <= 0) return base * 1000.0;
    int newDem = dem + m_demUnit;
    if (newDem <= cap) {
        // Mild util-based gradient to avoid funnelling, but mostly cheap
        double util = (double)newDem / (double)cap;
        return base * (1.0 + 0.5 * util);
    }
    // Hard penalty on overflow; grows linearly with how much over
    int over = newDem - cap;
    double overFrac = (double)over / (double)std::max(m_demUnit, 1);
    return base * (m_kCong + 2.0 * overFrac);
}

double Router2D::cellCostShared(int dem, int cap, float hist, int eid) const {
    // Edges already used by another subnet of the current net are essentially
    // free to re-use (verifier dedups via LayerAssignment BFS).
    if (m_curNetIdx >= 0 && m_netEdgeGen[eid] == m_netEdgeCurGen) {
        return 0.05;  // tiny non-zero to keep ordering sane
    }
    return cellCost(dem, cap, hist);
}

void Router2D::markNetEdges(int netIdx, int excludeSubIdx) {
    m_netEdgeCurGen++;
    m_curNetIdx = netIdx;
    auto* net = m_d->nets[netIdx];
    for (size_t si = 0; si < net->twopin.size(); ++si) {
        if ((int)si == excludeSubIdx) continue;
        for (const auto& p : net->twopin[si].path) {
            int eid = p.hori ? hEdgeId(p.x, p.y) : vEdgeId(p.x, p.y);
            m_netEdgeGen[eid] = m_netEdgeCurGen;
        }
    }
}

void Router2D::addDemandPath(const vector<RPoint>& path) {
    // Skips edges already marked as in-tree for the current net so the
    // global demand counter matches what the BFS-deduped output produces.
    // IMPORTANT: do NOT mark new edges here; doing so causes a tentative
    // add-then-revert (via removeDemandPath) to incorrectly skip them.
    // markNetEdges() is the only place that updates marks per processing.
    for (const auto& p : path) {
        int eid = p.hori ? hEdgeId(p.x, p.y) : vEdgeId(p.x, p.y);
        if (m_curNetIdx >= 0 && m_netEdgeGen[eid] == m_netEdgeCurGen) continue;
        if (p.hori) m_h[hIdx(p.x, p.y)].dem += m_demUnit;
        else        m_v[vIdx(p.x, p.y)].dem += m_demUnit;
    }
}

void Router2D::removeDemandPath(const vector<RPoint>& path) {
    // Removes demand only when no other subnet of this net keeps the edge alive.
    for (const auto& p : path) {
        int eid = p.hori ? hEdgeId(p.x, p.y) : vEdgeId(p.x, p.y);
        if (m_curNetIdx >= 0 && m_netEdgeGen[eid] == m_netEdgeCurGen) continue;
        if (p.hori) m_h[hIdx(p.x, p.y)].dem -= m_demUnit;
        else        m_v[vIdx(p.x, p.y)].dem -= m_demUnit;
    }
}

bool Router2D::pathHasOverflow(const vector<RPoint>& path) const {
    for (const auto& p : path) {
        if (p.hori) {
            const auto& e = m_h[hIdx(p.x, p.y)];
            if (e.dem > e.cap) return true;
        } else {
            const auto& e = m_v[vIdx(p.x, p.y)];
            if (e.dem > e.cap) return true;
        }
    }
    return false;
}

void Router2D::patternL(int x1, int y1, int x2, int y2,
                        vector<RPoint>& out) {
    out.clear();
    if (x1 == x2 && y1 == y2) return;

    auto pathCost = [&](bool horFirst) {
        double c = 0;
        if (horFirst) {
            int xs = std::min(x1, x2), xe = std::max(x1, x2);
            for (int xi = xs; xi < xe; ++xi) {
                const auto& e = m_h[hIdx(xi, y1)];
                c += cellCostShared(e.dem, e.cap, e.hist, hEdgeId(xi, y1));
            }
            int ys = std::min(y1, y2), ye = std::max(y1, y2);
            for (int yi = ys; yi < ye; ++yi) {
                const auto& e = m_v[vIdx(x2, yi)];
                c += cellCostShared(e.dem, e.cap, e.hist, vEdgeId(x2, yi));
            }
        } else {
            int ys = std::min(y1, y2), ye = std::max(y1, y2);
            for (int yi = ys; yi < ye; ++yi) {
                const auto& e = m_v[vIdx(x1, yi)];
                c += cellCostShared(e.dem, e.cap, e.hist, vEdgeId(x1, yi));
            }
            int xs = std::min(x1, x2), xe = std::max(x1, x2);
            for (int xi = xs; xi < xe; ++xi) {
                const auto& e = m_h[hIdx(xi, y2)];
                c += cellCostShared(e.dem, e.cap, e.hist, hEdgeId(xi, y2));
            }
        }
        return c;
    };

    bool horFirst = pathCost(true) <= pathCost(false);

    if (horFirst) {
        int xs = std::min(x1, x2), xe = std::max(x1, x2);
        for (int xi = xs; xi < xe; ++xi) out.emplace_back(xi, y1, true);
        int ys = std::min(y1, y2), ye = std::max(y1, y2);
        for (int yi = ys; yi < ye; ++yi) out.emplace_back(x2, yi, false);
    } else {
        int ys = std::min(y1, y2), ye = std::max(y1, y2);
        for (int yi = ys; yi < ye; ++yi) out.emplace_back(x1, yi, false);
        int xs = std::min(x1, x2), xe = std::max(x1, x2);
        for (int xi = xs; xi < xe; ++xi) out.emplace_back(xi, y2, true);
    }
}

bool Router2D::monotonicRoute(int x1, int y1, int x2, int y2,
                              vector<RPoint>& out) {
    out.clear();
    if (x1 == x2 && y1 == y2) return true;
    if (x1 == x2 || y1 == y2) {
        if (x1 == x2) {
            int ys = std::min(y1, y2), ye = std::max(y1, y2);
            for (int yi = ys; yi < ye; ++yi) out.emplace_back(x1, yi, false);
        } else {
            int xs = std::min(x1, x2), xe = std::max(x1, x2);
            for (int xi = xs; xi < xe; ++xi) out.emplace_back(xi, y1, true);
        }
        return true;
    }

    int dx = (x2 > x1) ? 1 : -1;
    int dy = (y2 > y1) ? 1 : -1;
    int W = std::abs(x2 - x1) + 1;
    int H = std::abs(y2 - y1) + 1;
    int N = W * H;

    // Reuse member scratch (sized to m_W*m_H, always >= N).
    const double INF = std::numeric_limits<double>::infinity();
    std::fill(m_dpBuf.begin(), m_dpBuf.begin() + N, INF);
    std::fill(m_parBuf.begin(), m_parBuf.begin() + N, (int8_t)-1);

    auto idx = [&](int i, int j) { return j * W + i; };
    m_dpBuf[idx(0, 0)] = 0.0;

    auto gx = [&](int i) { return x1 + i * dx; };
    auto gy = [&](int j) { return y1 + j * dy; };

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            if (i == 0 && j == 0) continue;
            double best = INF;
            int8_t b = -1;
            if (i > 0) {
                int curX = gx(i), prvX = gx(i - 1);
                int yy = gy(j);
                int el = std::min(curX, prvX);
                const auto& e = m_h[hIdx(el, yy)];
                double c = m_dpBuf[idx(i - 1, j)] +
                           cellCostShared(e.dem, e.cap, e.hist, hEdgeId(el, yy));
                if (c < best) { best = c; b = 0; }
            }
            if (j > 0) {
                int curY = gy(j), prvY = gy(j - 1);
                int xx = gx(i);
                int eb = std::min(curY, prvY);
                const auto& e = m_v[vIdx(xx, eb)];
                double c = m_dpBuf[idx(i, j - 1)] +
                           cellCostShared(e.dem, e.cap, e.hist, vEdgeId(xx, eb));
                if (c < best) { best = c; b = 1; }
            }
            m_dpBuf[idx(i, j)] = best;
            m_parBuf[idx(i, j)] = b;
        }
    }

    int i = W - 1, j = H - 1;
    while (i > 0 || j > 0) {
        int8_t p = m_parBuf[idx(i, j)];
        if (p == 0) {
            int curX = gx(i), prvX = gx(i - 1);
            int el = std::min(curX, prvX);
            out.emplace_back(el, gy(j), true);
            i--;
        } else if (p == 1) {
            int curY = gy(j), prvY = gy(j - 1);
            int eb = std::min(curY, prvY);
            out.emplace_back(gx(i), eb, false);
            j--;
        } else {
            return false;
        }
    }
    std::reverse(out.begin(), out.end());
    return true;
}

bool Router2D::mazeRoute(int sx, int sy, int tx, int ty, int lenBound,
                         vector<RPoint>& out,
                         int xExtend, int yExtend) {
    out.clear();
    if (sx == tx && sy == ty) return true;

    int hpwl = std::abs(sx - tx) + std::abs(sy - ty);
    if (lenBound < hpwl) return false;
    int slack = lenBound - hpwl;
    int isoMargin = std::min(slack / 2 + 5, 40);
    // Direction-aware: xExtend>0 grows search in x; yExtend grows in y.
    int xMargin = std::max(isoMargin, xExtend);
    int yMargin = std::max(isoMargin, yExtend);
    int xmin = std::max(0, std::min(sx, tx) - xMargin);
    int xmax = std::min(m_W - 1, std::max(sx, tx) + xMargin);
    int ymin = std::max(0, std::min(sy, ty) - yMargin);
    int ymax = std::min(m_H - 1, std::max(sy, ty) + yMargin);

    m_curGen++;
    auto cellIndex = [&](int x, int y) { return y * m_W + x; };

    auto h = [&](int x, int y) {
        return (double)(std::abs(x - tx) + std::abs(y - ty));
    };

    using Item = std::tuple<double, int, int, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

    int sIdx = cellIndex(sx, sy);
    m_visit[sIdx] = m_curGen;
    m_dist[sIdx] = 0.0;
    m_plen[sIdx] = 0;
    m_par[sIdx] = -1;
    pq.emplace(h(sx, sy), sx, sy, 0);

    bool found = false;
    while (!pq.empty()) {
        double f;
        int cx, cy, cl;
        std::tie(f, cx, cy, cl) = pq.top();
        pq.pop();

        if (cx == tx && cy == ty) { found = true; break; }
        int cI = cellIndex(cx, cy);
        double curG = m_dist[cI];
        if (cl != m_plen[cI]) continue;
        if (f > curG + h(cx, cy) + 1e-9) continue;

        // Right
        if (cx + 1 <= xmax) {
            const auto& e = m_h[hIdx(cx, cy)];
            double nc = curG + cellCostShared(e.dem, e.cap, e.hist, hEdgeId(cx, cy));
            int nl = cl + 1;
            int nI = cellIndex(cx + 1, cy);
            if (nl <= lenBound &&
                (m_visit[nI] != m_curGen || nc < m_dist[nI])) {
                m_visit[nI] = m_curGen;
                m_dist[nI] = nc;
                m_plen[nI] = nl;
                m_par[nI] = 0;
                pq.emplace(nc + h(cx + 1, cy), cx + 1, cy, nl);
            }
        }
        // Left
        if (cx - 1 >= xmin) {
            const auto& e = m_h[hIdx(cx - 1, cy)];
            double nc = curG + cellCostShared(e.dem, e.cap, e.hist, hEdgeId(cx - 1, cy));
            int nl = cl + 1;
            int nI = cellIndex(cx - 1, cy);
            if (nl <= lenBound &&
                (m_visit[nI] != m_curGen || nc < m_dist[nI])) {
                m_visit[nI] = m_curGen;
                m_dist[nI] = nc;
                m_plen[nI] = nl;
                m_par[nI] = 1;
                pq.emplace(nc + h(cx - 1, cy), cx - 1, cy, nl);
            }
        }
        // Up
        if (cy + 1 <= ymax) {
            const auto& e = m_v[vIdx(cx, cy)];
            double nc = curG + cellCostShared(e.dem, e.cap, e.hist, vEdgeId(cx, cy));
            int nl = cl + 1;
            int nI = cellIndex(cx, cy + 1);
            if (nl <= lenBound &&
                (m_visit[nI] != m_curGen || nc < m_dist[nI])) {
                m_visit[nI] = m_curGen;
                m_dist[nI] = nc;
                m_plen[nI] = nl;
                m_par[nI] = 2;
                pq.emplace(nc + h(cx, cy + 1), cx, cy + 1, nl);
            }
        }
        // Down
        if (cy - 1 >= ymin) {
            const auto& e = m_v[vIdx(cx, cy - 1)];
            double nc = curG + cellCostShared(e.dem, e.cap, e.hist, vEdgeId(cx, cy - 1));
            int nl = cl + 1;
            int nI = cellIndex(cx, cy - 1);
            if (nl <= lenBound &&
                (m_visit[nI] != m_curGen || nc < m_dist[nI])) {
                m_visit[nI] = m_curGen;
                m_dist[nI] = nc;
                m_plen[nI] = nl;
                m_par[nI] = 3;
                pq.emplace(nc + h(cx, cy - 1), cx, cy - 1, nl);
            }
        }
    }

    if (!found) return false;

    int cx = tx, cy = ty;
    while (!(cx == sx && cy == sy)) {
        int cI = cellIndex(cx, cy);
        int8_t d = m_par[cI];
        if (d == 0) {
            out.emplace_back(cx - 1, cy, true);
            cx--;
        } else if (d == 1) {
            out.emplace_back(cx, cy, true);
            cx++;
        } else if (d == 2) {
            out.emplace_back(cx, cy - 1, false);
            cy--;
        } else if (d == 3) {
            out.emplace_back(cx, cy, false);
            cy++;
        } else {
            return false;
        }
    }
    std::reverse(out.begin(), out.end());
    return true;
}

bool Router2D::humRoute(int sx, int sy, int tx, int ty,
                        int xExtend, int yExtend,
                        vector<RPoint>& out) {
    // Hybrid Unilateral Monotonic: single-pass DP that is monotonic in one
    // axis (toward T) and free in the other (so it can detour around OF).
    // We run both X-monotonic-with-Y-free and Y-monotonic-with-X-free, then
    // keep the cheaper. Per-column / per-row vertical/horizontal sweep
    // converges in 2 passes (1D shortest path on nonneg weights).
    out.clear();
    if (sx == tx && sy == ty) return true;

    int xMin = std::max(0, std::min(sx, tx) - xExtend);
    int xMax = std::min(m_W - 1, std::max(sx, tx) + xExtend);
    int yMin = std::max(0, std::min(sy, ty) - yExtend);
    int yMax = std::min(m_H - 1, std::max(sy, ty) + yExtend);
    int LW = xMax - xMin + 1;
    int LH = yMax - yMin + 1;
    if (LW < 1 || LH < 1) return false;

    auto lidx = [&](int lx, int ly) { return ly * LW + lx; };
    const double INF = std::numeric_limits<double>::infinity();
    int N = LW * LH;

    int slx = sx - xMin, sly = sy - yMin;
    int tlx = tx - xMin, tly = ty - yMin;

    // par codes: 0 from left (-x), 1 from right (+x), 2 from below (-y), 3 from above (+y)
    // Backtracks against the CURRENT m_parBuf state (so caller must back-
    // track BEFORE running the next sub-DP, otherwise par is overwritten).
    auto backtrack = [&](std::vector<RPoint>& path) -> bool {
        path.clear();
        int cx = tlx, cy = tly;
        int safety = LW * LH + 10;
        while ((cx != slx || cy != sly) && safety-- > 0) {
            int8_t p = m_parBuf[lidx(cx, cy)];
            int gx = cx + xMin, gy = cy + yMin;
            if      (p == 0) { path.emplace_back(gx - 1, gy, true);  cx--; }
            else if (p == 1) { path.emplace_back(gx,     gy, true);  cx++; }
            else if (p == 2) { path.emplace_back(gx, gy - 1, false); cy--; }
            else if (p == 3) { path.emplace_back(gx, gy,     false); cy++; }
            else return false;
        }
        if (cx != slx || cy != sly) return false;
        std::reverse(path.begin(), path.end());
        return true;
    };

    // X-monotonic-toward-T with Y-free: sweep columns in xdir, then within
    // each column relax up then down to allow vertical detours. Uses the
    // shared m_dpBuf/m_parBuf scratch (reset over the [0..N) prefix).
    auto runXMono = [&](double& outCost, std::vector<RPoint>& outPath) -> bool {
        std::fill(m_dpBuf.begin(), m_dpBuf.begin() + N, INF);
        std::fill(m_parBuf.begin(), m_parBuf.begin() + N, (int8_t)-1);
        m_dpBuf[lidx(slx, sly)] = 0.0;
        int xdir = (tx >= sx) ? +1 : -1;
        int8_t parFromPrevCol = (xdir == 1) ? 0 : 1;

        auto verticalRelax = [&](int lx) {
            int gx = lx + xMin;
            for (int ly = 1; ly < LH; ++ly) {
                int gy_below = ly - 1 + yMin;
                const auto& e = m_v[vIdx(gx, gy_below)];
                double c = m_dpBuf[lidx(lx, ly - 1)] +
                           cellCostShared(e.dem, e.cap, e.hist, vEdgeId(gx, gy_below));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c; m_parBuf[lidx(lx, ly)] = 2;
                }
            }
            for (int ly = LH - 2; ly >= 0; --ly) {
                int gy = ly + yMin;
                const auto& e = m_v[vIdx(gx, gy)];
                double c = m_dpBuf[lidx(lx, ly + 1)] +
                           cellCostShared(e.dem, e.cap, e.hist, vEdgeId(gx, gy));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c; m_parBuf[lidx(lx, ly)] = 3;
                }
            }
        };

        verticalRelax(slx);
        for (int lx = slx + xdir; lx >= 0 && lx < LW; lx += xdir) {
            int prev = lx - xdir;
            int gx_h = std::min(lx, prev) + xMin;
            for (int ly = 0; ly < LH; ++ly) {
                if (m_dpBuf[lidx(prev, ly)] >= INF) continue;
                int gy = ly + yMin;
                const auto& e = m_h[hIdx(gx_h, gy)];
                double c = m_dpBuf[lidx(prev, ly)] +
                           cellCostShared(e.dem, e.cap, e.hist, hEdgeId(gx_h, gy));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c;
                    m_parBuf[lidx(lx, ly)] = parFromPrevCol;
                }
            }
            verticalRelax(lx);
        }
        if (m_dpBuf[lidx(tlx, tly)] >= INF) return false;
        outCost = m_dpBuf[lidx(tlx, tly)];
        return backtrack(outPath);
    };

    auto runYMono = [&](double& outCost, std::vector<RPoint>& outPath) -> bool {
        std::fill(m_dpBuf.begin(), m_dpBuf.begin() + N, INF);
        std::fill(m_parBuf.begin(), m_parBuf.begin() + N, (int8_t)-1);
        m_dpBuf[lidx(slx, sly)] = 0.0;
        int ydir = (ty >= sy) ? +1 : -1;
        int8_t parFromPrevRow = (ydir == 1) ? 2 : 3;

        auto horizontalRelax = [&](int ly) {
            int gy = ly + yMin;
            for (int lx = 1; lx < LW; ++lx) {
                int gx_left = lx - 1 + xMin;
                const auto& e = m_h[hIdx(gx_left, gy)];
                double c = m_dpBuf[lidx(lx - 1, ly)] +
                           cellCostShared(e.dem, e.cap, e.hist, hEdgeId(gx_left, gy));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c; m_parBuf[lidx(lx, ly)] = 0;
                }
            }
            for (int lx = LW - 2; lx >= 0; --lx) {
                int gx = lx + xMin;
                const auto& e = m_h[hIdx(gx, gy)];
                double c = m_dpBuf[lidx(lx + 1, ly)] +
                           cellCostShared(e.dem, e.cap, e.hist, hEdgeId(gx, gy));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c; m_parBuf[lidx(lx, ly)] = 1;
                }
            }
        };

        horizontalRelax(sly);
        for (int ly = sly + ydir; ly >= 0 && ly < LH; ly += ydir) {
            int prev = ly - ydir;
            int gy_v = std::min(ly, prev) + yMin;
            for (int lx = 0; lx < LW; ++lx) {
                if (m_dpBuf[lidx(lx, prev)] >= INF) continue;
                int gx = lx + xMin;
                const auto& e = m_v[vIdx(gx, gy_v)];
                double c = m_dpBuf[lidx(lx, prev)] +
                           cellCostShared(e.dem, e.cap, e.hist, vEdgeId(gx, gy_v));
                if (c < m_dpBuf[lidx(lx, ly)]) {
                    m_dpBuf[lidx(lx, ly)] = c;
                    m_parBuf[lidx(lx, ly)] = parFromPrevRow;
                }
            }
            horizontalRelax(ly);
        }
        if (m_dpBuf[lidx(tlx, tly)] >= INF) return false;
        outCost = m_dpBuf[lidx(tlx, tly)];
        return backtrack(outPath);
    };

    double cX = INF, cY = INF;
    std::vector<RPoint> pX, pY;
    bool okX = runXMono(cX, pX);
    bool okY = runYMono(cY, pY);
    if (!okX && !okY) return false;
    if (okX && (!okY || cX <= cY)) out = std::move(pX);
    else                            out = std::move(pY);
    return true;
}

void Router2D::initialRoute() {
    // Stage-1 fast initial routing: cheap L-shape pattern (no DP allocation,
    // no PQ). Rip-up-and-reroute will fix the OF afterwards.
    for (size_t ni = 0; ni < m_d->nets.size(); ++ni) {
        auto* net = m_d->nets[ni];
        for (size_t si = 0; si < net->twopin.size(); ++si) {
            auto& tp = net->twopin[si];
            markNetEdges((int)ni, (int)si);
            patternL(tp.from.x, tp.from.y, tp.to.x, tp.to.y, tp.path);
            addDemandPath(tp.path);
        }
    }
    for (auto& s : m_subs) {
        auto* net = m_d->nets[s.netIdx];
        s.lastLen = (int)net->twopin[s.subIdx].path.size();
    }
}

int64_t Router2D::totalOverflow() const {
    int64_t tof = 0;
    for (const auto& e : m_h) if (e.dem > e.cap) tof += (e.dem - e.cap);
    for (const auto& e : m_v) if (e.dem > e.cap) tof += (e.dem - e.cap);
    return tof / m_demUnit;
}

int Router2D::maxOverflow() const {
    int mof = 0;
    for (const auto& e : m_h) if (e.dem - e.cap > mof) mof = e.dem - e.cap;
    for (const auto& e : m_v) if (e.dem - e.cap > mof) mof = e.dem - e.cap;
    return mof / m_demUnit;
}

int64_t Router2D::totalEdges() const {
    int64_t total = 0;
    for (auto* net : m_d->nets) {
        for (auto& tp : net->twopin) {
            total += (int64_t)tp.path.size();
        }
    }
    return total;
}

void Router2D::updateHistory() {
    // Mild multiplicative decay each iter: edges that were overflowing N
    // iters ago but are clean now should fade out, otherwise stale history
    // forces detours where they're no longer needed. Decay applies to ALL
    // edges, then OF edges get a fresh proportional boost on top.
    const float decay = 0.95f;
    const float histCap = 50.0f;  // hard cap to avoid late-iter blowup
    for (auto& e : m_h) {
        e.hist *= decay;
        if (e.dem > e.cap) {
            int over = e.dem - e.cap;
            float inc = (float)m_histInc * (1.0f + (float)over / (float)std::max(m_demUnit, 1) * 0.5f);
            e.hist += inc;
        }
        if (e.hist > histCap) e.hist = histCap;
    }
    for (auto& e : m_v) {
        e.hist *= decay;
        if (e.dem > e.cap) {
            int over = e.dem - e.cap;
            float inc = (float)m_histInc * (1.0f + (float)over / (float)std::max(m_demUnit, 1) * 0.5f);
            e.hist += inc;
        }
        if (e.hist > histCap) e.hist = histCap;
    }
}

void Router2D::nrrLoop() {
    // C10-style cascade (mono + HUM + maze ultra-fallback) plus a cheap
    // patternL pre-check that ONLY shortcircuits when patternL strictly
    // dominates the old path. Hard/congested subnets keep the full cascade.
    const int MAX_ITER = 200;
    const int64_t MAX_HUM_AREA = 60000;          // C10 baseline
    const int64_t MAX_MAZE_AREA = 40000;
    const int MAX_REROUTE_PER_ITER = 250000;
    double alpha = 1.2;
    int beta = 25;

    int64_t bestTOF = totalOverflow();
    int stuck = 0;

    // Snapshot: snap on strict improvement, but throttle "tiny" improvements
    // by requiring either >=0.5% TOF drop OR >=5 iters since last snap.
    // Bug-free: snapTOF tracks the snapshotted state, never overwritten by a
    // worse-than-best curTOF (since the entry guard is curTOF < snapTOF).
    std::vector<std::vector<RPoint>> snapPaths;
    std::vector<EdgeInfo> snapH, snapV;
    int64_t snapTOF = std::numeric_limits<int64_t>::max();
    int snapIter = -1;
    int lastSnapAt = -1000;

    auto pathOFCount = [&](const std::vector<RPoint>& p) {
        int cnt = 0;
        for (const auto& q : p) {
            if (q.hori) {
                const auto& e = m_h[hIdx(q.x, q.y)];
                if (e.dem + m_demUnit > e.cap) cnt++;
            } else {
                const auto& e = m_v[vIdx(q.x, q.y)];
                if (e.dem + m_demUnit > e.cap) cnt++;
            }
        }
        return cnt;
    };

    for (m_iter = 1; m_iter <= MAX_ITER; ++m_iter) {
        if (timeUp(60)) break;

        int rerouteCount = 0;
        std::vector<int> ofScore(m_subs.size(), 0);
        for (size_t i = 0; i < m_subs.size(); ++i) {
            auto& s = m_subs[i];
            auto* net = m_d->nets[s.netIdx];
            auto& tp = net->twopin[s.subIdx];
            int score = 0, ofH = 0, ofV = 0;
            for (const auto& p : tp.path) {
                if (p.hori) {
                    const auto& e = m_h[hIdx(p.x, p.y)];
                    if (e.dem > e.cap) { score += (e.dem - e.cap); ofH++; }
                } else {
                    const auto& e = m_v[vIdx(p.x, p.y)];
                    if (e.dem > e.cap) { score += (e.dem - e.cap); ofV++; }
                }
            }
            s.overflow = (score > 0);
            s.ripup = s.overflow;
            ofScore[i] = score;
            if (s.ripup) {
                rerouteCount++;
                s.stuckCount++;
                int growStep = 4 + std::min(s.stuckCount, 6);
                if (ofH >= ofV && s.yExtend < 60) s.yExtend += growStep;
                if (ofV >= ofH && s.xExtend < 60) s.xExtend += growStep;
            } else {
                s.stuckCount = 0;
                if (s.xExtend > 4) s.xExtend = std::max(4, s.xExtend - 1);
                if (s.yExtend > 4) s.yExtend = std::max(4, s.yExtend - 1);
            }
        }

        if (rerouteCount == 0) break;

        std::vector<int> order;
        order.reserve(rerouteCount);
        for (size_t i = 0; i < m_subs.size(); ++i) {
            if (m_subs[i].ripup) order.push_back((int)i);
        }
        const int C7 = 256;
        const int C8 = 1;
        auto rankScore = [&](int i) {
            return (int64_t)C7 * ofScore[i] + (int64_t)C8 * m_subs[i].lastLen;
        };
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return rankScore(a) > rankScore(b);
        });

        if ((int)order.size() > MAX_REROUTE_PER_ITER)
            order.resize(MAX_REROUTE_PER_ITER);

        int nPat = 0, nMono = 0, nHum = 0, nMaze = 0;

        for (int si : order) {
            auto& s = m_subs[si];
            auto* net = m_d->nets[s.netIdx];
            auto& tp = net->twopin[s.subIdx];

            markNetEdges(s.netIdx, s.subIdx);
            removeDemandPath(tp.path);

            // OF count of the old path on the post-removal demand state.
            // Used as the comparator for the patternL dominance pre-check.
            int oldOF = pathOFCount(tp.path);
            int oldLen = (int)tp.path.size();

            // Cheap patternL pre-check. Accept ONLY if it strictly dominates
            // (no OF, OR no worse OF AND strictly shorter). Otherwise fall
            // through to the C10 cascade so quality is preserved on
            // congested subnets where pattern can't escape OF.
            std::vector<RPoint> patPath;
            patternL(s.x1, s.y1, s.x2, s.y2, patPath);
            int patOF = pathOFCount(patPath);
            bool patternDominates =
                (patOF == 0) ||
                (patOF <= oldOF && (int)patPath.size() < oldLen);

            std::vector<RPoint> bestPath;
            int bestOF;
            if (patternDominates) {
                bestPath = std::move(patPath);
                bestOF = patOF;
                nPat++;
            } else {
                // Step 1: monotonic baseline.
                monotonicRoute(s.x1, s.y1, s.x2, s.y2, bestPath);
                bestOF = pathOFCount(bestPath);
                nMono++;

                // Keep the patternL candidate as a fallback if it happens to
                // beat monotonic (shorter at equal OF, or fewer OF).
                if (patOF < bestOF ||
                    (patOF == bestOF && patPath.size() < bestPath.size())) {
                    bestPath = std::move(patPath);
                    bestOF = patOF;
                }

                // Step 2: HUM with adaptive direction-aware bbox.
                if (bestOF > 0) {
                    int64_t humArea =
                        (int64_t)(std::min(m_W - 1, std::max(s.x1, s.x2) + s.xExtend) -
                                  std::max(0, std::min(s.x1, s.x2) - s.xExtend) + 1) *
                        (int64_t)(std::min(m_H - 1, std::max(s.y1, s.y2) + s.yExtend) -
                                  std::max(0, std::min(s.y1, s.y2) - s.yExtend) + 1);
                    if (humArea <= MAX_HUM_AREA) {
                        std::vector<RPoint> humPath;
                        if (humRoute(s.x1, s.y1, s.x2, s.y2,
                                     s.xExtend, s.yExtend, humPath)) {
                            int hof = pathOFCount(humPath);
                            if (hof < bestOF ||
                                (hof == bestOF && humPath.size() < bestPath.size())) {
                                bestPath = std::move(humPath);
                                bestOF = hof;
                            }
                        }
                        nHum++;
                    }
                }

                // Step 3: maze ultra-fallback for stubborn subnets.
                if (bestOF > 0 && s.stuckCount >= 4) {
                    int lenBound = std::max(s.hpwl + beta,
                                            (int)(alpha * s.lastLen) + beta);
                    int slack = lenBound - s.hpwl;
                    int isoMargin = std::min(slack / 2 + 5, 40);
                    int xMargin = std::max(isoMargin, s.xExtend);
                    int yMargin = std::max(isoMargin, s.yExtend);
                    int bbxLo = std::max(0, std::min(s.x1, s.x2) - xMargin);
                    int bbxHi = std::min(m_W - 1, std::max(s.x1, s.x2) + xMargin);
                    int bbyLo = std::max(0, std::min(s.y1, s.y2) - yMargin);
                    int bbyHi = std::min(m_H - 1, std::max(s.y1, s.y2) + yMargin);
                    int64_t bbArea = (int64_t)(bbxHi - bbxLo + 1) *
                                     (int64_t)(bbyHi - bbyLo + 1);
                    if (bbArea <= MAX_MAZE_AREA) {
                        std::vector<RPoint> mazePath;
                        if (mazeRoute(s.x1, s.y1, s.x2, s.y2, lenBound, mazePath,
                                      s.xExtend, s.yExtend)) {
                            int mof = pathOFCount(mazePath);
                            if (mof < bestOF ||
                                (mof == bestOF && mazePath.size() < bestPath.size())) {
                                bestPath = std::move(mazePath);
                                bestOF = mof;
                            }
                        }
                        nMaze++;
                    }
                }
            }

            tp.path = std::move(bestPath);
            addDemandPath(tp.path);
            s.lastLen = (int)tp.path.size();
        }

        updateHistory();

        int64_t curTOF = totalOverflow();
        int curMOF = maxOverflow();

        // Throttled snap: strict improvement only, then require either a
        // meaningful (>=0.5%) drop or >=5 iters since last snap. First snap
        // is always taken.
        bool snapNow = false;
        if (curTOF < snapTOF) {
            bool firstSnap = (snapTOF == std::numeric_limits<int64_t>::max());
            bool meaningful = !firstSnap && (snapTOF - curTOF) * 200 >= snapTOF;
            bool farEnough = !firstSnap && (m_iter - lastSnapAt >= 5);
            if (firstSnap || meaningful || farEnough) {
                snapTOF = curTOF;
                snapIter = m_iter;
                lastSnapAt = m_iter;
                snapH = m_h;
                snapV = m_v;
                snapPaths.resize(m_subs.size());
                for (size_t i = 0; i < m_subs.size(); ++i) {
                    auto& s = m_subs[i];
                    snapPaths[i] = m_d->nets[s.netIdx]->twopin[s.subIdx].path;
                }
                snapNow = true;
            }
        }

        if (curTOF == 0) {
            fprintf(stderr,
                    "[Router2D] iter=%d TOF=%lld MOF=%d rerouted=%d nP=%d nM=%d nH=%d nZ=%d time=%.1fs stuck=%d\n",
                    m_iter, (long long)curTOF, curMOF, rerouteCount,
                    nPat, nMono, nHum, nMaze, elapsedSec(), stuck);
            break;
        }

        if (m_iter >= 5 && m_kCong < 24.0) m_kCong += 0.5;

        if (curTOF >= bestTOF) {
            stuck++;
            if (m_histInc < 5.0) m_histInc *= 1.25;
        } else {
            stuck = 0;
            bestTOF = curTOF;
        }

        fprintf(stderr,
                "[Router2D] iter=%d TOF=%lld MOF=%d rerouted=%d nP=%d nM=%d nH=%d nZ=%d time=%.1fs hInc=%.2f kCong=%.2f stuck=%d%s\n",
                m_iter, (long long)curTOF, curMOF, rerouteCount,
                nPat, nMono, nHum, nMaze, elapsedSec(),
                m_histInc, m_kCong, stuck,
                snapNow ? " SNAP" : "");

        if (stuck >= 40) break;
    }

    // Restore best-seen NRR snapshot if the loop ended above its low-water
    // mark (common when oscillating around a plateau).
    int64_t curTOF = totalOverflow();
    if (snapIter > 0 && snapTOF < curTOF) {
        m_h = std::move(snapH);
        m_v = std::move(snapV);
        for (size_t i = 0; i < m_subs.size(); ++i) {
            auto& s = m_subs[i];
            m_d->nets[s.netIdx]->twopin[s.subIdx].path = std::move(snapPaths[i]);
            s.lastLen = (int)m_d->nets[s.netIdx]->twopin[s.subIdx].path.size();
        }
        fprintf(stderr,
                "[Router2D] restored snapshot iter=%d TOF=%lld (was TOF=%lld)\n",
                snapIter, (long long)snapTOF, (long long)curTOF);
    }
}

void Router2D::refinement() {
    // Run refinement to shorten paths even if some OF remains; only accept
    // if the path doesn't add new OF AND total OF doesn't increase.
    int64_t startTOF = totalOverflow();

    int improved = 0;
    for (auto& s : m_subs) {
        if (timeUp(60)) break;
        auto* net = m_d->nets[s.netIdx];
        auto& tp = net->twopin[s.subIdx];
        if ((int)tp.path.size() <= s.hpwl) continue;

        markNetEdges(s.netIdx, s.subIdx);
        removeDemandPath(tp.path);
        std::vector<RPoint> newPath;
        bool ok = monotonicRoute(s.x1, s.y1, s.x2, s.y2, newPath);
        if (!ok) {
            addDemandPath(tp.path);
            continue;
        }
        addDemandPath(newPath);
        bool nfok = !pathHasOverflow(newPath);
        if (nfok && newPath.size() < tp.path.size()) {
            tp.path = std::move(newPath);
            s.lastLen = (int)tp.path.size();
            improved++;
        } else {
            removeDemandPath(newPath);
            addDemandPath(tp.path);
        }
    }
    int64_t endTOF = totalOverflow();
    fprintf(stderr, "[Router2D] refinement improved %d subnets (TOF %lld -> %lld)\n",
            improved, (long long)startTOF, (long long)endTOF);
}

void Router2D::run() {
    fprintf(stderr,
            "[Router2D] grid=%dx%d nets=%zu time_budget=%ds demUnit=%d\n",
            m_W, m_H, m_d->nets.size(), m_timeLimit, m_demUnit);
    buildCapacity();
    fprintf(stderr, "[Router2D] capacity built [%.2fs]\n", elapsedSec());
    decomposeNets();
    fprintf(stderr, "[Router2D] decomposition done [%.2fs]\n", elapsedSec());
    initialRoute();
    fprintf(stderr,
            "[Router2D] initial routing done [%.2fs] TOF=%lld MOF=%d edges=%lld\n",
            elapsedSec(), (long long)totalOverflow(), maxOverflow(),
            (long long)totalEdges());
    nrrLoop();
    fprintf(stderr,
            "[Router2D] NRR done [%.2fs] TOF=%lld MOF=%d edges=%lld\n",
            elapsedSec(), (long long)totalOverflow(), maxOverflow(),
            (long long)totalEdges());
    refinement();
    fprintf(stderr,
            "[Router2D] refinement done [%.2fs] TOF=%lld MOF=%d edges=%lld\n",
            elapsedSec(), (long long)totalOverflow(), maxOverflow(),
            (long long)totalEdges());
}

} // namespace Router
