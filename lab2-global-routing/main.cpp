#include "LayerAssignment.h"
#include "ispdData.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

struct GridPoint {
    int x;
    int y;

    GridPoint() : x(0), y(0) {}
    GridPoint(int ax, int ay) : x(ax), y(ay) {}
};

struct EdgeUse {
    int x;
    int y;
    bool hori;
};

struct RouteNet {
    ISPDParser::Net *src;
    std::vector<GridPoint> pins;
    std::unordered_set<long long> usedEdges;
    std::unordered_map<long long, int> usedEdgeCount;
    std::unordered_set<long long> usedNodes;
    std::unordered_map<long long, int> usedNodeCount;
    std::unordered_set<long long> preferredEdges;
    int bboxArea;
    int hpwl;
    int previousWireLength;
};

struct RouteTask {
    int netIndex;
    GridPoint source;
    GridPoint target;
    std::vector<EdgeUse> edges;
    ISPDParser::TwoPin two;
    int hpwl;
    int previousWireLength;
    int lastOverflow;
};

struct HeapNode {
    double key;
    int idx;

    bool operator<(const HeapNode &rhs) const {
        return key > rhs.key;
    }
};

struct NetScore {
    int routeIndex;
    int overflowAmount;
    int overflowEdges;
    int wireLength;
};

class GlobalRouter2D {
public:
    explicit GlobalRouter2D(ISPDParser::ispdData &data)
        : db(data),
          xNum(data.numXGrid),
          yNum(data.numYGrid),
          defaultTrackDemand(1) {
        initCapacities();
    }

    void buildRouteList() {
        routes.clear();
        tasks.clear();

        for (ISPDParser::Net *net : db.nets) {
            net->pin2D.clear();
            net->pin3D.clear();
            net->twopin.clear();

            if (net->numPins > 1000)
                continue;

            std::vector<GridPoint> pins2D;
            std::unordered_set<long long> seen2D;
            std::unordered_set<long long> seen3D;
            int minX = std::numeric_limits<int>::max();
            int minY = std::numeric_limits<int>::max();
            int maxX = std::numeric_limits<int>::min();
            int maxY = std::numeric_limits<int>::min();

            for (size_t i = 0; i < net->pins.size(); ++i) {
                const int gx = (std::get<0>(net->pins[i]) - db.lowerLeftX) / db.tileWidth;
                const int gy = (std::get<1>(net->pins[i]) - db.lowerLeftY) / db.tileHeight;
                const int gz = std::get<2>(net->pins[i]) - 1;

                const long long key2D = pointKey(gx, gy);
                if (seen2D.insert(key2D).second) {
                    pins2D.push_back(GridPoint(gx, gy));
                    net->pin2D.push_back(ISPDParser::Point(gx, gy, 0));
                }

                const long long key3D = pointKey(gx, gy) * 16LL + gz;
                if (seen3D.insert(key3D).second)
                    net->pin3D.push_back(ISPDParser::Point(gx, gy, gz));

                minX = std::min(minX, gx);
                minY = std::min(minY, gy);
                maxX = std::max(maxX, gx);
                maxY = std::max(maxY, gy);
            }

            if (pins2D.size() <= 1)
                continue;
            if (minX == maxX && minY == maxY)
                continue;

            RouteNet rn;
            rn.src = net;
            rn.pins = pins2D;
            rn.hpwl = (maxX - minX) + (maxY - minY);
            rn.bboxArea = std::max(1, (maxX - minX + 1) * (maxY - minY + 1));
            rn.previousWireLength = std::max(1, rn.hpwl);
            buildPreferredSkeleton(rn);
            routes.push_back(rn);
        }

        // The monotonic stage benefits from routing the least flexible nets first.
        std::sort(routes.begin(), routes.end(), [](const RouteNet &a, const RouteNet &b) {
            if (a.bboxArea != b.bboxArea)
                return a.bboxArea < b.bboxArea;
            if (a.hpwl != b.hpwl)
                return a.hpwl < b.hpwl;
            return a.pins.size() > b.pins.size();
        });

        buildRouteTasks();
    }

    void routeAll() {
        const bool verbose = std::getenv("ROUTER_DEBUG") != NULL;
        for (size_t i = 0; i < tasks.size(); ++i)
            routeTask(tasks[i], false, 0);

        OverflowSummary cur = computeOverflow();
        if (verbose)
            std::cerr << "2D initial overflow=" << cur.totalOverflow << " max=" << cur.maxOverflow << "\n";

        if (useSmallCaseFastFlow())
            patternOverflowRefine(cur, 1, verbose);

        int bestOverflow = cur.totalOverflow;
        int staleRounds = 0;
        const int maxIterations = chooseIterationLimit();

        for (int iter = 1; iter <= maxIterations; ++iter) {
            if (cur.totalOverflow == 0)
                break;

            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            const size_t taskLimit = chooseNrrTaskLimit(iter, rerouteList.size());
            if (rerouteList.size() > taskLimit)
                rerouteList.resize(taskLimit);
            const size_t mazeBudget = chooseNrrMazeBudget(iter, rerouteList.size());
            resetOverflowFrequency();

            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                ripUpTask(task);
                routeTask(task, i < mazeBudget, iter);
                recordOverflowFrequency(task);
            }
            applyOverflowFrequencyToHistory();

            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D iter=" << iter << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouteList.size() << "\n";
            if (cur.totalOverflow < bestOverflow) {
                bestOverflow = cur.totalOverflow;
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 8 && iter >= 12)
                break;
            if (useSmallCaseFastFlow() && iter >= 7 && cur.totalOverflow <= 2500)
                break;
        }

        polishMediumOverflow(cur, maxIterations + 1, verbose);
        polishSmallOverflow(cur, maxIterations + 1, verbose);
        greedyOverflowRefine(cur, maxIterations + 64, verbose);
        coherentNetRefine(cur, maxIterations + 96, verbose);
        hotspotEscapeRefine(cur, maxIterations + 112, verbose);
        regionScheduleRefine(cur, maxIterations + 128, verbose);
        polishTinyOverflow(cur, maxIterations + 96, verbose);
        postRefine();
        multipinTreeRefine(cur, maxIterations + 160, verbose);
        maxOverflowRefine(cur, maxIterations + 224, verbose);
        repairDisconnectedNets();
        finalizeTwopins();
    }

    void assignLayersAndWrite(const char *filename) {
        LayerAssignment::Graph graph;
        graph.initialLA(db, 1);
        const bool verbose = std::getenv("ROUTER_DEBUG") != NULL;
        graph.convertGRtoLA(db, verbose);
        graph.COLA(verbose);
        graph.output3Dresult(filename);
    }

private:
    struct OverflowSummary {
        int totalOverflow;
        int maxOverflow;
    };

    struct RouteSnapshot {
        std::vector<std::vector<EdgeUse>> edges;
        std::vector<ISPDParser::TwoPin> twopins;
        std::vector<int> previousWireLengths;
    };

    struct RegionBucket {
        int key;
        int score;
        std::vector<int> tasks;
    };

    ISPDParser::ispdData &db;
    int xNum;
    int yNum;
    int defaultTrackDemand;
    std::vector<int> hCap;
    std::vector<int> vCap;
    std::vector<int> hDemand;
    std::vector<int> vDemand;
    std::vector<int> hHistory;
    std::vector<int> vHistory;
    std::vector<int> hOverflowFreq;
    std::vector<int> vOverflowFreq;
    std::vector<int> hMark;
    std::vector<int> vMark;
    std::vector<RouteNet> routes;
    std::vector<RouteTask> tasks;
    mutable std::unordered_map<std::string, std::vector<GridPoint>> fluteLookupCache;

    long long pointKey(int x, int y) const {
        return static_cast<long long>(y) * xNum + x;
    }

    int hIndex(int x, int y) const {
        return y * (xNum - 1) + x;
    }

    int vIndex(int x, int y) const {
        return y * xNum + x;
    }

    long long edgeKey(const EdgeUse &edge) const {
        return (static_cast<long long>(edge.hori ? yNum : 0) + edge.y) * xNum + edge.x;
    }

    EdgeUse edgeFromKey(long long key) const {
        const long long row = key / xNum;
        EdgeUse edge;
        edge.x = static_cast<int>(key - row * xNum);
        edge.hori = row >= yNum;
        edge.y = static_cast<int>(edge.hori ? row - yNum : row);
        return edge;
    }

    int gridToX(int x) const { return x * db.tileWidth + db.lowerLeftX; }
    int gridToY(int y) const { return y * db.tileHeight + db.lowerLeftY; }

    int edgeCapacity(const EdgeUse &edge) const {
        return edge.hori ? hCap[hIndex(edge.x, edge.y)] : vCap[vIndex(edge.x, edge.y)];
    }

    int edgeDemand(const EdgeUse &edge) const {
        return edge.hori ? hDemand[hIndex(edge.x, edge.y)] : vDemand[vIndex(edge.x, edge.y)];
    }

    int edgeHistory(const EdgeUse &edge) const {
        return edge.hori ? hHistory[hIndex(edge.x, edge.y)] : vHistory[vIndex(edge.x, edge.y)];
    }

    int edgeMark(const EdgeUse &edge) const {
        return edge.hori ? hMark[hIndex(edge.x, edge.y)] : vMark[vIndex(edge.x, edge.y)];
    }

    int chooseIterationLimit() const {
        const int taskCount = static_cast<int>(tasks.size());
        if (taskCount > 450000)
            return 28;
        if (taskCount > 260000)
            return 34;
        return 42;
    }

    bool useSmallCaseFastFlow() const {
        return db.nets.size() < 260000;
    }

    size_t chooseNrrTaskLimit(int iter, size_t rerouteCount) const {
        if (useSmallCaseFastFlow())
            return std::min(rerouteCount, iter <= 4 ? static_cast<size_t>(45000) : static_cast<size_t>(30000));

        return rerouteCount;
    }

    size_t chooseNrrMazeBudget(int iter, size_t rerouteCount) const {
        (void)iter;
        return rerouteCount;
    }

    int searchMarginCap() const {
        return useSmallCaseFastFlow() ? 45 : 95;
    }

    int patternExtensionLimit(int iter) const {
        if (iter <= 0)
            return 0;
        if (useSmallCaseFastFlow())
            return std::min(24, 4 + iter * 2);
        return std::min(12, 2 + iter);
    }

    void initCapacities() {
        hCap.assign(std::max(0, xNum - 1) * yNum, 0);
        vCap.assign(xNum * std::max(0, yNum - 1), 0);
        hDemand.assign(hCap.size(), 0);
        vDemand.assign(vCap.size(), 0);
        hHistory.assign(hCap.size(), 0);
        vHistory.assign(vCap.size(), 0);
        hOverflowFreq.assign(hCap.size(), 0);
        vOverflowFreq.assign(vCap.size(), 0);
        hMark.assign(hCap.size(), 0);
        vMark.assign(vCap.size(), 0);

        for (int z = 0; z < db.numLayer; ++z) {
            const int wireSize = std::max(1, db.minimumWidth[z] + db.minimumSpacing[z]);
            const int hTracks = db.horizontalCapacity[z] / wireSize;
            const int vTracks = db.verticalCapacity[z] / wireSize;

            for (int y = 0; y < yNum; ++y)
                for (int x = 0; x < xNum - 1; ++x)
                    hCap[hIndex(x, y)] += hTracks;

            for (int y = 0; y < yNum - 1; ++y)
                for (int x = 0; x < xNum; ++x)
                    vCap[vIndex(x, y)] += vTracks;
        }

        for (size_t i = 0; i < db.capacityAdjs.size(); ++i) {
            const ISPDParser::CapacityAdj &adj = *db.capacityAdjs[i];
            const int x1 = std::get<0>(adj.grid1);
            const int y1 = std::get<1>(adj.grid1);
            const int z1 = std::get<2>(adj.grid1) - 1;
            const int x2 = std::get<0>(adj.grid2);
            const int y2 = std::get<1>(adj.grid2);
            const int wireSize = std::max(1, db.minimumWidth[z1] + db.minimumSpacing[z1]);

            if (x1 != x2) {
                const int idx = hIndex(std::min(x1, x2), y1);
                const int oldTracks = db.horizontalCapacity[z1] / wireSize;
                const int newTracks = adj.reducedCapacityLevel / wireSize;
                hCap[idx] = std::max(0, hCap[idx] - oldTracks + newTracks);
            } else if (y1 != y2) {
                const int idx = vIndex(x1, std::min(y1, y2));
                const int oldTracks = db.verticalCapacity[z1] / wireSize;
                const int newTracks = adj.reducedCapacityLevel / wireSize;
                vCap[idx] = std::max(0, vCap[idx] - oldTracks + newTracks);
            }
        }
    }

    EdgeUse makeEdge(const GridPoint &a, const GridPoint &b) const {
        EdgeUse edge;
        if (a.x != b.x) {
            edge.x = std::min(a.x, b.x);
            edge.y = a.y;
            edge.hori = true;
        } else {
            edge.x = a.x;
            edge.y = std::min(a.y, b.y);
            edge.hori = false;
        }
        return edge;
    }

    double edgeCost(const EdgeUse &edge, int iter, bool maze, const RouteNet *net) const {
        const int cap = edgeCapacity(edge);
        const int dem = edgeDemand(edge);
        const int hist = edgeHistory(edge);
        const int after = dem + defaultTrackDemand;
        const int marks = maze ? edgeMark(edge) : 0;
        const double virtualDemand = (marks > 0) ? 0.4 * std::sqrt(static_cast<double>(marks)) : 0.0;
        const double afterForCost = static_cast<double>(after) + virtualDemand;

        if (cap <= 0) {
            return 100000.0 + 500.0 * (afterForCost + static_cast<double>(hist));
        }

        const double overflow = std::max(0.0, afterForCost - static_cast<double>(cap));
        const double util = afterForCost / static_cast<double>(cap);
        const double delta = static_cast<double>(cap) - afterForCost;
        const double logisticArg = std::max(-50.0, std::min(50.0, delta / 0.3));
        const double pe = 1.0 + 150.0 / (1.0 + std::exp(logisticArg));
        const double be = maze ? (30.0 + 200.0 / std::pow(2.0, std::min(iter, 20))) : 1.0;
        const double dah = (hist > 0)
            ? static_cast<double>(hist) / (7.0 + 4.0 * std::sqrt(static_cast<double>(std::max(1, iter))))
            : 0.0;

        double cost = be + (1.0 + dah) * pe;
        if (overflow > 0.0) {
            cost += (maze ? 40.0 : 15.0) * overflow * overflow;
        } else if (util > 0.80) {
            cost += (util - 0.80) * 15.0;
        } else if (marks > 0) {
            cost += 2.0 * virtualDemand;
        }

        if (net != NULL) {
            const long long key = edgeKey(edge);
            if (net->usedEdges.find(key) != net->usedEdges.end()) {
                cost *= 0.25;
            } else if (overflow <= 0.0 &&
                       net->preferredEdges.find(key) != net->preferredEdges.end()) {
                cost *= maze ? 0.94 : 0.90;
            }
        }

        return cost;
    }

    int mstLength(const std::vector<GridPoint> &points,
                  std::vector<std::pair<int, int>> *mstEdges) const {
        const int n = static_cast<int>(points.size());
        if (n <= 1)
            return 0;

        std::vector<int> minDist(n, std::numeric_limits<int>::max());
        std::vector<int> parent(n, -1);
        std::vector<char> used(n, 0);
        minDist[0] = 0;

        int total = 0;
        for (int step = 0; step < n; ++step) {
            int u = -1;
            for (int i = 0; i < n; ++i)
                if (!used[i] && (u < 0 || minDist[i] < minDist[u]))
                    u = i;

            if (u < 0)
                break;

            used[u] = 1;
            total += minDist[u];
            if (mstEdges != NULL && parent[u] >= 0)
                mstEdges->push_back(std::make_pair(parent[u], u));

            for (int v = 0; v < n; ++v) {
                if (used[v])
                    continue;
                const int d = manhattan(points[u], points[v]);
                if (d < minDist[v]) {
                    minDist[v] = d;
                    parent[v] = u;
                }
            }
        }

        return total;
    }

    int coordinateRank(const std::vector<int> &values, int value) const {
        return static_cast<int>(std::lower_bound(values.begin(), values.end(), value) - values.begin());
    }

    std::string fluteLookupKey(const std::vector<GridPoint> &pins,
                               const std::vector<int> &xs,
                               const std::vector<int> &ys) const {
        std::vector<GridPoint> rankedPins;
        rankedPins.reserve(pins.size());
        for (size_t i = 0; i < pins.size(); ++i) {
            rankedPins.push_back(GridPoint(coordinateRank(xs, pins[i].x),
                                           coordinateRank(ys, pins[i].y)));
        }
        std::sort(rankedPins.begin(), rankedPins.end(), [](const GridPoint &a, const GridPoint &b) {
            if (a.x != b.x)
                return a.x < b.x;
            return a.y < b.y;
        });

        std::string key = std::to_string(pins.size());
        key += "|x";
        for (size_t i = 1; i < xs.size(); ++i) {
            key += ",";
            key += std::to_string(xs[i] - xs[i - 1]);
        }
        key += "|y";
        for (size_t i = 1; i < ys.size(); ++i) {
            key += ",";
            key += std::to_string(ys[i] - ys[i - 1]);
        }
        key += "|p";
        for (size_t i = 0; i < rankedPins.size(); ++i) {
            key += ";";
            key += std::to_string(rankedPins[i].x);
            key += ",";
            key += std::to_string(rankedPins[i].y);
        }
        return key;
    }

    std::vector<GridPoint> solveSmallFluteLookup(const std::vector<GridPoint> &pins,
                                                 const std::vector<int> &xs,
                                                 const std::vector<int> &ys) const {
        std::unordered_set<long long> terminalSet;
        for (size_t i = 0; i < pins.size(); ++i)
            terminalSet.insert(pointKey(pins[i].x, pins[i].y));

        std::vector<GridPoint> candidates;
        candidates.reserve(xs.size() * ys.size());
        for (size_t xi = 0; xi < xs.size(); ++xi) {
            for (size_t yi = 0; yi < ys.size(); ++yi) {
                const GridPoint cand(xs[xi], ys[yi]);
                if (terminalSet.find(pointKey(cand.x, cand.y)) == terminalSet.end())
                    candidates.push_back(cand);
            }
        }

        const int baseLen = mstLength(pins, NULL);
        int bestLen = baseLen;
        std::vector<GridPoint> best;
        std::vector<GridPoint> test = pins;

        for (size_t a = 0; a < candidates.size(); ++a) {
            test.push_back(candidates[a]);
            int len = mstLength(test, NULL);
            if (len < bestLen) {
                bestLen = len;
                best.assign(1, candidates[a]);
            }

            for (size_t b = a + 1; b < candidates.size(); ++b) {
                test.push_back(candidates[b]);
                len = mstLength(test, NULL);
                if (len < bestLen) {
                    bestLen = len;
                    best.clear();
                    best.push_back(candidates[a]);
                    best.push_back(candidates[b]);
                }

                for (size_t c = b + 1; c < candidates.size(); ++c) {
                    test.push_back(candidates[c]);
                    len = mstLength(test, NULL);
                    if (len < bestLen) {
                        bestLen = len;
                        best.clear();
                        best.push_back(candidates[a]);
                        best.push_back(candidates[b]);
                        best.push_back(candidates[c]);
                    }
                    test.pop_back();
                }
                test.pop_back();
            }
            test.pop_back();
        }

        std::vector<GridPoint> ranked;
        ranked.reserve(best.size());
        for (size_t i = 0; i < best.size(); ++i) {
            ranked.push_back(GridPoint(coordinateRank(xs, best[i].x),
                                       coordinateRank(ys, best[i].y)));
        }
        return ranked;
    }

    std::vector<GridPoint> buildFluteLookupPointSet(const std::vector<GridPoint> &pins,
                                                    const std::vector<int> &xs,
                                                    const std::vector<int> &ys) const {
        std::vector<GridPoint> points = pins;
        if (pins.size() <= 2 || pins.size() > 6 || xs.empty() || ys.empty())
            return points;

        const size_t hananSites = xs.size() * ys.size();
        const size_t candidateBudget = hananSites > pins.size() ? hananSites - pins.size() : 0;
        if (candidateBudget > 10 || (pins.size() > 4 && candidateBudget > 8))
            return points;

        const std::string key = fluteLookupKey(pins, xs, ys);
        std::unordered_map<std::string, std::vector<GridPoint>>::const_iterator it =
            fluteLookupCache.find(key);
        if (it == fluteLookupCache.end()) {
            const std::vector<GridPoint> ranked = solveSmallFluteLookup(pins, xs, ys);
            it = fluteLookupCache.insert(std::make_pair(key, ranked)).first;
        }

        std::unordered_set<long long> existing;
        for (size_t i = 0; i < points.size(); ++i)
            existing.insert(pointKey(points[i].x, points[i].y));

        for (size_t i = 0; i < it->second.size(); ++i) {
            const int xr = it->second[i].x;
            const int yr = it->second[i].y;
            if (xr < 0 || xr >= static_cast<int>(xs.size()) ||
                yr < 0 || yr >= static_cast<int>(ys.size()))
                continue;

            const GridPoint p(xs[xr], ys[yr]);
            if (existing.insert(pointKey(p.x, p.y)).second)
                points.push_back(p);
        }
        return points;
    }

    void appendPreferredL(RouteNet &net, const GridPoint &source, const GridPoint &target, bool horizontalFirst) const {
        std::vector<GridPoint> path = buildLPath(source, target, horizontalFirst);
        for (size_t i = 1; i < path.size(); ++i)
            net.preferredEdges.insert(edgeKey(makeEdge(path[i - 1], path[i])));
    }

    std::vector<GridPoint> buildSteinerPointSet(const std::vector<GridPoint> &pins) const {
        std::vector<GridPoint> points = pins;
        if (points.size() > 2 && points.size() <= 128) {
            std::unordered_set<long long> existing;
            for (size_t i = 0; i < points.size(); ++i)
                existing.insert(pointKey(points[i].x, points[i].y));

            std::vector<int> xs;
            std::vector<int> ys;
            xs.reserve(points.size());
            ys.reserve(points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                xs.push_back(points[i].x);
                ys.push_back(points[i].y);
            }
            std::sort(xs.begin(), xs.end());
            std::sort(ys.begin(), ys.end());
            xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
            ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

            if (points.size() <= 6) {
                std::vector<GridPoint> lookupPoints = buildFluteLookupPointSet(points, xs, ys);
                if (lookupPoints.size() > points.size())
                    return lookupPoints;
            }

            std::vector<GridPoint> candidates;
            const size_t hananCount = xs.size() * ys.size();
            if (points.size() <= 14 || hananCount <= 400) {
                for (size_t xi = 0; xi < xs.size(); ++xi)
                    for (size_t yi = 0; yi < ys.size(); ++yi)
                        candidates.push_back(GridPoint(xs[xi], ys[yi]));
            } else {
                std::vector<std::pair<int, GridPoint>> scored;
                scored.reserve(std::min<size_t>(hananCount, 4096));
                const int medianX = xs[xs.size() / 2];
                const int medianY = ys[ys.size() / 2];

                for (size_t xi = 0; xi < xs.size(); ++xi) {
                    for (size_t yi = 0; yi < ys.size(); ++yi) {
                        const GridPoint cand(xs[xi], ys[yi]);
                        const long long key = pointKey(cand.x, cand.y);
                        if (existing.find(key) != existing.end())
                            continue;

                        int best1 = std::numeric_limits<int>::max();
                        int best2 = std::numeric_limits<int>::max();
                        int best3 = std::numeric_limits<int>::max();
                        for (size_t p = 0; p < pins.size(); ++p) {
                            const int d = manhattan(cand, pins[p]);
                            if (d < best1) {
                                best3 = best2;
                                best2 = best1;
                                best1 = d;
                            } else if (d < best2) {
                                best3 = best2;
                                best2 = d;
                            } else if (d < best3) {
                                best3 = d;
                            }
                        }

                        const int centerPenalty =
                            (std::abs(cand.x - medianX) + std::abs(cand.y - medianY)) / 4;
                        scored.push_back(std::make_pair(best1 + best2 + best3 + centerPenalty, cand));
                    }
                }

                std::sort(scored.begin(), scored.end(),
                          [](const std::pair<int, GridPoint> &a,
                             const std::pair<int, GridPoint> &b) {
                              if (a.first != b.first)
                                  return a.first < b.first;
                              if (a.second.x != b.second.x)
                                  return a.second.x < b.second.x;
                              return a.second.y < b.second.y;
                          });

                const size_t keep = (points.size() <= 32) ? 160 : 80;
                for (size_t i = 0; i < scored.size() && i < keep; ++i)
                    candidates.push_back(scored[i].second);
            }

            const int maxSteinerPoints = (points.size() <= 9) ? 3 : ((points.size() <= 32) ? 3 : 2);
            for (int iter = 0; iter < maxSteinerPoints; ++iter) {
                const int baseLen = mstLength(points, NULL);
                int bestLen = baseLen;
                GridPoint bestPoint;
                bool found = false;

                for (size_t ci = 0; ci < candidates.size(); ++ci) {
                    const GridPoint cand = candidates[ci];
                    const long long key = pointKey(cand.x, cand.y);
                    if (existing.find(key) != existing.end())
                        continue;

                    points.push_back(cand);
                    const int len = mstLength(points, NULL);
                    points.pop_back();

                    if (len + 1 < bestLen) {
                        bestLen = len;
                        bestPoint = cand;
                        found = true;
                    }
                }

                if (!found)
                    break;

                points.push_back(bestPoint);
                existing.insert(pointKey(bestPoint.x, bestPoint.y));
            }
        }

        return points;
    }

    void buildPreferredSkeleton(RouteNet &net) const {
        if (net.pins.size() <= 2)
            return;

        const std::vector<GridPoint> points = buildSteinerPointSet(net.pins);
        std::vector<std::pair<int, int>> mst;
        mstLength(points, &mst);
        for (size_t i = 0; i < mst.size(); ++i) {
            const GridPoint &a = points[mst[i].first];
            const GridPoint &b = points[mst[i].second];
            appendPreferredL(net, a, b, true);
            appendPreferredL(net, a, b, false);
        }
    }

    void buildRouteTasks() {
        tasks.clear();

        for (size_t n = 0; n < routes.size(); ++n) {
            RouteNet &net = routes[n];
            std::vector<std::pair<int, int>> mst;
            mstLength(net.pins, &mst);

            for (size_t i = 0; i < mst.size(); ++i) {
                const GridPoint source = net.pins[mst[i].first];
                const GridPoint target = net.pins[mst[i].second];
                if (source.x == target.x && source.y == target.y)
                    continue;

                RouteTask task;
                task.netIndex = static_cast<int>(n);
                task.source = source;
                task.target = target;
                task.hpwl = manhattan(source, target);
                task.previousWireLength = std::max(1, task.hpwl);
                task.lastOverflow = 0;
                task.two.from = ISPDParser::Point(source.x, source.y, 0);
                task.two.to = ISPDParser::Point(target.x, target.y, 0);
                task.two.parNet = net.src;
                task.two.wlen = 0;
                tasks.push_back(task);
            }
        }

        std::sort(tasks.begin(), tasks.end(), [](const RouteTask &a, const RouteTask &b) {
            const int adx = std::abs(a.source.x - a.target.x);
            const int ady = std::abs(a.source.y - a.target.y);
            const int bdx = std::abs(b.source.x - b.target.x);
            const int bdy = std::abs(b.source.y - b.target.y);
            const long long aArea = static_cast<long long>(adx + 1) * static_cast<long long>(ady + 1);
            const long long bArea = static_cast<long long>(bdx + 1) * static_cast<long long>(bdy + 1);
            if (aArea != bArea)
                return aArea < bArea;
            if (a.hpwl != b.hpwl)
                return a.hpwl < b.hpwl;
            return a.netIndex < b.netIndex;
        });
    }

    void addDemand(const EdgeUse &edge) {
        if (edge.hori)
            hDemand[hIndex(edge.x, edge.y)] += defaultTrackDemand;
        else
            vDemand[vIndex(edge.x, edge.y)] += defaultTrackDemand;
    }

    void removeDemand(const EdgeUse &edge) {
        if (edge.hori)
            hDemand[hIndex(edge.x, edge.y)] -= defaultTrackDemand;
        else
            vDemand[vIndex(edge.x, edge.y)] -= defaultTrackDemand;
    }

    void addMark(const EdgeUse &edge) {
        if (edge.hori)
            ++hMark[hIndex(edge.x, edge.y)];
        else
            ++vMark[vIndex(edge.x, edge.y)];
    }

    void removeMark(const EdgeUse &edge) {
        if (edge.hori) {
            int &mark = hMark[hIndex(edge.x, edge.y)];
            if (mark > 0)
                --mark;
        } else {
            int &mark = vMark[vIndex(edge.x, edge.y)];
            if (mark > 0)
                --mark;
        }
    }

    void adjustMark(const EdgeUse &edge, int delta) {
        if (edge.hori) {
            int &mark = hMark[hIndex(edge.x, edge.y)];
            mark = std::max(0, mark + delta);
        } else {
            int &mark = vMark[vIndex(edge.x, edge.y)];
            mark = std::max(0, mark + delta);
        }
    }

    void markEdges(const std::vector<EdgeUse> &edges, int delta) {
        for (size_t i = 0; i < edges.size(); ++i) {
            if (delta > 0)
                addMark(edges[i]);
            else if (delta < 0)
                removeMark(edges[i]);
        }
    }

    void markOverflowEdges(const RouteTask &task, int amount) {
        std::unordered_set<long long> seen;
        for (size_t i = 0; i < task.edges.size(); ++i) {
            const EdgeUse &edge = task.edges[i];
            if (!seen.insert(edgeKey(edge)).second)
                continue;
            if (edgeDemand(edge) > edgeCapacity(edge))
                adjustMark(edge, amount);
        }
    }

    void markPeakOverflowEdges(const RouteTask &task, int threshold, int amount) {
        std::unordered_set<long long> seen;
        for (size_t i = 0; i < task.edges.size(); ++i) {
            const EdgeUse &edge = task.edges[i];
            if (!seen.insert(edgeKey(edge)).second)
                continue;
            if (edgeDemand(edge) - edgeCapacity(edge) >= threshold)
                adjustMark(edge, amount);
        }
    }

    void clearMarks() {
        std::fill(hMark.begin(), hMark.end(), 0);
        std::fill(vMark.begin(), vMark.end(), 0);
    }

    void addNetEdgeUse(RouteNet &net, const EdgeUse &edge) {
        const long long key = edgeKey(edge);
        int &count = net.usedEdgeCount[key];
        if (count == 0)
            addDemand(edge);
        ++count;
        net.usedEdges.insert(key);

        GridPoint a(edge.x, edge.y);
        GridPoint b = edge.hori ? GridPoint(edge.x + 1, edge.y) : GridPoint(edge.x, edge.y + 1);
        const long long ak = pointKey(a.x, a.y);
        const long long bk = pointKey(b.x, b.y);
        ++net.usedNodeCount[ak];
        ++net.usedNodeCount[bk];
        net.usedNodes.insert(ak);
        net.usedNodes.insert(bk);
    }

    void removeNetEdgeUse(RouteNet &net, const EdgeUse &edge) {
        const long long key = edgeKey(edge);
        std::unordered_map<long long, int>::iterator it = net.usedEdgeCount.find(key);
        if (it == net.usedEdgeCount.end())
            return;

        --it->second;
        if (it->second <= 0) {
            net.usedEdgeCount.erase(it);
            net.usedEdges.erase(key);
            removeDemand(edge);
        }

        GridPoint a(edge.x, edge.y);
        GridPoint b = edge.hori ? GridPoint(edge.x + 1, edge.y) : GridPoint(edge.x, edge.y + 1);
        decrementNodeUse(net, pointKey(a.x, a.y));
        decrementNodeUse(net, pointKey(b.x, b.y));
    }

    void decrementNodeUse(RouteNet &net, long long key) {
        std::unordered_map<long long, int>::iterator it = net.usedNodeCount.find(key);
        if (it == net.usedNodeCount.end())
            return;

        --it->second;
        if (it->second <= 0) {
            net.usedNodeCount.erase(it);
            net.usedNodes.erase(key);
        }
    }

    void resetOverflowFrequency() {
        std::fill(hOverflowFreq.begin(), hOverflowFreq.end(), 0);
        std::fill(vOverflowFreq.begin(), vOverflowFreq.end(), 0);
    }

    void recordOverflowFrequency(const RouteTask &task) {
        std::unordered_set<long long> seen;
        for (size_t i = 0; i < task.edges.size(); ++i) {
            const EdgeUse &edge = task.edges[i];
            const long long key = edgeKey(edge);
            if (!seen.insert(key).second)
                continue;

            if (edgeDemand(edge) <= edgeCapacity(edge))
                continue;

            if (edge.hori)
                ++hOverflowFreq[hIndex(edge.x, edge.y)];
            else
                ++vOverflowFreq[vIndex(edge.x, edge.y)];
        }
    }

    void applyOverflowFrequencyToHistory() {
        for (size_t i = 0; i < hHistory.size(); ++i)
            hHistory[i] += hOverflowFreq[i];
        for (size_t i = 0; i < vHistory.size(); ++i)
            vHistory[i] += vOverflowFreq[i];
    }

    OverflowSummary computeOverflow() const {
        OverflowSummary out;
        out.totalOverflow = 0;
        out.maxOverflow = 0;

        for (size_t i = 0; i < hDemand.size(); ++i) {
            const int overflow = hDemand[i] - hCap[i];
            if (overflow > 0) {
                out.totalOverflow += overflow;
                out.maxOverflow = std::max(out.maxOverflow, overflow);
            }
        }

        for (size_t i = 0; i < vDemand.size(); ++i) {
            const int overflow = vDemand[i] - vCap[i];
            if (overflow > 0) {
                out.totalOverflow += overflow;
                out.maxOverflow = std::max(out.maxOverflow, overflow);
            }
        }

        return out;
    }

    std::vector<NetScore> collectOverflowedNets() {
        std::vector<NetScore> result;
        result.reserve(tasks.size() / 8 + 1);

        for (size_t i = 0; i < tasks.size(); ++i) {
            RouteTask &task = tasks[i];
            int amount = 0;
            int edges = 0;

            for (size_t j = 0; j < task.edges.size(); ++j) {
                const EdgeUse &edge = task.edges[j];
                const int overflow = edgeDemand(edge) - edgeCapacity(edge);
                if (overflow > 0) {
                    amount += overflow;
                    ++edges;
                }
            }

            task.lastOverflow = amount;
            if (amount > 0) {
                NetScore score;
                score.routeIndex = static_cast<int>(i);
                score.overflowAmount = amount;
                score.overflowEdges = edges;
                score.wireLength = static_cast<int>(task.edges.size());
                result.push_back(score);
            }
        }

        return result;
    }

    void sortRerouteList(std::vector<NetScore> &rerouteList) const {
        std::sort(rerouteList.begin(), rerouteList.end(), [](const NetScore &a, const NetScore &b) {
            const long long sa = static_cast<long long>(a.overflowEdges) * 30LL + a.wireLength;
            const long long sb = static_cast<long long>(b.overflowEdges) * 30LL + b.wireLength;
            if (sa != sb)
                return sa > sb;
            if (a.overflowAmount != b.overflowAmount)
                return a.overflowAmount > b.overflowAmount;
            return a.routeIndex < b.routeIndex;
        });
    }

    void routeTaskPattern(RouteTask &task, int iter) {
        RouteNet &net = routes[task.netIndex];
        const int extension = patternExtensionLimit(iter);
        const std::vector<GridPoint> path =
            patternPath(net, task.source, task.target, iter, true, extension);
        commitTaskPath(task, path);
    }

    void patternOverflowRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0)
            return;

        RouteSnapshot bestSnapshot = makeSnapshot();
        int bestOverflow = cur.totalOverflow;
        int bestMaxOverflow = cur.maxOverflow;
        int staleRounds = 0;

        for (int round = 0; round < 16 && cur.totalOverflow > 0; ++round) {
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 70000)
                rerouteList.resize(70000);

            resetOverflowFrequency();
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                ripUpTask(task);
                routeTaskPattern(task, firstIter + round);
                recordOverflowFrequency(task);
            }
            applyOverflowFrequencyToHistory();

            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D pattern=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouteList.size() << "\n";

            if (cur.totalOverflow < bestOverflow ||
                (cur.totalOverflow == bestOverflow && cur.maxOverflow < bestMaxOverflow)) {
                bestOverflow = cur.totalOverflow;
                bestMaxOverflow = cur.maxOverflow;
                bestSnapshot = makeSnapshot();
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 5)
                break;
        }

        if (cur.totalOverflow > bestOverflow ||
            (cur.totalOverflow == bestOverflow && cur.maxOverflow > bestMaxOverflow)) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    void polishSmallOverflow(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 1024)
            return;

        int bestOverflow = cur.totalOverflow;
        RouteSnapshot bestSnapshot = makeSnapshot();
        int staleRounds = 0;
        for (int round = 0; round < 128 && cur.totalOverflow > 0; ++round) {
            const int iter = firstIter + round;
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 8000)
                rerouteList.resize(8000);

            resetOverflowFrequency();
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                ripUpTask(task);
                routeTask(task, true, iter);
                recordOverflowFrequency(task);
            }
            applyOverflowFrequencyToHistory();

            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D polish=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouteList.size() << "\n";

            if (cur.totalOverflow < bestOverflow) {
                bestOverflow = cur.totalOverflow;
                bestSnapshot = makeSnapshot();
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 28)
                break;
        }

        if (cur.totalOverflow > bestOverflow) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    void polishMediumOverflow(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow <= 256 || cur.totalOverflow > 5000)
            return;

        int bestOverflow = cur.totalOverflow;
        RouteSnapshot bestSnapshot = makeSnapshot();
        int staleRounds = 0;
        for (int round = 0; round < 40 && cur.totalOverflow > 512; ++round) {
            const int iter = firstIter + round;
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 12000)
                rerouteList.resize(12000);

            resetOverflowFrequency();
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                ripUpTask(task);
                routeTask(task, true, iter);
                recordOverflowFrequency(task);
            }
            applyOverflowFrequencyToHistory();

            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D medium=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouteList.size() << "\n";

            if (cur.totalOverflow < bestOverflow) {
                bestOverflow = cur.totalOverflow;
                bestSnapshot = makeSnapshot();
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 12)
                break;
        }

        if (cur.totalOverflow > bestOverflow) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    void polishTinyOverflow(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 128)
            return;

        int bestOverflow = cur.totalOverflow;
        RouteSnapshot bestSnapshot = makeSnapshot();
        int staleRounds = 0;
        for (int round = 0; round < 80 && cur.totalOverflow > 0; ++round) {
            const int iter = firstIter + round * 2;
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 12000)
                rerouteList.resize(12000);

            resetOverflowFrequency();
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                ripUpTask(task);
                routeTask(task, true, iter);
                recordOverflowFrequency(task);
            }
            applyOverflowFrequencyToHistory();

            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D tiny=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouteList.size() << "\n";

            if (cur.totalOverflow < bestOverflow) {
                bestOverflow = cur.totalOverflow;
                bestSnapshot = makeSnapshot();
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 20)
                break;
        }

        if (cur.totalOverflow > bestOverflow) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    void greedyOverflowRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 2048)
            return;

        for (int round = 0; round < 10 && cur.totalOverflow > 0; ++round) {
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 8000)
                rerouteList.resize(8000);

            resetOverflowFrequency();
            int accepted = 0;
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                const std::vector<EdgeUse> oldEdges = task.edges;
                const ISPDParser::TwoPin oldTwoPin = task.two;
                const size_t oldLength = oldEdges.size();
                const OverflowSummary before = cur;

                ripUpTask(task);
                routeTask(task, true, firstIter + round * 3);
                OverflowSummary after = computeOverflow();

                const bool improvesOverflow = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && task.edges.size() < oldLength;

                if (improvesOverflow || improvesLength) {
                    cur = after;
                    recordOverflowFrequency(task);
                    ++accepted;
                    if (cur.totalOverflow == 0)
                        break;
                } else {
                    ripUpTask(task);
                    task.edges = oldEdges;
                    task.two = oldTwoPin;
                    for (size_t j = 0; j < task.edges.size(); ++j) {
                        addNetEdgeUse(routes[task.netIndex], task.edges[j]);
                    }
                    cur = before;
                }
            }
            applyOverflowFrequencyToHistory();

            if (verbose)
                std::cerr << "2D greedy=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " accepted=" << accepted << "\n";

            if (accepted == 0)
                break;
        }
    }

    void coherentNetRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 4096)
            return;

        std::vector<std::vector<int>> tasksByNet(routes.size());
        for (size_t i = 0; i < tasks.size(); ++i)
            tasksByNet[tasks[i].netIndex].push_back(static_cast<int>(i));

        int staleRounds = 0;
        for (int round = 0; round < 8 && cur.totalOverflow > 0; ++round) {
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            std::vector<char> selected(routes.size(), 0);
            std::vector<int> netOrder;
            netOrder.reserve(std::min<size_t>(rerouteList.size(), 2048));
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                const int netIndex = tasks[rerouteList[i].routeIndex].netIndex;
                if (selected[netIndex])
                    continue;
                selected[netIndex] = 1;
                netOrder.push_back(netIndex);
                if (netOrder.size() >= 2048)
                    break;
            }

            resetOverflowFrequency();
            int accepted = 0;
            const OverflowSummary roundStart = cur;

            for (size_t ni = 0; ni < netOrder.size(); ++ni) {
                const std::vector<int> &ids = tasksByNet[netOrder[ni]];
                if (ids.empty())
                    continue;

                OverflowSummary before = cur;
                size_t oldLength = 0;
                std::vector<std::vector<EdgeUse>> oldEdges(ids.size());
                std::vector<ISPDParser::TwoPin> oldTwopins(ids.size());
                std::vector<int> oldPrevious(ids.size());

                for (size_t i = 0; i < ids.size(); ++i) {
                    RouteTask &task = tasks[ids[i]];
                    oldEdges[i] = task.edges;
                    oldTwopins[i] = task.two;
                    oldPrevious[i] = task.previousWireLength;
                    oldLength += task.edges.size();
                }

                for (size_t i = 0; i < ids.size(); ++i)
                    ripUpTask(tasks[ids[i]]);

                for (size_t i = 0; i < ids.size(); ++i)
                    routeTask(tasks[ids[i]], true, firstIter + round);

                OverflowSummary after = computeOverflow();
                size_t newLength = 0;
                for (size_t i = 0; i < ids.size(); ++i)
                    newLength += tasks[ids[i]].edges.size();

                const bool improvesOverflow = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && newLength < oldLength;

                if (improvesOverflow || improvesLength) {
                    cur = after;
                    ++accepted;
                    for (size_t i = 0; i < ids.size(); ++i)
                        recordOverflowFrequency(tasks[ids[i]]);
                    if (cur.totalOverflow == 0)
                        break;
                } else {
                    for (size_t i = 0; i < ids.size(); ++i)
                        ripUpTask(tasks[ids[i]]);

                    for (size_t i = 0; i < ids.size(); ++i) {
                        RouteTask &task = tasks[ids[i]];
                        task.edges = oldEdges[i];
                        task.two = oldTwopins[i];
                        task.previousWireLength = oldPrevious[i];
                        for (size_t j = 0; j < task.edges.size(); ++j)
                            addNetEdgeUse(routes[task.netIndex], task.edges[j]);
                    }
                    cur = before;
                }
            }

            applyOverflowFrequencyToHistory();
            if (verbose)
                std::cerr << "2D coherent=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " accepted=" << accepted << "\n";

            if (cur.totalOverflow < roundStart.totalOverflow)
                staleRounds = 0;
            else
                ++staleRounds;

            if (accepted == 0 || staleRounds >= 2)
                break;
        }
    }

    void hotspotEscapeRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 512)
            return;

        for (int round = 0; round < 10 && cur.totalOverflow > 0; ++round) {
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 6000)
                rerouteList.resize(6000);

            resetOverflowFrequency();
            int accepted = 0;
            const int roundStartOverflow = cur.totalOverflow;

            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                const std::vector<EdgeUse> oldEdges = task.edges;
                const ISPDParser::TwoPin oldTwoPin = task.two;
                const int oldPrevious = task.previousWireLength;
                const size_t oldLength = task.edges.size();
                const OverflowSummary before = cur;

                clearMarks();
                markOverflowEdges(task, 64);
                ripUpTask(task);
                routeTask(task, true, firstIter + round);
                clearMarks();

                const OverflowSummary after = computeOverflow();
                const bool improvesOverflow = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && task.edges.size() < oldLength;

                if (improvesOverflow || improvesLength) {
                    cur = after;
                    recordOverflowFrequency(task);
                    ++accepted;
                    if (cur.totalOverflow == 0)
                        break;
                } else {
                    ripUpTask(task);
                    task.edges = oldEdges;
                    task.two = oldTwoPin;
                    task.previousWireLength = oldPrevious;
                    for (size_t e = 0; e < task.edges.size(); ++e)
                        addNetEdgeUse(routes[task.netIndex], task.edges[e]);
                    cur = before;
                }
            }

            applyOverflowFrequencyToHistory();
            if (verbose)
                std::cerr << "2D hotspot=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " accepted=" << accepted << "\n";

            if (accepted == 0 || cur.totalOverflow >= roundStartOverflow)
                break;
        }
    }

    int overflowRegionKey(const EdgeUse &edge) const {
        const int bucketSize = regionBucketSize();
        const int bx = edge.x / bucketSize;
        const int by = edge.y / bucketSize;
        return by * 1024 + bx;
    }

    int regionBucketSize() const {
        return 20;
    }

    std::vector<RegionBucket> collectOverflowRegions() const {
        std::unordered_map<int, RegionBucket> buckets;

        for (size_t i = 0; i < tasks.size(); ++i) {
            const RouteTask &task = tasks[i];
            std::unordered_set<int> seenRegions;
            for (size_t j = 0; j < task.edges.size(); ++j) {
                const EdgeUse &edge = task.edges[j];
                const int overflow = edgeDemand(edge) - edgeCapacity(edge);
                if (overflow <= 0)
                    continue;

                const int key = overflowRegionKey(edge);
                if (!seenRegions.insert(key).second)
                    continue;

                RegionBucket &bucket = buckets[key];
                bucket.key = key;
                bucket.score += overflow;
                bucket.tasks.push_back(static_cast<int>(i));
            }
        }

        std::vector<RegionBucket> out;
        out.reserve(buckets.size());
        for (std::unordered_map<int, RegionBucket>::iterator it = buckets.begin();
             it != buckets.end(); ++it)
            out.push_back(it->second);

        std::sort(out.begin(), out.end(), [](const RegionBucket &a, const RegionBucket &b) {
            if (a.score != b.score)
                return a.score > b.score;
            return a.key < b.key;
        });
        return out;
    }

    bool edgeInRegionNeighborhood(const EdgeUse &edge, int key) const {
        const int bucketSize = regionBucketSize();
        const int bx = key % 1024;
        const int by = key / 1024;
        const int xLo = std::max(0, bx * bucketSize - bucketSize);
        const int yLo = std::max(0, by * bucketSize - bucketSize);
        const int xHi = std::min(xNum - 1, (bx + 2) * bucketSize - 1);
        const int yHi = std::min(yNum - 1, (by + 2) * bucketSize - 1);
        return edge.x >= xLo && edge.x <= xHi && edge.y >= yLo && edge.y <= yHi;
    }

    std::vector<int> regionReliefTaskOrder(const RegionBucket &bucket) const {
        std::unordered_set<int> seen;
        std::vector<std::pair<int, int>> scored;

        for (size_t i = 0; i < bucket.tasks.size(); ++i) {
            const int id = bucket.tasks[i];
            if (seen.insert(id).second) {
                const RouteTask &task = tasks[id];
                scored.push_back(std::make_pair(1000000 + task.lastOverflow * 100 +
                                                static_cast<int>(task.edges.size()), id));
            }
        }

        for (size_t i = 0; i < tasks.size(); ++i) {
            const int id = static_cast<int>(i);
            if (seen.find(id) != seen.end())
                continue;

            const RouteTask &task = tasks[i];
            int score = 0;
            for (size_t e = 0; e < task.edges.size(); ++e) {
                const EdgeUse &edge = task.edges[e];
                if (!edgeInRegionNeighborhood(edge, bucket.key))
                    continue;

                const int cap = edgeCapacity(edge);
                const int dem = edgeDemand(edge);
                if (cap <= 0) {
                    score += 32;
                } else if (dem >= cap) {
                    score += 16 + (dem - cap) * 8;
                } else if (dem * 100 >= cap * 85) {
                    score += 4;
                }
            }

            if (score > 0 && seen.insert(id).second)
                scored.push_back(std::make_pair(score, id));
        }

        std::sort(scored.begin(), scored.end(),
                  [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
                      if (a.first != b.first)
                          return a.first > b.first;
                      return a.second < b.second;
                  });

        std::vector<int> out;
        out.reserve(std::min<size_t>(scored.size(), 1024));
        std::unordered_set<int> lockedNets;
        for (size_t i = 0; i < scored.size() && out.size() < 1024; ++i) {
            const int id = scored[i].second;
            if (!lockedNets.insert(tasks[id].netIndex).second)
                continue;
            out.push_back(id);
        }
        return out;
    }

    size_t batchWireLength(const std::vector<int> &batch) const {
        size_t total = 0;
        for (size_t i = 0; i < batch.size(); ++i)
            total += tasks[batch[i]].edges.size();
        return total;
    }

    void restoreTaskBatch(const std::vector<int> &batch,
                          const std::vector<std::vector<EdgeUse>> &oldEdges,
                          const std::vector<ISPDParser::TwoPin> &oldTwopins,
                          const std::vector<int> &oldPrevious) {
        clearMarks();
        for (size_t i = 0; i < batch.size(); ++i)
            ripUpTask(tasks[batch[i]]);

        for (size_t i = 0; i < batch.size(); ++i) {
            RouteTask &task = tasks[batch[i]];
            task.edges = oldEdges[i];
            task.two = oldTwopins[i];
            task.previousWireLength = oldPrevious[i];
            for (size_t e = 0; e < task.edges.size(); ++e)
                addNetEdgeUse(routes[task.netIndex], task.edges[e]);
        }
    }

    void routeCollisionAwareBatch(const std::vector<int> &batch,
                                  const std::vector<std::vector<EdgeUse>> &oldEdges,
                                  int iter,
                                  int &rerouted) {
        if (batch.empty())
            return;

        clearMarks();
        for (size_t i = 0; i < batch.size(); ++i)
            markEdges(oldEdges[i], 1);

        for (size_t i = 0; i < batch.size(); ++i)
            ripUpTask(tasks[batch[i]]);

        for (size_t i = 0; i < batch.size(); ++i) {
            RouteTask &task = tasks[batch[i]];
            markEdges(oldEdges[i], -1);
            routeTask(task, true, iter);
            ++rerouted;
        }

        clearMarks();
    }

    void regionScheduleRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 2048)
            return;

        int bestOverflow = cur.totalOverflow;
        int bestMaxOverflow = cur.maxOverflow;
        RouteSnapshot bestSnapshot = makeSnapshot();
        int staleRounds = 0;

        for (int round = 0; round < 8 && cur.totalOverflow > 0; ++round) {
            std::vector<RegionBucket> buckets = collectOverflowRegions();
            if (buckets.empty())
                break;

            const size_t bucketLimit = std::min<size_t>(buckets.size(), 16);
            resetOverflowFrequency();
            int rerouted = 0;

            for (size_t b = 0; b < bucketLimit; ++b) {
                const std::vector<int> batch = regionReliefTaskOrder(buckets[b]);
                if (batch.empty())
                    continue;

                std::vector<std::vector<EdgeUse>> oldEdges(batch.size());
                std::vector<ISPDParser::TwoPin> oldTwopins(batch.size());
                std::vector<int> oldPrevious(batch.size());
                for (size_t i = 0; i < batch.size(); ++i) {
                    const RouteTask &task = tasks[batch[i]];
                    oldEdges[i] = task.edges;
                    oldTwopins[i] = task.two;
                    oldPrevious[i] = task.previousWireLength;
                }

                const OverflowSummary before = cur;
                const size_t oldLength = batchWireLength(batch);
                routeCollisionAwareBatch(batch, oldEdges, firstIter + round, rerouted);
                const OverflowSummary after = computeOverflow();
                const size_t newLength = batchWireLength(batch);

                const bool improvesOverflow = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && newLength < oldLength;

                if (improvesOverflow || improvesLength) {
                    cur = after;
                    for (size_t i = 0; i < batch.size(); ++i)
                        recordOverflowFrequency(tasks[batch[i]]);
                    if (cur.totalOverflow == 0)
                        break;
                } else {
                    restoreTaskBatch(batch, oldEdges, oldTwopins, oldPrevious);
                    cur = before;
                }
            }

            applyOverflowFrequencyToHistory();
            cur = computeOverflow();
            if (verbose)
                std::cerr << "2D region=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " rerouted=" << rerouted << "\n";

            if (cur.totalOverflow < bestOverflow ||
                (cur.totalOverflow == bestOverflow && cur.maxOverflow < bestMaxOverflow)) {
                bestOverflow = cur.totalOverflow;
                bestMaxOverflow = cur.maxOverflow;
                bestSnapshot = makeSnapshot();
                staleRounds = 0;
            } else {
                ++staleRounds;
            }

            if (staleRounds >= 3)
                break;
        }

        if (cur.totalOverflow > bestOverflow ||
            (cur.totalOverflow == bestOverflow && cur.maxOverflow > bestMaxOverflow)) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    bool buildMultipinTreeCandidate(int netIndex,
                                    int iter,
                                    std::vector<RouteTask> &candidateTasks) {
        RouteNet &net = routes[netIndex];
        if (net.pins.size() <= 1)
            return false;

        std::vector<GridPoint> treeNodes;
        std::unordered_set<long long> treeSet;
        std::vector<char> connected(net.pins.size(), 0);

        const int root = chooseRootPin(net.pins);
        treeNodes.push_back(net.pins[root]);
        treeSet.insert(pointKey(net.pins[root].x, net.pins[root].y));
        connected[root] = 1;
        int connectedCount = 1;

        while (connectedCount < static_cast<int>(net.pins.size())) {
            bool absorbedPin = false;
            for (size_t p = 0; p < net.pins.size(); ++p) {
                if (!connected[p] &&
                    treeSet.find(pointKey(net.pins[p].x, net.pins[p].y)) != treeSet.end()) {
                    connected[p] = 1;
                    ++connectedCount;
                    absorbedPin = true;
                }
            }
            if (connectedCount >= static_cast<int>(net.pins.size()))
                break;
            if (absorbedPin)
                continue;

            int bestPin = -1;
            int bestDist = std::numeric_limits<int>::max();
            double bestMetric = std::numeric_limits<double>::infinity();
            GridPoint bestAnchor = treeNodes[0];
            const size_t treeStride = std::max<size_t>(1, treeNodes.size() / 4096);

            for (size_t p = 0; p < net.pins.size(); ++p) {
                if (connected[p])
                    continue;

                int pinBest = std::numeric_limits<int>::max();
                GridPoint pinAnchor = treeNodes[0];
                for (size_t t = 0; t < treeNodes.size(); t += treeStride) {
                    const int dist = manhattan(net.pins[p], treeNodes[t]);
                    if (dist < pinBest) {
                        pinBest = dist;
                        pinAnchor = treeNodes[t];
                    }
                }

                const std::vector<GridPoint> guide = monotonicPath(net, net.pins[p], pinAnchor, iter);
                const double pinMetric = pathCost(net, guide, iter, true);
                if (pinMetric < bestMetric) {
                    bestMetric = pinMetric;
                    bestDist = pinBest;
                    bestPin = static_cast<int>(p);
                    bestAnchor = pinAnchor;
                }
            }

            if (bestPin < 0)
                return false;

            std::vector<GridPoint> path = shortestPath(net, net.pins[bestPin], bestAnchor, treeSet, bestDist, iter);
            if (path.empty())
                path = monotonicPath(net, net.pins[bestPin], bestAnchor, iter);
            trimAtExistingTree(path, treeSet);
            if (path.size() <= 1)
                return false;

            RouteTask task;
            task.netIndex = netIndex;
            task.source = path.front();
            task.target = path.back();
            task.hpwl = std::max(1, manhattan(task.source, task.target));
            task.previousWireLength = std::max(task.hpwl, static_cast<int>(path.size() - 1));
            task.lastOverflow = 0;
            commitTaskPath(task, path);
            candidateTasks.push_back(task);

            for (size_t i = 0; i < path.size(); ++i) {
                const long long key = pointKey(path[i].x, path[i].y);
                if (treeSet.insert(key).second)
                    treeNodes.push_back(path[i]);
            }
            connected[bestPin] = 1;
            ++connectedCount;
        }

        return !candidateTasks.empty();
    }

    void multipinTreeRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.totalOverflow > 2048)
            return;

        int staleRounds = 0;
        for (int round = 0; round < 6 && cur.totalOverflow > 0; ++round) {
            const int roundStartOverflow = cur.totalOverflow;
            std::vector<NetScore> rerouteList = collectOverflowedNets();
            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            std::vector<std::vector<int>> tasksByNet(routes.size());
            for (size_t i = 0; i < tasks.size(); ++i)
                tasksByNet[tasks[i].netIndex].push_back(static_cast<int>(i));

            std::vector<char> selected(routes.size(), 0);
            std::vector<int> netOrder;
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                const int netIndex = tasks[rerouteList[i].routeIndex].netIndex;
                if (selected[netIndex])
                    continue;
                selected[netIndex] = 1;
                netOrder.push_back(netIndex);
                if (netOrder.size() >= 2048)
                    break;
            }

            int accepted = 0;
            for (size_t ni = 0; ni < netOrder.size() && cur.totalOverflow > 0; ++ni) {
                const int netIndex = netOrder[ni];
                const std::vector<int> &ids = tasksByNet[netIndex];
                if (ids.empty())
                    continue;

                OverflowSummary before = cur;
                size_t oldLength = 0;
                std::vector<std::vector<EdgeUse>> oldEdges(ids.size());
                std::vector<ISPDParser::TwoPin> oldTwopins(ids.size());
                std::vector<int> oldPrevious(ids.size());

                for (size_t i = 0; i < ids.size(); ++i) {
                    RouteTask &task = tasks[ids[i]];
                    oldEdges[i] = task.edges;
                    oldTwopins[i] = task.two;
                    oldPrevious[i] = task.previousWireLength;
                    oldLength += task.edges.size();
                }

                for (size_t i = 0; i < ids.size(); ++i)
                    ripUpTask(tasks[ids[i]]);

                std::vector<RouteTask> candidateTasks;
                const bool built = buildMultipinTreeCandidate(
                    netIndex, firstIter + round * 8 + static_cast<int>(ni % 8), candidateTasks);
                OverflowSummary after = built ? computeOverflow() : before;

                size_t newLength = 0;
                for (size_t i = 0; i < candidateTasks.size(); ++i)
                    newLength += candidateTasks[i].edges.size();

                const bool improvesOverflow = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && newLength < oldLength;

                if (built && (improvesOverflow || improvesLength)) {
                    for (size_t i = 0; i < candidateTasks.size(); ++i)
                        tasks.push_back(candidateTasks[i]);
                    cur = after;
                    ++accepted;
                } else {
                    for (size_t i = 0; i < candidateTasks.size(); ++i)
                        ripUpTask(candidateTasks[i]);

                    for (size_t i = 0; i < ids.size(); ++i) {
                        RouteTask &task = tasks[ids[i]];
                        task.edges = oldEdges[i];
                        task.two = oldTwopins[i];
                        task.previousWireLength = oldPrevious[i];
                        for (size_t j = 0; j < task.edges.size(); ++j)
                            addNetEdgeUse(routes[task.netIndex], task.edges[j]);
                    }
                    cur = before;
                }
            }

            if (verbose)
                std::cerr << "2D multipin=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " accepted=" << accepted << "\n";

            if (cur.totalOverflow < roundStartOverflow)
                staleRounds = 0;
            else
                ++staleRounds;

            if (accepted == 0 || staleRounds >= 3)
                break;
        }
    }

    void maxOverflowRefine(OverflowSummary &cur, int firstIter, bool verbose) {
        if (cur.totalOverflow == 0 || cur.maxOverflow <= 1 || cur.totalOverflow > 1024)
            return;

        RouteSnapshot bestSnapshot = makeSnapshot();
        OverflowSummary best = cur;
        const int totalBudget = cur.totalOverflow + std::max(4, cur.maxOverflow * 2);
        int staleRounds = 0;

        for (int round = 0; round < 16 && cur.maxOverflow > 1; ++round) {
            std::vector<NetScore> rerouteList;
            rerouteList.reserve(tasks.size() / 32 + 1);
            const int threshold = std::max(1, cur.maxOverflow);
            bool improvedBestInRound = false;

            for (size_t i = 0; i < tasks.size(); ++i) {
                RouteTask &task = tasks[i];
                int amount = 0;
                int edges = 0;
                for (size_t e = 0; e < task.edges.size(); ++e) {
                    const int overflow = edgeDemand(task.edges[e]) - edgeCapacity(task.edges[e]);
                    if (overflow >= threshold) {
                        amount += overflow;
                        ++edges;
                    }
                }
                if (edges > 0) {
                    NetScore score;
                    score.routeIndex = static_cast<int>(i);
                    score.overflowAmount = amount;
                    score.overflowEdges = edges;
                    score.wireLength = static_cast<int>(task.edges.size());
                    rerouteList.push_back(score);
                }
            }

            if (rerouteList.empty())
                break;

            sortRerouteList(rerouteList);
            if (rerouteList.size() > 8000)
                rerouteList.resize(8000);

            resetOverflowFrequency();
            int accepted = 0;
            for (size_t i = 0; i < rerouteList.size(); ++i) {
                RouteTask &task = tasks[rerouteList[i].routeIndex];
                const std::vector<EdgeUse> oldEdges = task.edges;
                const ISPDParser::TwoPin oldTwoPin = task.two;
                const int oldPrevious = task.previousWireLength;
                const size_t oldLength = task.edges.size();
                const OverflowSummary before = cur;

                clearMarks();
                markPeakOverflowEdges(task, threshold, 256);
                ripUpTask(task);
                routeTask(task, true, firstIter + round);
                clearMarks();

                const OverflowSummary after = computeOverflow();
                const bool improvesMax = after.maxOverflow < before.maxOverflow &&
                    after.totalOverflow <= totalBudget;
                const bool improvesTotal = after.totalOverflow < before.totalOverflow ||
                    (after.totalOverflow == before.totalOverflow && after.maxOverflow < before.maxOverflow);
                const bool improvesLength = after.totalOverflow == before.totalOverflow &&
                    after.maxOverflow == before.maxOverflow && task.edges.size() < oldLength;

                if (improvesMax || improvesTotal || improvesLength) {
                    cur = after;
                    recordOverflowFrequency(task);
                    ++accepted;
                    if (cur.maxOverflow < best.maxOverflow ||
                        (cur.maxOverflow == best.maxOverflow && cur.totalOverflow < best.totalOverflow)) {
                        best = cur;
                        bestSnapshot = makeSnapshot();
                        improvedBestInRound = true;
                    }
                    if (cur.totalOverflow == 0 || cur.maxOverflow <= 1)
                        break;
                } else {
                    ripUpTask(task);
                    task.edges = oldEdges;
                    task.two = oldTwoPin;
                    task.previousWireLength = oldPrevious;
                    for (size_t e = 0; e < task.edges.size(); ++e)
                        addNetEdgeUse(routes[task.netIndex], task.edges[e]);
                    cur = before;
                }
            }

            applyOverflowFrequencyToHistory();
            if (verbose)
                std::cerr << "2D maxof=" << round + 1 << " overflow=" << cur.totalOverflow
                          << " max=" << cur.maxOverflow << " accepted=" << accepted << "\n";

            if (improvedBestInRound)
                staleRounds = 0;
            else
                ++staleRounds;

            if (accepted == 0 || staleRounds >= 4)
                break;
        }

        if (best.maxOverflow < cur.maxOverflow ||
            (best.maxOverflow == cur.maxOverflow && best.totalOverflow < cur.totalOverflow)) {
            restoreSnapshot(bestSnapshot);
            cur = computeOverflow();
        }
    }

    RouteSnapshot makeSnapshot() const {
        RouteSnapshot snap;
        snap.edges.resize(tasks.size());
        snap.twopins.resize(tasks.size());
        snap.previousWireLengths.resize(tasks.size());
        for (size_t i = 0; i < tasks.size(); ++i) {
            snap.edges[i] = tasks[i].edges;
            snap.twopins[i] = tasks[i].two;
            snap.previousWireLengths[i] = tasks[i].previousWireLength;
        }
        return snap;
    }

    void restoreSnapshot(const RouteSnapshot &snap) {
        hDemand.assign(hDemand.size(), 0);
        vDemand.assign(vDemand.size(), 0);
        clearMarks();
        for (size_t i = 0; i < routes.size(); ++i) {
            routes[i].usedEdges.clear();
            routes[i].usedEdgeCount.clear();
            routes[i].usedNodes.clear();
            routes[i].usedNodeCount.clear();
        }

        if (tasks.size() > snap.edges.size())
            tasks.resize(snap.edges.size());

        for (size_t i = 0; i < tasks.size(); ++i) {
            tasks[i].edges = snap.edges[i];
            tasks[i].two = snap.twopins[i];
            tasks[i].previousWireLength = snap.previousWireLengths[i];
            for (size_t j = 0; j < tasks[i].edges.size(); ++j) {
                addNetEdgeUse(routes[tasks[i].netIndex], tasks[i].edges[j]);
            }
        }
    }

    void routeTask(RouteTask &task, bool maze, int iter) {
        RouteNet &net = routes[task.netIndex];
        std::vector<GridPoint> path;

        if (maze)
            path = shortestTaskPath(task, net, iter);

        if (path.empty()) {
            const int extension = patternExtensionLimit(iter);
            path = patternPath(net, task.source, task.target, iter, maze || iter > 0, extension);
        }

        commitTaskPath(task, path);
    }

    int manhattan(const GridPoint &a, const GridPoint &b) const {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    int chooseRootPin(const std::vector<GridPoint> &pins) const {
        long long sumX = 0;
        long long sumY = 0;
        for (size_t i = 0; i < pins.size(); ++i) {
            sumX += pins[i].x;
            sumY += pins[i].y;
        }

        const double cx = static_cast<double>(sumX) / static_cast<double>(pins.size());
        const double cy = static_cast<double>(sumY) / static_cast<double>(pins.size());
        int best = 0;
        double bestDist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < pins.size(); ++i) {
            const double dist = std::abs(static_cast<double>(pins[i].x) - cx) +
                                std::abs(static_cast<double>(pins[i].y) - cy);
            if (dist < bestDist) {
                bestDist = dist;
                best = static_cast<int>(i);
            }
        }
        return best;
    }

    std::vector<GridPoint> monotonicPath(const RouteNet &net,
                                         const GridPoint &source,
                                         const GridPoint &target,
                                         int iter) const {
        return patternPath(net, source, target, iter, false, 0);
    }

    std::unordered_set<long long> connectedComponent(const RouteNet &net,
                                                     const GridPoint &seed) const {
        std::unordered_set<long long> component;
        const long long seedKey = pointKey(seed.x, seed.y);
        if (net.usedNodes.find(seedKey) == net.usedNodes.end())
            return component;

        std::queue<long long> wave;
        component.insert(seedKey);
        wave.push(seedKey);

        const int dirs[4][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        while (!wave.empty()) {
            const long long key = wave.front();
            wave.pop();

            const int x = static_cast<int>(key % xNum);
            const int y = static_cast<int>(key / xNum);

            for (int d = 0; d < 4; ++d) {
                const GridPoint next(x + dirs[d][0], y + dirs[d][1]);
                if (next.x < 0 || next.x >= xNum || next.y < 0 || next.y >= yNum)
                    continue;

                const EdgeUse edge = makeEdge(GridPoint(x, y), next);
                if (net.usedEdges.find(edgeKey(edge)) == net.usedEdges.end())
                    continue;

                const long long nextKey = pointKey(next.x, next.y);
                if (component.insert(nextKey).second)
                    wave.push(nextKey);
            }
        }

        return component;
    }

    GridPoint nearestNodeInSet(const std::unordered_set<long long> &nodes,
                               const GridPoint &source,
                               int *bestDist) const {
        GridPoint best = source;
        int distBest = std::numeric_limits<int>::max();
        for (std::unordered_set<long long>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
            const int x = static_cast<int>(*it % xNum);
            const int y = static_cast<int>(*it / xNum);
            const int d = std::abs(source.x - x) + std::abs(source.y - y);
            if (d < distBest) {
                distBest = d;
                best = GridPoint(x, y);
            }
        }

        if (bestDist != NULL)
            *bestDist = (distBest == std::numeric_limits<int>::max()) ? 0 : distBest;
        return best;
    }

    std::vector<GridPoint> pathToComponent(const RouteNet &net,
                                           const GridPoint &source,
                                           const std::unordered_set<long long> &component,
                                           int iter) const {
        if (component.empty())
            return std::vector<GridPoint>();

        int bestDist = 0;
        const GridPoint anchor = nearestNodeInSet(component, source, &bestDist);
        if (bestDist <= 0)
            return std::vector<GridPoint>();

        std::vector<GridPoint> path = shortestPath(net, source, anchor, component, bestDist, iter);
        if (path.empty())
            path = monotonicPath(net, source, anchor, iter);
        trimAtExistingTree(path, component);
        return path;
    }

    void appendVertical(std::vector<GridPoint> &path, int y) const {
        if (path.empty())
            return;
        GridPoint cur = path.back();
        while (cur.y != y) {
            cur.y += (y > cur.y) ? 1 : -1;
            path.push_back(cur);
        }
    }

    void appendHorizontal(std::vector<GridPoint> &path, int x) const {
        if (path.empty())
            return;
        GridPoint cur = path.back();
        while (cur.x != x) {
            cur.x += (x > cur.x) ? 1 : -1;
            path.push_back(cur);
        }
    }

    std::vector<GridPoint> buildHTrunkPath(const GridPoint &source,
                                           const GridPoint &target,
                                           int trunkY) const {
        std::vector<GridPoint> path;
        path.push_back(source);
        appendVertical(path, trunkY);
        appendHorizontal(path, target.x);
        appendVertical(path, target.y);
        return path;
    }

    std::vector<GridPoint> buildVTrunkPath(const GridPoint &source,
                                           const GridPoint &target,
                                           int trunkX) const {
        std::vector<GridPoint> path;
        path.push_back(source);
        appendHorizontal(path, trunkX);
        appendVertical(path, target.y);
        appendHorizontal(path, target.x);
        return path;
    }

    void addCandidateValue(std::vector<int> &values, int value, int lo, int hi) const {
        value = std::max(lo, std::min(hi, value));
        values.push_back(value);
    }

    std::vector<int> patternCandidateValues(int a, int b, int gridMax, int extension) const {
        const int lo = std::min(a, b);
        const int hi = std::max(a, b);
        std::vector<int> values;
        values.reserve(8);
        addCandidateValue(values, a, 0, gridMax);
        addCandidateValue(values, b, 0, gridMax);
        addCandidateValue(values, (lo + hi) / 2, 0, gridMax);
        if (extension > 0) {
            const int half = std::max(1, extension / 2);
            addCandidateValue(values, lo - half, 0, gridMax);
            addCandidateValue(values, hi + half, 0, gridMax);
            addCandidateValue(values, lo - extension, 0, gridMax);
            addCandidateValue(values, hi + extension, 0, gridMax);
        }
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
        return values;
    }

    std::vector<GridPoint> patternPath(const RouteNet &net,
                                       const GridPoint &source,
                                       const GridPoint &target,
                                       int iter,
                                       bool mazeCost,
                                       int extension) const {
        std::vector<GridPoint> best;
        double bestCost = std::numeric_limits<double>::infinity();
        int bestLen = std::numeric_limits<int>::max();

        const std::vector<int> yCandidates =
            patternCandidateValues(source.y, target.y, yNum - 1, extension);
        for (size_t i = 0; i < yCandidates.size(); ++i) {
            std::vector<GridPoint> path = buildHTrunkPath(source, target, yCandidates[i]);
            const int len = static_cast<int>(path.size()) - 1;
            const double costValue = pathCost(net, path, iter, mazeCost);
            if (costValue + 1e-9 < bestCost ||
                (std::abs(costValue - bestCost) <= 1e-9 && len < bestLen)) {
                bestCost = costValue;
                bestLen = len;
                best = path;
            }
        }

        const std::vector<int> xCandidates =
            patternCandidateValues(source.x, target.x, xNum - 1, extension);
        for (size_t i = 0; i < xCandidates.size(); ++i) {
            std::vector<GridPoint> path = buildVTrunkPath(source, target, xCandidates[i]);
            const int len = static_cast<int>(path.size()) - 1;
            const double costValue = pathCost(net, path, iter, mazeCost);
            if (costValue + 1e-9 < bestCost ||
                (std::abs(costValue - bestCost) <= 1e-9 && len < bestLen)) {
                bestCost = costValue;
                bestLen = len;
                best = path;
            }
        }

        if (best.empty())
            best = buildLPath(source, target, true);
        return best;
    }

    std::vector<GridPoint> buildLPath(const GridPoint &source, const GridPoint &target, bool horizontalFirst) const {
        std::vector<GridPoint> path;
        GridPoint cur = source;
        path.push_back(cur);

        if (horizontalFirst) {
            while (cur.x != target.x) {
                cur.x += (target.x > cur.x) ? 1 : -1;
                path.push_back(cur);
            }
            while (cur.y != target.y) {
                cur.y += (target.y > cur.y) ? 1 : -1;
                path.push_back(cur);
            }
        } else {
            while (cur.y != target.y) {
                cur.y += (target.y > cur.y) ? 1 : -1;
                path.push_back(cur);
            }
            while (cur.x != target.x) {
                cur.x += (target.x > cur.x) ? 1 : -1;
                path.push_back(cur);
            }
        }

        return path;
    }

    double pathCost(const RouteNet &net, const std::vector<GridPoint> &path, int iter, bool maze) const {
        double total = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
            total += edgeCost(makeEdge(path[i - 1], path[i]), iter, maze, &net);
        return total;
    }

    double postEdgeCost(const EdgeUse &edge, const RouteNet *net) const {
        const int cap = edgeCapacity(edge);
        const int dem = edgeDemand(edge) + defaultTrackDemand;
        if (cap <= 0)
            return 100000.0 + static_cast<double>(dem) * 500.0;

        double cost = 1.0 + 3.0 * static_cast<double>(dem) / static_cast<double>(cap + 1);
        if (dem > cap) {
            const double overflow = static_cast<double>(dem - cap);
            cost += 80.0 * overflow * overflow;
        }

        if (net != NULL) {
            const long long key = edgeKey(edge);
            if (net->usedEdges.find(key) != net->usedEdges.end())
                cost *= 0.20;
            else if (net->preferredEdges.find(key) != net->preferredEdges.end())
                cost *= 0.90;
        }

        return cost;
    }

    double postPathCost(const RouteNet &net, const std::vector<GridPoint> &path) const {
        double total = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
            total += postEdgeCost(makeEdge(path[i - 1], path[i]), &net);
        return total;
    }

    std::vector<GridPoint> monotonicPostPath(const RouteNet &net,
                                             const GridPoint &source,
                                             const GridPoint &target) const {
        std::vector<GridPoint> hv = buildLPath(source, target, true);
        std::vector<GridPoint> vh = buildLPath(source, target, false);
        const double hvCost = postPathCost(net, hv);
        const double vhCost = postPathCost(net, vh);
        return (vhCost < hvCost) ? vh : hv;
    }

    std::vector<GridPoint> postRefinePath(const RouteTask &task,
                                          const RouteNet &net,
                                          int oldLength) const {
        if (oldLength <= task.hpwl)
            return monotonicPostPath(net, task.source, task.target);

        std::vector<GridPoint> path =
            boundedTaskSearch(task, net, 1, std::max(task.hpwl, oldLength), true, true);
        if (path.empty())
            path = monotonicPostPath(net, task.source, task.target);
        return path;
    }

    int lengthBound(const RouteNet &net, int estimatedDist, int iter) const {
        if (estimatedDist <= 0)
            return 0;

        const bool compactCase = db.nets.size() < 300000;
        const double alpha = 9.0;
        const double beta = compactCase ? 4.0 : 8.0;
        double factor = 1.0 + std::atan(static_cast<double>(iter) - alpha) + beta;
        if (factor < 1.0)
            factor = 1.0;

        int bound = static_cast<int>(std::ceil(static_cast<double>(estimatedDist) * factor));
        const int history = historyLengthEstimate(net, estimatedDist);
        if (iter <= 5) {
            const int earlySlack = compactCase ? estimatedDist : estimatedDist * 2;
            bound = std::min(bound, history + std::max(8, earlySlack));
        }
        return std::max(estimatedDist, bound);
    }

    int historyLengthEstimate(const RouteNet &net, int estimatedDist) const {
        if (estimatedDist <= 0)
            return 0;

        double ratio = 1.0;
        if (net.hpwl > 0 && net.previousWireLength > 0)
            ratio = static_cast<double>(net.previousWireLength) / static_cast<double>(net.hpwl);
        ratio = std::max(1.0, std::min(3.0, ratio));
        return std::max(estimatedDist,
                        static_cast<int>(std::ceil(static_cast<double>(estimatedDist) * ratio)));
    }

    bool preferMazeCandidate(double candCost,
                             int candLen,
                             bool candSlack,
                             double oldCost,
                             int oldLen,
                             bool oldSlack) const {
        if (oldLen == std::numeric_limits<int>::max())
            return true;

        if (candSlack != oldSlack)
            return candSlack;

        if (candSlack) {
            if (candCost + 1e-9 < oldCost)
                return true;
            if (std::abs(candCost - oldCost) <= 1e-9 && candLen < oldLen)
                return true;
            return false;
        }

        if (candLen != oldLen)
            return candLen < oldLen;
        return candCost + 1e-9 < oldCost;
    }

    std::vector<GridPoint> boundedMazeSearch(const RouteNet &net,
                                             const GridPoint &source,
                                             const GridPoint &target,
                                             const std::unordered_set<long long> &treeSet,
                                             int estimatedDist,
                                             int iter,
                                             int bound,
                                             bool bounded) const {
        int baseMargin = std::max(5, estimatedDist / 7 + 3);
        if (bounded) {
            const int slack = std::max(0, bound - estimatedDist);
            baseMargin = std::max(3, slack / 2 + 2);
        }
        const int margin = std::min(searchMarginCap(), baseMargin + (bounded ? std::min(iter, 6) : iter * 2));
        const int xLo = std::max(0, std::min(source.x, target.x) - margin);
        const int xHi = std::min(xNum - 1, std::max(source.x, target.x) + margin);
        const int yLo = std::max(0, std::min(source.y, target.y) - margin);
        const int yHi = std::min(yNum - 1, std::max(source.y, target.y) + margin);
        const int lx = xHi - xLo + 1;
        const int ly = yHi - yLo + 1;
        const int localSize = lx * ly;

        std::vector<double> dist(localSize, std::numeric_limits<double>::infinity());
        std::vector<int> pathLen(localSize, std::numeric_limits<int>::max());
        std::vector<char> enoughSlack(localSize, 0);
        std::vector<int> parent(localSize, -1);
        std::vector<char> closed(localSize, 0);
        std::priority_queue<HeapNode> heap;

        const int start = localIndex(source.x, source.y, xLo, yLo, lx);
        const int goal = localIndex(target.x, target.y, xLo, yLo, lx);
        dist[start] = 0.0;
        pathLen[start] = 0;
        enoughSlack[start] = 1;
        heap.push(HeapNode{static_cast<double>(manhattan(source, target)), start});
        int reached = -1;
        const int historyLength = historyLengthEstimate(net, estimatedDist);
        const int sourceTargetDist = std::max(1, estimatedDist);

        const int dirs[4][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        while (!heap.empty()) {
            const HeapNode cur = heap.top();
            heap.pop();
            if (closed[cur.idx])
                continue;
            closed[cur.idx] = 1;

            const GridPoint node = globalPoint(cur.idx, xLo, yLo, lx);
            if (cur.idx != start && treeSet.find(pointKey(node.x, node.y)) != treeSet.end()) {
                reached = cur.idx;
                break;
            }

            for (int d = 0; d < 4; ++d) {
                const GridPoint nxt(node.x + dirs[d][0], node.y + dirs[d][1]);
                if (nxt.x < xLo || nxt.x > xHi || nxt.y < yLo || nxt.y > yHi)
                    continue;

                const int ni = localIndex(nxt.x, nxt.y, xLo, yLo, lx);
                if (closed[ni])
                    continue;
                if (pathLen[cur.idx] == std::numeric_limits<int>::max())
                    continue;

                const int candLen = pathLen[cur.idx] + 1;
                const int remManh = std::abs(nxt.x - target.x) + std::abs(nxt.y - target.y);
                if (bounded && candLen + remManh > bound)
                    continue;

                const double nd = dist[cur.idx] + edgeCost(makeEdge(node, nxt), iter, true, &net);
                bool candSlack = true;
                if (bounded) {
                    const double estimatedRemain =
                        static_cast<double>(historyLength) * static_cast<double>(remManh) /
                        static_cast<double>(sourceTargetDist);
                    candSlack = static_cast<double>(candLen) + estimatedRemain <=
                                static_cast<double>(bound) + 1e-9;
                }

                if (preferMazeCandidate(nd, candLen, candSlack, dist[ni], pathLen[ni], enoughSlack[ni] != 0)) {
                    dist[ni] = nd;
                    pathLen[ni] = candLen;
                    enoughSlack[ni] = candSlack ? 1 : 0;
                    parent[ni] = cur.idx;
                    const double key = nd + static_cast<double>(std::abs(nxt.x - target.x) + std::abs(nxt.y - target.y));
                    heap.push(HeapNode{key, ni});
                }
            }
        }

        if (reached < 0)
            reached = goal;

        if (parent[reached] < 0 && reached != start)
            return std::vector<GridPoint>();

        std::vector<GridPoint> rev;
        for (int at = reached; at >= 0; at = parent[at]) {
            rev.push_back(globalPoint(at, xLo, yLo, lx));
            if (at == start)
                break;
        }
        std::reverse(rev.begin(), rev.end());
        return rev;
    }

    std::vector<GridPoint> shortestPath(const RouteNet &net,
                                        const GridPoint &source,
                                        const GridPoint &target,
                                        const std::unordered_set<long long> &treeSet,
                                        int estimatedDist,
                                        int iter) const {
        const int strictBound = lengthBound(net, estimatedDist, iter);
        std::vector<GridPoint> path =
            boundedMazeSearch(net, source, target, treeSet, estimatedDist, iter, strictBound, true);
        if (!path.empty())
            return path;

        const int relaxedBound = strictBound + std::max(4, estimatedDist / 4);
        path = boundedMazeSearch(net, source, target, treeSet, estimatedDist, iter, relaxedBound, true);
        if (!path.empty())
            return path;

        return boundedMazeSearch(net, source, target, treeSet, estimatedDist, iter, strictBound, false);
    }

    int taskHistoryLengthEstimate(const RouteTask &task) const {
        if (task.hpwl <= 0)
            return 0;

        double ratio = static_cast<double>(std::max(task.hpwl, task.previousWireLength)) /
                       static_cast<double>(task.hpwl);
        ratio = std::max(1.0, std::min(3.0, ratio));
        return std::max(task.hpwl, static_cast<int>(std::ceil(static_cast<double>(task.hpwl) * ratio)));
    }

    int taskLengthBound(const RouteTask &task, int iter) const {
        if (task.hpwl <= 0)
            return 0;

        const bool compactCase = db.nets.size() < 300000;
        const double alpha = 9.0;
        const double beta = compactCase ? 4.0 : 8.0;
        double factor = 1.0 + std::atan(static_cast<double>(iter) - alpha) + beta;
        if (factor < 1.0)
            factor = 1.0;

        int bound = static_cast<int>(std::ceil(static_cast<double>(task.hpwl) * factor));
        const int history = taskHistoryLengthEstimate(task);
        if (iter <= 5) {
            const int earlySlack = compactCase ? task.hpwl : task.hpwl * 2;
            bound = std::min(bound, history + std::max(8, earlySlack));
        }
        return std::max(task.hpwl, bound);
    }

    std::vector<GridPoint> boundedTaskSearch(const RouteTask &task,
                                             const RouteNet &net,
                                             int iter,
                                             int bound,
                                             bool bounded,
                                             bool postMode = false) const {
        const GridPoint &source = task.source;
        const GridPoint &target = task.target;
        int baseMargin = std::max(5, task.hpwl / 7 + 3);
        if (bounded) {
            const int slack = std::max(0, bound - task.hpwl);
            baseMargin = std::max(3, slack / 2 + 2);
        }

        const int margin = std::min(searchMarginCap(), baseMargin + (bounded ? std::min(iter, 6) : iter * 2));
        const int xLo = std::max(0, std::min(source.x, target.x) - margin);
        const int xHi = std::min(xNum - 1, std::max(source.x, target.x) + margin);
        const int yLo = std::max(0, std::min(source.y, target.y) - margin);
        const int yHi = std::min(yNum - 1, std::max(source.y, target.y) + margin);
        const int lx = xHi - xLo + 1;
        const int ly = yHi - yLo + 1;
        const int localSize = lx * ly;

        std::vector<double> dist(localSize, std::numeric_limits<double>::infinity());
        std::vector<int> pathLen(localSize, std::numeric_limits<int>::max());
        std::vector<char> enoughSlack(localSize, 0);
        std::vector<int> parent(localSize, -1);
        std::vector<char> closed(localSize, 0);
        std::priority_queue<HeapNode> heap;

        const int start = localIndex(source.x, source.y, xLo, yLo, lx);
        const int goal = localIndex(target.x, target.y, xLo, yLo, lx);
        dist[start] = 0.0;
        pathLen[start] = 0;
        enoughSlack[start] = 1;
        heap.push(HeapNode{static_cast<double>(task.hpwl), start});
        const int historyLength = taskHistoryLengthEstimate(task);
        const int sourceTargetDist = std::max(1, task.hpwl);

        const int dirs[4][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        while (!heap.empty()) {
            const HeapNode cur = heap.top();
            heap.pop();
            if (closed[cur.idx])
                continue;
            closed[cur.idx] = 1;

            if (cur.idx == goal)
                break;

            const GridPoint node = globalPoint(cur.idx, xLo, yLo, lx);
            for (int d = 0; d < 4; ++d) {
                const GridPoint nxt(node.x + dirs[d][0], node.y + dirs[d][1]);
                if (nxt.x < xLo || nxt.x > xHi || nxt.y < yLo || nxt.y > yHi)
                    continue;

                const int ni = localIndex(nxt.x, nxt.y, xLo, yLo, lx);
                if (closed[ni] || pathLen[cur.idx] == std::numeric_limits<int>::max())
                    continue;

                const int candLen = pathLen[cur.idx] + 1;
                const int remManh = std::abs(nxt.x - target.x) + std::abs(nxt.y - target.y);
                if (bounded && candLen + remManh > bound)
                    continue;

                const EdgeUse step = makeEdge(node, nxt);
                const double stepCost = postMode
                    ? postEdgeCost(step, &net)
                    : edgeCost(step, iter, true, &net);
                const double nd = dist[cur.idx] + stepCost;
                bool candSlack = true;
                if (bounded) {
                    const double estimatedRemain =
                        static_cast<double>(historyLength) * static_cast<double>(remManh) /
                        static_cast<double>(sourceTargetDist);
                    candSlack = static_cast<double>(candLen) + estimatedRemain <=
                                static_cast<double>(bound) + 1e-9;
                }

                if (preferMazeCandidate(nd, candLen, candSlack, dist[ni], pathLen[ni], enoughSlack[ni] != 0)) {
                    dist[ni] = nd;
                    pathLen[ni] = candLen;
                    enoughSlack[ni] = candSlack ? 1 : 0;
                    parent[ni] = cur.idx;
                    const double key = nd + static_cast<double>(remManh);
                    heap.push(HeapNode{key, ni});
                }
            }
        }

        if (parent[goal] < 0 && goal != start)
            return std::vector<GridPoint>();

        std::vector<GridPoint> rev;
        for (int at = goal; at >= 0; at = parent[at]) {
            rev.push_back(globalPoint(at, xLo, yLo, lx));
            if (at == start)
                break;
        }
        std::reverse(rev.begin(), rev.end());
        return rev;
    }

    std::vector<GridPoint> shortestTaskPath(const RouteTask &task,
                                            const RouteNet &net,
                                            int iter) const {
        const int strictBound = taskLengthBound(task, iter);
        std::vector<GridPoint> path = boundedTaskSearch(task, net, iter, strictBound, true);
        if (!path.empty())
            return path;

        const int relaxedBound = strictBound + std::max(4, task.hpwl / 4);
        path = boundedTaskSearch(task, net, iter, relaxedBound, true);
        if (!path.empty())
            return path;

        return boundedTaskSearch(task, net, iter, strictBound, false);
    }

    int localIndex(int x, int y, int xLo, int yLo, int lx) const {
        return (y - yLo) * lx + (x - xLo);
    }

    GridPoint globalPoint(int idx, int xLo, int yLo, int lx) const {
        const int yOff = idx / lx;
        const int xOff = idx - yOff * lx;
        return GridPoint(xLo + xOff, yLo + yOff);
    }

    void trimAtExistingTree(std::vector<GridPoint> &path, const std::unordered_set<long long> &treeSet) const {
        if (path.size() <= 1)
            return;

        for (size_t i = 1; i < path.size(); ++i) {
            if (treeSet.find(pointKey(path[i].x, path[i].y)) != treeSet.end()) {
                path.resize(i + 1);
                return;
            }
        }
    }

    void commitTaskPath(RouteTask &task, const std::vector<GridPoint> &path) {
        if (path.size() <= 1)
            return;

        RouteNet &net = routes[task.netIndex];
        task.two = ISPDParser::TwoPin();
        task.two.from = ISPDParser::Point(path.front().x, path.front().y, 0);
        task.two.to = ISPDParser::Point(path.back().x, path.back().y, 0);
        task.two.parNet = net.src;
        task.two.wlen = static_cast<int>(path.size() - 1);

        for (size_t i = 1; i < path.size(); ++i) {
            const EdgeUse edge = makeEdge(path[i - 1], path[i]);
            addNetEdgeUse(net, edge);
            task.edges.push_back(edge);
            task.two.path.push_back(ISPDParser::RPoint(edge.x, edge.y, 0, edge.hori));
        }
    }

    void ripUpTask(RouteTask &task) {
        RouteNet &net = routes[task.netIndex];
        task.previousWireLength = std::max(task.hpwl, static_cast<int>(task.edges.size()));
        for (size_t i = 0; i < task.edges.size(); ++i) {
            removeNetEdgeUse(net, task.edges[i]);
        }
        task.edges.clear();
        task.two.path.clear();
        task.two.wlen = 0;
    }

    void finalizeTwopins() {
        for (size_t i = 0; i < routes.size(); ++i)
            routes[i].src->twopin.clear();

        for (size_t i = 0; i < tasks.size(); ++i) {
            if (!tasks[i].two.path.empty())
                routes[tasks[i].netIndex].src->twopin.push_back(tasks[i].two);
        }
    }

    void repairDisconnectedNets() {
        const int repairIter = chooseIterationLimit() + 160;

        for (size_t n = 0; n < routes.size(); ++n) {
            RouteNet &net = routes[n];
            if (net.pins.size() <= 1)
                continue;

            const GridPoint root = net.pins[0];
            std::unordered_set<long long> rootComponent = connectedComponent(net, root);
            if (rootComponent.empty())
                rootComponent.insert(pointKey(root.x, root.y));

            for (size_t p = 1; p < net.pins.size(); ++p) {
                const GridPoint pin = net.pins[p];
                if (rootComponent.find(pointKey(pin.x, pin.y)) != rootComponent.end())
                    continue;

                RouteTask repair;
                repair.netIndex = static_cast<int>(n);
                repair.source = pin;
                repair.target = nearestNodeInSet(rootComponent, pin, NULL);
                repair.hpwl = std::max(1, manhattan(repair.source, repair.target));
                repair.previousWireLength = repair.hpwl;
                repair.lastOverflow = 0;

                std::vector<GridPoint> path = pathToComponent(net, pin, rootComponent, repairIter);
                if (path.empty())
                    path = monotonicPath(net, pin, repair.target, repairIter);

                commitTaskPath(repair, path);
                if (!repair.edges.empty())
                    tasks.push_back(repair);

                rootComponent = connectedComponent(net, root);
                if (rootComponent.empty())
                    rootComponent.insert(pointKey(root.x, root.y));
            }
        }
    }

    void postRefine() {
        OverflowSummary before = computeOverflow();
        if (before.totalOverflow != 0)
            return;

        std::vector<int> order(tasks.size());
        for (size_t i = 0; i < tasks.size(); ++i)
            order[i] = static_cast<int>(i);
        std::sort(order.begin(), order.end(), [this](int a, int b) {
            if (tasks[a].edges.size() != tasks[b].edges.size())
                return tasks[a].edges.size() > tasks[b].edges.size();
            return tasks[a].hpwl > tasks[b].hpwl;
        });

        size_t limit = order.size();
        if (db.nets.size() < 300000)
            limit = std::min<size_t>(limit, 30000);
        else if (db.nets.size() < 430000)
            limit = std::min<size_t>(limit, 90000);

        for (size_t i = 0; i < limit; ++i) {
            RouteTask &task = tasks[order[i]];
            const std::vector<EdgeUse> oldEdges = task.edges;
            const ISPDParser::TwoPin oldTwoPin = task.two;
            const int oldPrevious = task.previousWireLength;
            const int oldLength = static_cast<int>(oldEdges.size());

            ripUpTask(task);
            const std::vector<GridPoint> path = postRefinePath(task, routes[task.netIndex], oldLength);
            commitTaskPath(task, path);
            OverflowSummary after = computeOverflow();

            if (after.totalOverflow > 0 || task.edges.empty() ||
                static_cast<int>(task.edges.size()) > oldLength) {
                ripUpTask(task);
                task.edges = oldEdges;
                task.two = oldTwoPin;
                task.previousWireLength = oldPrevious;
                for (size_t j = 0; j < task.edges.size(); ++j) {
                    addNetEdgeUse(routes[task.netIndex], task.edges[j]);
                }
            }
        }
    }
};

} // namespace

int main(int argc, char **argv) {
    assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile>");

    std::ifstream fp(argv[1]);
    assert(fp.is_open() && "Failed to open input file");
    ISPDParser::ispdData *data = ISPDParser::parse(fp);
    fp.close();

    GlobalRouter2D router(*data);
    router.buildRouteList();
    router.routeAll();
    router.assignLayersAndWrite(argv[2]);

    delete data;
    return 0;
}
