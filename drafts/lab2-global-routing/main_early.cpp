// #include "ispdData.h"
// #include "LayerAssignment.h"

// #include <iostream>
// #include <fstream>
// #include <algorithm>
// #include <utility>
// #include <string>
// #include <cassert>


// int main(int argc, char **argv) {

//     assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile>");
//     std::ifstream fp(argv[1]);
//     assert(fp.is_open() && "Failed to open input file");
//     ISPDParser::ispdData *ispdData = ISPDParser::parse(fp);
//     fp.close();

//     // std::cout << *ispdData << std::endl;

//     // Convert XY coordinates to grid coordinates
//     // Delete nets that have more than 1000 sinks
//     // Delete nets that have all pins inside the same tile
//     ispdData->nets.erase(std::remove_if(ispdData->nets.begin(), ispdData->nets.end(), [&](ISPDParser::Net *net) {

//         for (auto &pin : net->pins) {

//             int x = (std::get<0>(pin) - ispdData->lowerLeftX) / ispdData->tileWidth;
//             int y = (std::get<1>(pin) - ispdData->lowerLeftY) / ispdData->tileHeight;
//             int z = std::get<2>(pin) - 1;

//             if (std::any_of(net->pin3D.begin(), net->pin3D.end(), [x, y, z](const auto &pin) {
//                 return pin.x == x && pin.y == y && pin.z == z;
//             })) continue;
//             net->pin3D.emplace_back(x, y, z);

//             if (std::any_of(net->pin2D.begin(), net->pin2D.end(), [x, y](const auto &pin) { 
//                 return pin.x == x && pin.y == y;
//             })) continue;
//             net->pin2D.emplace_back(x, y);

//         }

//         return net->pin3D.size() > 1000 || net->pin2D.size() <= 1;

//     }), ispdData->nets.end());
//     ispdData->numNet = ispdData->nets.size();


//     // Describe the usage of the given layer assignment algorithm
//     // Only works for the given input file "3d.txt"
//     if (std::string(argv[1]).find("3d.txt") != std::string::npos) {

//         ISPDParser::Net *net = ispdData->nets[0];

//         // Decompose multi-pin nets into two-pin nets
//         // Since there are only 2 pins in the given net, this step is trivial
//         net->twopin.push_back(ISPDParser::TwoPin());
//         ISPDParser::TwoPin &twoPin = net->twopin.back();
//         twoPin.from = net->pin3D[0];
//         twoPin.to   = net->pin3D[1];

//         // Assume the two pin net is routed
//         // The following code is to assign routing paths to the two-pin net
//         // The routing path is a sequence of routing segments
//         // For a horizontal segment, the start point is the left grid coordinate
//         // For a vertical segment, the start point is the bottom grid coordinate
//         // Please check the figures in https://www.ispd.cc/contests/08/3d.pdf
//         twoPin.parNet = net;
//         twoPin.path.emplace_back(0, 0, true);
//         twoPin.path.emplace_back(1, 0, false);
//         twoPin.path.emplace_back(0, 1, true);
//         twoPin.path.emplace_back(0, 1, false);
//         twoPin.path.emplace_back(0, 2, true);
//         twoPin.path.emplace_back(1, 2, true);
//         twoPin.path.emplace_back(2, 1, false);
//         twoPin.path.emplace_back(2, 0, false);

//         // Assign routing layers to the two-pin net
//         LayerAssignment::Graph graph;
//         graph.initialLA(*ispdData, 1);
//         graph.convertGRtoLA(*ispdData, true);
//         graph.COLA(true);

//         // Output result
//         graph.output3Dresult("3ds1.txt");
//     }

//     delete ispdData;
//     return 0;
// }
#include "ispdData.h"
#include "LayerAssignment.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>
#include <vector>
#include <queue>
#include <cmath>
#include <climits>
#include <set>
#include <numeric>
#include <chrono>

using namespace std;

struct GlobalRouter {
    ISPDParser::ispdData* data;
    int X, Y, numLayer;

    // 2D edge arrays
    // h: horizontal edges (x,y)-(x+1,y), x in [0,X-2], y in [0,Y-1]
    // v: vertical edges (x,y)-(x,y+1), x in [0,X-1], y in [0,Y-2]
    vector<int> h_cap, h_dem;
    vector<int> v_cap, v_dem;
    vector<double> h_his, v_his;

    // Flat array helpers
    inline int hidx(int x, int y) const { return x * Y + y; }
    inline int vidx(int x, int y) const { return x * (Y-1) + y; }
    inline int gidx(int x, int y) const { return x * Y + y; }

    chrono::steady_clock::time_point startTime;
    double timeLimitSec = 1400.0;

    GlobalRouter(ISPDParser::ispdData* d) : data(d) {
        startTime = chrono::steady_clock::now();
        X = d->numXGrid; Y = d->numYGrid; numLayer = d->numLayer;

        vector<int> ws(numLayer);
        for (int l = 0; l < numLayer; l++)
            ws[l] = max(1, d->minimumWidth[l] + d->minimumSpacing[l]);

        h_cap.assign((X-1)*Y, 0);
        h_dem.assign((X-1)*Y, 0);
        h_his.assign((X-1)*Y, 0.0);
        v_cap.assign(X*(Y-1), 0);
        v_dem.assign(X*(Y-1), 0);
        v_his.assign(X*(Y-1), 0.0);

        for (int x = 0; x < X-1; x++)
            for (int y = 0; y < Y; y++)
                for (int l = 0; l < numLayer; l++)
                    h_cap[hidx(x,y)] += d->horizontalCapacity[l] / ws[l];

        for (int x = 0; x < X; x++)
            for (int y = 0; y < Y-1; y++)
                for (int l = 0; l < numLayer; l++)
                    v_cap[vidx(x,y)] += d->verticalCapacity[l] / ws[l];

        // Apply capacity adjustments
        for (auto* adj : d->capacityAdjs) {
            int x1=get<0>(adj->grid1), y1=get<1>(adj->grid1), z1=get<2>(adj->grid1)-1;
            int x2=get<0>(adj->grid2), y2=get<1>(adj->grid2);
            int reduced=adj->reducedCapacityLevel;
            int wss=ws[z1];
            if (x1 != x2) {
                int ex=min(x1,x2), ey=y1;
                if (ex>=0&&ex<X-1&&ey>=0&&ey<Y) {
                    int origCap=d->horizontalCapacity[z1];
                    int delta=(origCap-reduced)/wss;
                    h_cap[hidx(ex,ey)] = max(0, h_cap[hidx(ex,ey)]-delta);
                }
            } else {
                int ex=x1, ey=min(y1,y2);
                if (ex>=0&&ex<X&&ey>=0&&ey<Y-1) {
                    int origCap=d->verticalCapacity[z1];
                    int delta=(origCap-reduced)/wss;
                    v_cap[vidx(ex,ey)] = max(0, v_cap[vidx(ex,ey)]-delta);
                }
            }
        }
    }

    double elapsed() {
        return chrono::duration<double>(chrono::steady_clock::now()-startTime).count();
    }
    bool timeUp() { return elapsed() > timeLimitSec; }

    // Compute edge cost for routing
    inline double hCost(int x, int y, double histW) const {
        int i = hidx(x,y);
        int cap=h_cap[i], dem=h_dem[i];
        if (cap <= 0) return 1e7;
        double his = h_his[i];
        double cong;
        if (dem >= cap) cong = 5.0*(dem-cap+1);
        else cong = (double)dem/cap;
        return 1.0 + cong + histW*his;
    }
    inline double vCost(int x, int y, double histW) const {
        int i = vidx(x,y);
        int cap=v_cap[i], dem=v_dem[i];
        if (cap <= 0) return 1e7;
        double his = v_his[i];
        double cong;
        if (dem >= cap) cong = 5.0*(dem-cap+1);
        else cong = (double)dem/cap;
        return 1.0 + cong + histW*his;
    }

    // Dijkstra routing (correct, no A* stale-check issue)
    vector<ISPDParser::RPoint> route2Pin(int sx, int sy, int tx, int ty, double histW) {
        if (sx==tx && sy==ty) return {};

        int N = X*Y;
        vector<double> dist(N, 1e18);
        vector<int> prevNode(N, -1);
        vector<int> prevDirArr(N, -1);

        using T = pair<double,int>;
        priority_queue<T, vector<T>, greater<T>> pq;

        int src = gidx(sx,sy);
        int dst = gidx(tx,ty);
        dist[src] = 0.0;
        pq.push({0.0, src});

        int dx[] = {1,0,-1,0};
        int dy[] = {0,1,0,-1};

        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            if (d > dist[u]+1e-9) continue;
            if (u == dst) break;

            int x = u/Y, y = u%Y;

            for (int dir=0; dir<4; dir++) {
                int nx=x+dx[dir], ny=y+dy[dir];
                if (nx<0||nx>=X||ny<0||ny>=Y) continue;

                double ec;
                if (dir==0) { if (x>=X-1) continue; ec=hCost(x,y,histW); }
                else if (dir==2) { if (nx>=X-1) continue; ec=hCost(nx,y,histW); }
                else if (dir==1) { if (y>=Y-1) continue; ec=vCost(x,y,histW); }
                else { if (ny>=Y-1) continue; ec=vCost(x,ny,histW); }

                int v = gidx(nx,ny);
                double nd = dist[u] + ec;
                if (nd < dist[v]-1e-9) {
                    dist[v] = nd;
                    prevNode[v] = u;
                    prevDirArr[v] = dir;
                    pq.push({nd, v});
                }
            }
        }

        if (dist[dst] >= 1e17) return {};

        vector<ISPDParser::RPoint> path;
        int cur = dst;
        while (cur != src) {
            int par = prevNode[cur];
            int dir = prevDirArr[cur];
            int cx=cur/Y, cy=cur%Y;
            int px=par/Y, py=par%Y;
            ISPDParser::RPoint rp;
            if (dir==0) rp=ISPDParser::RPoint(px,py,0,true);
            else if (dir==1) rp=ISPDParser::RPoint(px,py,0,false);
            else if (dir==2) rp=ISPDParser::RPoint(cx,cy,0,true);
            else rp=ISPDParser::RPoint(cx,cy,0,false);
            path.push_back(rp);
            cur=par;
        }
        reverse(path.begin(), path.end());
        return path;
    }

    void routeNet(ISPDParser::Net* net, double histW) {
        int numPins = net->pin2D.size();
        if (numPins <= 1) return;

        vector<pair<int,int>> pins;
        for (auto& p : net->pin2D) pins.push_back({p.x, p.y});
        sort(pins.begin(), pins.end());
        pins.erase(unique(pins.begin(),pins.end()), pins.end());
        if ((int)pins.size() <= 1) return;

        net->twopin.clear();

        // Prim-like approach: track all tree nodes for connection point lookup
        // Use a visited grid to mark tree nodes for faster nearest lookup
        vector<bool> inTree(pins.size(), false);
        inTree[0] = true;

        // tree points: store as flat set
        vector<pair<int,int>> treePoints = {pins[0]};

        for (int iter=0; iter<(int)pins.size()-1 && !timeUp(); iter++) {
            // Find unconnected pin with nearest tree point
            int bestPin=-1;
            pair<int,int> bestFrom;
            int bestDist=INT_MAX;

            for (int i=0;i<(int)pins.size();i++) {
                if (inTree[i]) continue;
                // Check against all tree points (fast for small nets)
                for (auto& tp : treePoints) {
                    int d=abs(pins[i].first-tp.first)+abs(pins[i].second-tp.second);
                    if (d < bestDist) { bestDist=d; bestPin=i; bestFrom=tp; }
                }
            }
            if (bestPin==-1) break;

            auto path=route2Pin(bestFrom.first,bestFrom.second,
                                pins[bestPin].first,pins[bestPin].second, histW);

            ISPDParser::TwoPin tp2;
            tp2.from=ISPDParser::Point(bestFrom.first,bestFrom.second);
            tp2.to=ISPDParser::Point(pins[bestPin].first,pins[bestPin].second);
            tp2.parNet=net;
            tp2.path=path;
            net->twopin.push_back(tp2);

            for (auto& rp : path) {
                if (rp.hori) h_dem[hidx(rp.x,rp.y)]++;
                else v_dem[vidx(rp.x,rp.y)]++;
                treePoints.push_back({rp.x,rp.y});
                if (rp.hori) treePoints.push_back({rp.x+1,rp.y});
                else treePoints.push_back({rp.x,rp.y+1});
            }
            inTree[bestPin]=true;
        }
    }

    void unrouteNet(ISPDParser::Net* net) {
        for (auto& tp : net->twopin)
            for (auto& rp : tp.path)
                if (rp.hori) h_dem[hidx(rp.x,rp.y)]=max(0,h_dem[hidx(rp.x,rp.y)]-1);
                else v_dem[vidx(rp.x,rp.y)]=max(0,v_dem[vidx(rp.x,rp.y)]-1);
    }

    int totalOverflow() {
        int tof=0;
        for (int x=0;x<X-1;x++) for (int y=0;y<Y;y++) tof+=max(0,h_dem[hidx(x,y)]-h_cap[hidx(x,y)]);
        for (int x=0;x<X;x++) for (int y=0;y<Y-1;y++) tof+=max(0,v_dem[vidx(x,y)]-v_cap[vidx(x,y)]);
        return tof;
    }

    void updateHistory(double inc) {
        for (int x=0;x<X-1;x++) for (int y=0;y<Y;y++) {
            int i=hidx(x,y);
            if (h_dem[i]>h_cap[i]) h_his[i]+=inc*(h_dem[i]-h_cap[i]);
        }
        for (int x=0;x<X;x++) for (int y=0;y<Y-1;y++) {
            int i=vidx(x,y);
            if (v_dem[i]>v_cap[i]) v_his[i]+=inc*(v_dem[i]-v_cap[i]);
        }
    }

    int getHPWL(ISPDParser::Net* net) {
        if (net->pin2D.empty()) return 0;
        int mnx=INT_MAX,mny=INT_MAX,mxx=INT_MIN,mxy=INT_MIN;
        for (auto& p:net->pin2D){ mnx=min(mnx,p.x);mny=min(mny,p.y);mxx=max(mxx,p.x);mxy=max(mxy,p.y); }
        return (mxx-mnx)+(mxy-mny);
    }

    bool netHasOverflow(ISPDParser::Net* net) {
        for (auto& tp : net->twopin)
            for (auto& rp : tp.path) {
                if (rp.hori && h_dem[hidx(rp.x,rp.y)]>h_cap[hidx(rp.x,rp.y)]) return true;
                if (!rp.hori && v_dem[vidx(rp.x,rp.y)]>v_cap[vidx(rp.x,rp.y)]) return true;
            }
        return false;
    }

    void route() {
        auto& nets = data->nets;
        int n = nets.size();
        printf("Starting global routing: %d nets, grid %dx%d\n", n, X, Y);

        vector<int> order(n);
        iota(order.begin(), order.end(), 0);

        // Sort by HPWL ascending: short nets first (less likely to overflow)
        sort(order.begin(), order.end(), [&](int a, int b){
            return getHPWL(nets[a]) < getHPWL(nets[b]);
        });

        // Initial routing pass
        printf("Pass 0: initial routing...\n");
        for (int i = 0; i < n && !timeUp(); i++)
            routeNet(nets[order[i]], 0.0);
        printf("  TOF=%d (t=%.1fs)\n", totalOverflow(), elapsed());

        double histW=0.1, histInc=0.3;

        for (int iter=0; iter<200 && !timeUp(); iter++) {
            int tof=totalOverflow();
            if (tof==0) break;

            // Collect and sort overflow nets by HPWL ascending
            vector<int> ovfNets;
            for (int i=0;i<n;i++)
                if (netHasOverflow(nets[order[i]])) ovfNets.push_back(i);

            if (ovfNets.empty()) break;

            // Sort overflow nets: route short ones last (they're easier, route hard ones first)
            sort(ovfNets.begin(), ovfNets.end(), [&](int a, int b){
                return getHPWL(nets[order[a]]) > getHPWL(nets[order[b]]);
            });

            updateHistory(histInc);

            for (int idx : ovfNets) {
                if (timeUp()) break;
                auto* net=nets[order[idx]];
                unrouteNet(net);
                routeNet(net, histW);
            }

            int newTof=totalOverflow();
            printf("Iter %d: %d->%d OF nets=%d histW=%.2f (t=%.1fs)\n",
                   iter, tof, newTof, (int)ovfNets.size(), histW, elapsed());

            histW=min(histW*1.2, 15.0);
            histInc=min(histInc*1.05, 2.0);
        }

        printf("Final TOF: %d (t=%.1fs)\n", totalOverflow(), elapsed());
    }
};

int main(int argc, char **argv) {
    assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile>");
    ifstream fp(argv[1]);
    assert(fp.is_open() && "Failed to open input file");
    ISPDParser::ispdData *ispdData = ISPDParser::parse(fp);
    fp.close();

    ispdData->nets.erase(remove_if(ispdData->nets.begin(), ispdData->nets.end(),
        [&](ISPDParser::Net *net) {
            for (auto &pin : net->pins) {
                int x=(get<0>(pin)-ispdData->lowerLeftX)/ispdData->tileWidth;
                int y=(get<1>(pin)-ispdData->lowerLeftY)/ispdData->tileHeight;
                int z=get<2>(pin)-1;
                if (any_of(net->pin3D.begin(),net->pin3D.end(),[x,y,z](const auto &p){
                    return p.x==x&&p.y==y&&p.z==z;})) continue;
                net->pin3D.emplace_back(x,y,z);
                if (any_of(net->pin2D.begin(),net->pin2D.end(),[x,y](const auto &p){
                    return p.x==x&&p.y==y;})) continue;
                net->pin2D.emplace_back(x,y);
            }
            return net->pin3D.size()>1000 || net->pin2D.size()<=1;
        }), ispdData->nets.end());
    ispdData->numNet=ispdData->nets.size();

    GlobalRouter router(ispdData);
    router.route();

    LayerAssignment::Graph graph;
    graph.initialLA(*ispdData, 1);
    graph.convertGRtoLA(*ispdData, true);
    graph.COLA(true);
    graph.output3Dresult(argv[2]);

    delete ispdData;
    return 0;
}
