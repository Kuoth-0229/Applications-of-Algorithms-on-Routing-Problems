// *********************************************************************************************************************
// *********************************************************************************************************************
// Copyright (c) 2013 by Wen-Hao Liu and Yih-Lang Li ("Authors") 
// http://cs.nctu.edu.tw/~whliu/NCTU-GR.htm ("URL")
// All right reserved
// 
// Redistribution, with or without modification, are permitted provided that the following conditions are met:
// 1. Redistributions must reproduce the above copyright notice, this list of conditions and the following 
//    disclaimer in the documentation and/or other materials provided with the distribution.
// 2. Neither the names nor any trademark of the Authors may be used to endorse or promote products derived from 
//    this software without specific prior written permission.
// 3. Use is limited to academic research groups only. Users who are interested in industry or commercial purposes
//    must notify Authors and request separate license agreement.
// 
// THIS FREE SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
// NO EVENT SHALL THE AUTHOR OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, EFFECTS OF UNAUTHORIZED OR MALICIOUS
// NETWORK ACCESS; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// *********************************************************************************************************************
// *********************************************************************************************************************

#pragma once

#include "ispdData.h"

#include <vector>
#include <string>
#include <set>

#define PostRound 3
#define VIA_COST 100
#define WIRE_OF_COST 1000000
#define WIRE_CANT_PLACE 100000000
#define MAX_CONG_COST 20

namespace LayerAssignment {

using std::vector;
using std::string;
using std::set;

struct GridEdge3D;
struct Net;
struct Edge;
struct Sol;
struct SolVia;
struct Node;
struct Pin;
struct Line;

struct Graph {

    // overflow info
    int totalOF;
    int maxOF;
    int totalVia;
    int origiWL;

    int xNum, yNum, zNum;
    int numNet;
    double slope;
    double Amax;
    bool postLA;
    int maxLen;
    int viaCost;
    double weight;
    // layer info
    vector<int> wireSpace, wireWidth, wireSize;

    // 2D
    int v_cap2D, h_cap2D;
    int **horDemand, **verDemand;
    int **horCompactOF, **verCompactOF;
    int **horCeilOF, **verCeilOF;
    int **horFloorOF, **verFloorOF;
    int blx, bly, cellW, cellH;
    int totalWireOF;

    // 3D
    vector<vector<int>> cap3D;
    vector<vector<vector<vector<GridEdge3D>>>> gEdge3D;

    // DP-based sol pruning
    int ****binMinCost;
    Sol ****binBestSol;

    Graph();
    ~Graph();

    vector<Net> netArray;
    vector<Net *> netAddress;
    vector<vector<SolVia>> solArray;

    // IO
    void output3Dresult(const char *);
    void initialGraph();
    void BFS_net(vector<vector<int>> &nodeGraph, vector<vector<int>> &horEdgeG, vector<vector<int>> &verEdgeG, vector<vector<int>> &horEdgeLay,
                 vector<vector<int>> &verEdgeLay, vector<vector<int>> &pinGraph, vector<vector<int>> &pinMaxLay, vector<vector<int>> &pinMinLay, Net &nn);

    // LA
    // Implementation of the algorithm proposed in the following paper:
    // T. -H. Lee and T. -C. Wang, "Congestion-Constrained Layer Assignment for Via Minimization in Global Routing," 
    // in IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 
    // vol. 27, no. 9, pp. 1643-1656, Sept. 2008, doi: 10.1109/TCAD.2008.927733.
    void COLA(bool print_to_screen);
    bool singleNetLA(Net &nn);
    void initialLeaf(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray);
    void propagate(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray);
    void mergeSubTree(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray);
    void solPrunning(vector<vector<Sol>> &) const;
    void topDownAssignment(Net &nn, vector<vector<SolVia>> &solArray);

    // integrate 3D route
    void initialLA(ISPDParser::ispdData &netdb, int vc);
    void convertGRtoLA(ISPDParser::ispdData &netdb, bool print_to_screen);

    // tool
    inline int get3DDem(int x, int y, int z, int hori) const;
    inline void set3DDem(int x, int y, int z, int hori, int value);
    inline void incr3DDem(int x, int y, int z, int hori);
    inline void decr3DDem(int x, int y, int z, int hori);
    inline int get3DHis(int x, int y, int z, int hori) const;

    inline int getCost(Edge &e, int lay) const;
    inline void getStraight(vector<Line> &lines, vector<Edge> &edgeArray, vector<Line> &tmpLines, vector<vector<int>> &tmpInfo) const;
    int getTotalOverflow(int &maxOF, double &wieghtOF);
    int getSingleNetlVia(Net &cn);
    void ripUp(Net &nn);
    void sort_net();
};

struct Net {
    int netID;
    string name;
    int numVia;
    double score;
    vector<Node> nodeArray;
    vector<Edge> edgeArray;
    vector<Pin> pinArray;
    Net(int id) {
        netID = id;
    }
    Net() = default;
};

struct Sol {
    int vL;
    int maxLay;
    int minLay;
    int cost;
    int chiLayer[4];
};

struct SolVia {
    int vL;
    int maxLay;
    int minLay;
    int cost;
    int chiLayer[4];
};

struct Node {
    int x, y;
    int degree;
    int chiIndex[4];
    int parIndex;

    bool pin;
    int pinMaxLay;
    int pinMinLay;

    SolVia bestSolVia;
};

struct Edge {
    int x, y, z;
    bool hori;
};

struct Pin {
    int x, y, z;
    Pin(int ax, int ay, int az)
    {
        x = ax;
        y = ay;
        z = az;
    };
    Pin() = default;
};

struct Line {
    int x, y, z;
    int hori;
    int len;
};

struct GridEdge3D
{
    int dem;
    int his;
};

}
