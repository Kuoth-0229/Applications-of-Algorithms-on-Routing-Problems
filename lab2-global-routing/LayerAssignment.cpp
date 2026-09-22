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

#include "LayerAssignment.h"
#include <cmath>
#include <climits>
#include <algorithm>

namespace LayerAssignment {

using std::min;
using std::max;

Graph::Graph()
{
    postLA = false;
    horDemand = verDemand = horCompactOF = verCompactOF = horCeilOF = verCeilOF = horFloorOF = verFloorOF = NULL;
    binMinCost = NULL;
    binBestSol = NULL;
}

Graph::~Graph()
{
    if (cap3D.size() != 0)
    {
        for (int i = 0; i < xNum - 1; i++)
        {
            delete[] horDemand[i];
            delete[] horCompactOF[i];
            delete[] horCeilOF[i];
            delete[] horFloorOF[i];
        }
        delete[] horDemand;
        delete[] horCompactOF;
        delete[] horCeilOF;
        delete[] horFloorOF;

        for (int i = 0; i < xNum; i++)
        {
            delete[] verDemand[i];
            delete[] verCompactOF[i];
            delete[] verCeilOF[i];
            delete[] verFloorOF[i];
        }
        delete[] verDemand;
        delete[] verCompactOF;
        delete[] verCeilOF;
        delete[] verFloorOF;
    }

    if (binMinCost != NULL)
    {
        for (int i = 0; i < zNum; i++)
        {
            for (int j = 0; j < zNum; j++)
            {
                for (int k = 0; k <= int(Amax); k++)
                {
                    delete[] binMinCost[i][j][k];
                    delete[] binBestSol[i][j][k];
                }
                delete[] binMinCost[i][j];
                delete[] binBestSol[i][j];
            }
            delete[] binMinCost[i];
            delete[] binBestSol[i];
        }
        delete[] binMinCost;
        delete[] binBestSol;
    }
}

inline int Graph::get3DDem(int x, int y, int z, int hori) const
{
    return gEdge3D[hori][x][y][z].dem;
}

inline void Graph::set3DDem(int x, int y, int z, int hori, int value)
{
    gEdge3D[hori][x][y][z].dem = value;
}

inline void Graph::incr3DDem(int x, int y, int z, int hori)
{
    gEdge3D[hori][x][y][z].dem++;
    if (gEdge3D[hori][x][y][z].dem > cap3D[hori][z])
    {
        if (hori)
            horCompactOF[x][y]++;
        else
            verCompactOF[x][y]++;
    }
}

inline void Graph::decr3DDem(int x, int y, int z, int hori)
{
    if (gEdge3D[hori][x][y][z].dem > cap3D[hori][z])
    {
        if (hori)
            horCompactOF[x][y]--;
        else
            verCompactOF[x][y]--;
    }
    gEdge3D[hori][x][y][z].dem--;
}

inline int Graph::get3DHis(int x, int y, int z, int hori) const
{
    return gEdge3D[hori][x][y][z].his;
}

inline int Graph::getCost(Edge &e, int lay) const
{
    int cost;
    int dem = get3DDem(e.x, e.y, lay, e.hori);
    int cap = cap3D[e.hori][lay];
    if (postLA)
    {
        cost = (dem >= cap) ? WIRE_OF_COST * ((1 + dem - cap) * wireSize[lay]) : (5 * dem) / cap;
    }
    else
    {
        int his = get3DHis(e.x, e.y, lay, e.hori);
        int overflow;
        double congCost;
        {
            overflow = dem - cap;
            if (overflow > 0)
                overflow = overflow * wireSize[lay];

            congCost = double(1.0 + double(MAX_CONG_COST) / double(1.0 + exp(slope * double(-overflow))));
            cost = int(0.5 + congCost * (1.0 + pow(1 + double(his), 2)));
        }
    }

    return cost + (zNum - lay - 1);
}

int Graph::getTotalOverflow(int &maxOF, double &weightOF)
{
    int overflow;
    maxOF = 0;
    weightOF = 0;
    int totalOverflow = 0;
    for (int i = 0; i < (xNum - 1); i++)
        for (int j = 0; j < yNum; j++)
            for (int k = 0; k < zNum; k++)
            {
                overflow = get3DDem(i, j, k, true) - cap3D[1][k];
                if (overflow > 0)
                {
                    totalOverflow += overflow * wireSize[k];
                    weightOF += pow(overflow * wireSize[k], weight);
                    maxOF = max(maxOF, overflow * wireSize[k]);
                }
            }

    for (int i = 0; i < xNum; i++)
        for (int j = 0; j < (yNum - 1); j++)
            for (int k = 0; k < zNum; k++)
            {
                overflow = get3DDem(i, j, k, false) - cap3D[0][k];
                if (overflow > 0)
                {
                    totalOverflow += overflow * wireSize[k];
                    weightOF += pow(overflow * wireSize[k], weight);
                    maxOF = max(maxOF, overflow * wireSize[k]);
                }
            }

    return totalOverflow;
}

void Graph::ripUp(Net &nn)
{
    for (int i = 0; i < nn.edgeArray.size(); i++)
    {
        Edge &ce = nn.edgeArray[i];
        decr3DDem(ce.x, ce.y, ce.z, ce.hori);
        ce.z = -1;
    }
    totalVia -= nn.numVia;
}

void Graph::BFS_net(vector<vector<int>> &nodeGraph, vector<vector<int>> &horEdgeG, vector<vector<int>> &verEdgeG, vector<vector<int>> &horEdgeLay,
                    vector<vector<int>> &verEdgeLay, vector<vector<int>> &pinGraph, vector<vector<int>> &pinMaxLay, vector<vector<int>> &pinMinLay, Net &nn)
{
    const int dir_x[] = {0, 1, 0, -1};
    const int dir_y[] = {1, 0, -1, 0};

    int pin_size = nn.pinArray.size();
    for (int i = 0; i < pin_size; i++)
    {
        if (pinGraph[nn.pinArray[i].x][nn.pinArray[i].y] == nn.netID)
        {
            pinMaxLay[nn.pinArray[i].x][nn.pinArray[i].y] = max(pinMaxLay[nn.pinArray[i].x][nn.pinArray[i].y], nn.pinArray[i].z);
            pinMinLay[nn.pinArray[i].x][nn.pinArray[i].y] = min(pinMinLay[nn.pinArray[i].x][nn.pinArray[i].y], nn.pinArray[i].z);
        }
        else
        {
            pinMaxLay[nn.pinArray[i].x][nn.pinArray[i].y] = nn.pinArray[i].z;
            pinMinLay[nn.pinArray[i].x][nn.pinArray[i].y] = nn.pinArray[i].z;
        }

        pinGraph[nn.pinArray[i].x][nn.pinArray[i].y] = nn.netID;
    }
    int flag = 0;
    Node pNode;
    Edge pEdge;

    pNode.x = nn.pinArray[0].x;
    pNode.y = nn.pinArray[0].y;
    pNode.degree = 0;
    pNode.parIndex = -1;
    pNode.pin = true;
    pNode.pinMaxLay = pinMaxLay[pNode.x][pNode.y];
    pNode.pinMinLay = pinMinLay[pNode.x][pNode.y];
    nodeGraph[pNode.x][pNode.y] = -1;
    nn.nodeArray.push_back(pNode);
    while (flag < nn.nodeArray.size())
    {
        for (int i = 0; i < 4; ++i)
        {
            Node &cNode = nn.nodeArray[flag];
            pNode.x = cNode.x + dir_x[i];
            pNode.y = cNode.y + dir_y[i];

            if (pNode.x >= 0 && pNode.x < xNum && pNode.y >= 0 && pNode.y < yNum)
                if ((i == 0 && verEdgeG[cNode.x][cNode.y] == nn.netID) || (i == 1 && horEdgeG[cNode.x][cNode.y] == nn.netID) || (i == 2 && verEdgeG[pNode.x][pNode.y] == nn.netID) || (i == 3 && horEdgeG[pNode.x][pNode.y] == nn.netID))
                {
                    if (nodeGraph[pNode.x][pNode.y] == nn.netID)
                    {
                        // new pNode
                        pNode.degree = 0;
                        pNode.parIndex = flag;
                        pNode.pin = (pinGraph[pNode.x][pNode.y] == nn.netID);
                        pNode.pinMaxLay = pinMaxLay[pNode.x][pNode.y];
                        pNode.pinMinLay = pinMinLay[pNode.x][pNode.y];
                        nodeGraph[pNode.x][pNode.y] = -1;

                        // update cNode
                        cNode.chiIndex[cNode.degree] = nn.nodeArray.size();
                        cNode.degree++;

                        // insert
                        pEdge.x = min(pNode.x, cNode.x);
                        pEdge.y = min(pNode.y, cNode.y);
                        pEdge.hori = (i == 1 || i == 3);
                        pEdge.z = pEdge.hori ? horEdgeLay[pEdge.x][pEdge.y] : verEdgeLay[pEdge.x][pEdge.y];
                        nn.edgeArray.push_back(pEdge);
                        if (pEdge.hori)
                            horDemand[pEdge.x][pEdge.y]++;
                        else
                            verDemand[pEdge.x][pEdge.y]++;
                        nn.nodeArray.push_back(pNode);
                    }
                }
        }
        flag++;
    }
}

void Graph::initialGraph()
{
    // 2D overflow
    horDemand = new int *[xNum - 1];
    horCompactOF = new int *[xNum - 1];
    horCeilOF = new int *[xNum - 1];
    horFloorOF = new int *[xNum - 1];
    // 3D
    for (int i = 0; i < xNum - 1; i++)
    {
        horDemand[i] = new int[yNum];
        horCompactOF[i] = new int[yNum];
        horCeilOF[i] = new int[yNum];
        horFloorOF[i] = new int[yNum];

        for (int j = 0; j < yNum; j++)
        {
            horDemand[i][j] = 0;
            horCompactOF[i][j] = 0;
            horCeilOF[i][j] = 0;
            horFloorOF[i][j] = 0;
        }
    }

    // 2D
    verDemand = new int *[xNum];
    verCompactOF = new int *[xNum];
    verCeilOF = new int *[xNum];
    verFloorOF = new int *[xNum];
    // 3D
    for (int i = 0; i < xNum; i++)
    {
        verDemand[i] = new int[yNum - 1];
        verCompactOF[i] = new int[yNum - 1];
        verCeilOF[i] = new int[yNum - 1];
        verFloorOF[i] = new int[yNum - 1];

        for (int j = 0; j < yNum - 1; j++)
        {
            verDemand[i][j] = 0;
            verCompactOF[i][j] = 0;
            verCeilOF[i][j] = 0;
            verFloorOF[i][j] = 0;
        }
    }

    cap3D.resize(2);
    gEdge3D.resize(2);
    for (int dir = 0; dir < 2; dir++)
    {
        cap3D[dir].resize(zNum, 0);
        gEdge3D[dir].resize(xNum);
        for (int i = 0; i < xNum; i++)
        {
            gEdge3D[dir][i].resize(yNum);
            for (int j = 0; j < yNum; j++)
            {
                gEdge3D[dir][i][j].resize(zNum);
                for (int k = 0; k < zNum; k++)
                {
                    gEdge3D[dir][i][j][k].dem = 0;
                    gEdge3D[dir][i][j][k].his = 0;
                }
            }
        }
    }
}

void Graph::initialLA(ISPDParser::ispdData &netdb, int vc)
{
    xNum = netdb.numXGrid;
    yNum = netdb.numYGrid;
    zNum = netdb.numLayer;
    weight = 2.0;
    viaCost = vc;
    initialGraph();
    wireWidth = netdb.minimumWidth;
    wireSpace = netdb.minimumSpacing;
    wireSize.resize(zNum);
    for (int i = 0; i < zNum; i++)
    {
        wireSize[i] = wireWidth[i] + wireSpace[i];
        cap3D[0][i] = netdb.verticalCapacity[i] / wireSize[i];
        cap3D[1][i] = netdb.horizontalCapacity[i] / wireSize[i];
    }

    blx = netdb.lowerLeftX;
    bly = netdb.lowerLeftY;
    cellW = netdb.tileWidth;
    cellH = netdb.tileHeight;

    int net_size = netdb.nets.size();
    int pin_size;
    int index = 0;
    netArray.resize(net_size);
    for (int i = 0; i < net_size; i++)
    {

        ISPDParser::Net &n = *netdb.nets[i];
        if (n.pin2D.size() <= 1)
            continue;

        pin_size = n.pin3D.size();
        netArray[index].netID = n.id;
        netArray[index].name = n.name;
        netArray[index].pinArray.resize(pin_size);
        for (int j = 0; j < pin_size; j++)
        {
            netArray[index].pinArray[j].x = n.pin3D[j].x;
            netArray[index].pinArray[j].y = n.pin3D[j].y;
            netArray[index].pinArray[j].z = n.pin3D[j].z;
        }
        if (pin_size <= 1)
            printf("ERROR: %s %d netID=%d\n", __FILE__, __LINE__, netArray[index].netID);

        index++;
    }
    netArray.resize(index);
    int adj_size = netdb.numCapacityAdj;

    for (int i = 0; i < adj_size; i++)
    {
        ISPDParser::CapacityAdj &pp = *netdb.capacityAdjs[i];
        int adj = pp.reducedCapacityLevel;
        int x = min(std::get<0>(pp.grid1), std::get<0>(pp.grid2));
        int y = min(std::get<1>(pp.grid1), std::get<1>(pp.grid2));
        int z = std::get<2>(pp.grid1) - 1;
        bool isHorizontal = (std::get<0>(pp.grid1) != std::get<0>(pp.grid2));
        int oriCap = isHorizontal ? 
                     netdb.horizontalCapacity[z] : netdb.verticalCapacity[z];
        if (adj == oriCap)
            continue;
        adj /= wireSize[z];

        set3DDem(x, y, z, isHorizontal, cap3D[isHorizontal][z] - adj);
        if (!isHorizontal)
        {
            verDemand[x][y] += get3DDem(x, y, z, false);
        }
        else
        {
            horDemand[x][y] += get3DDem(x, y, z, true);
        }
    }

}

void Graph::convertGRtoLA(ISPDParser::ispdData &netdb, bool print_to_screen)
{
    maxLen = 0;
    vector<vector<int>> nodeGraph, pinGraph, horEdgeG, verEdgeG, horEdgeLay, verEdgeLay, pinMinLay, pinMaxLay;
    nodeGraph.resize(xNum);
    pinGraph.resize(xNum);
    pinMinLay.resize(xNum);
    pinMaxLay.resize(xNum);
    horEdgeG.resize(xNum - 1);
    horEdgeLay.resize(xNum - 1);
    verEdgeG.resize(xNum);
    verEdgeLay.resize(xNum);
    Amax = 100000;

    for (int i = 0; i < xNum; i++)
    {
        nodeGraph[i].resize(yNum, -100);
        pinGraph[i].resize(yNum, -100);
        pinMinLay[i].resize(yNum, -100);
        pinMaxLay[i].resize(yNum, -100);
        verEdgeG[i].resize(yNum - 1, -100);
        verEdgeLay[i].resize(yNum - 1);

        if (i < xNum - 1)
        {
            horEdgeG[i].resize(yNum, -100);
            horEdgeLay[i].resize(yNum);
        }
    }

    origiWL = 0;
    int net_size = netdb.nets.size();
    int index = 0;
    for (int i = 0; i < net_size; i++)
    {
        if (netdb.nets[i]->pin2D.size() <= 1)
            continue;
        int net_id = netArray[index].netID;
        if (netdb.nets[i]->id != net_id)
        {
            printf("ERROR : %s %d\n", __FILE__, __LINE__);
            exit(1);
        }

        for (int j = 0; j < netdb.nets[i]->twopin.size(); j++)
        {
            ISPDParser::TwoPin &two = netdb.nets[i]->twopin[j];
            for (vector<ISPDParser::RPoint>::iterator it = two.path.begin(); it != two.path.end(); ++it)
            {
                if (it->hori == 1)
                {
                    nodeGraph[it->x][it->y] = net_id;
                    nodeGraph[it->x + 1][it->y] = net_id;
                    horEdgeG[it->x][it->y] = net_id;
                    horEdgeLay[it->x][it->y] = it->z;
                }
                else if (it->hori == 0)
                {
                    nodeGraph[it->x][it->y] = net_id;
                    nodeGraph[it->x][it->y + 1] = net_id;
                    verEdgeG[it->x][it->y] = net_id;
                    verEdgeLay[it->x][it->y] = it->z;
                }
            }
        }
        BFS_net(nodeGraph, horEdgeG, verEdgeG, horEdgeLay, verEdgeLay, pinGraph, pinMaxLay, pinMinLay, netArray[index]);
        origiWL += netArray[index].edgeArray.size();
        maxLen = max(maxLen, int(netArray[index].edgeArray.size()));

        index++;
    }

    solArray.resize(maxLen + 1);
    for (int i = 0; i <= maxLen; i++)
        solArray[i].resize(zNum);

    h_cap2D = 0, v_cap2D = 0;
    for (int i = 0; i < zNum; i++)
    {
        h_cap2D += cap3D[1][i];
        v_cap2D += cap3D[0][i];
    }
    totalWireOF = 0;
    int tmpOverflow;
    for (int i = 0; i < xNum - 1; i++)
        for (int j = 0; j < yNum; j++)
        {
            if (horDemand[i][j] > h_cap2D)
            {
                tmpOverflow = horDemand[i][j] - h_cap2D;
                horCeilOF[i][j] = int(ceill(double(tmpOverflow) * 2.0 / double(zNum)));
                horFloorOF[i][j] = int(floorl(double(tmpOverflow) * 2.0 / double(zNum)));
                totalWireOF += tmpOverflow;
            }
        }

    for (int i = 0; i < xNum; i++)
        for (int j = 0; j < yNum - 1; j++)
        {
            if (verDemand[i][j] > v_cap2D)
            {
                tmpOverflow = verDemand[i][j] - v_cap2D;
                verCeilOF[i][j] = int(ceill(double(tmpOverflow) * 2.0 / double(zNum)));
                verFloorOF[i][j] = int(floorl(double(tmpOverflow) * 2.0 / double(zNum)));
                totalWireOF += tmpOverflow;
            }
        }
    totalWireOF = totalWireOF * wireSize[0];
    if (print_to_screen)
        printf("convertGRtoLA  input total overflow=%d  Wlen2D=%d  maxLen=%d  assignNet=%d\n", totalWireOF, origiWL, maxLen, net_size);
}

bool order_cp(const Net *a, const Net *b)
{
    return (a->score > b->score);
}

void Graph::sort_net()
{
    int net_Array_size = netArray.size();
    netAddress.resize(net_Array_size);
    for (int i = 0; i < net_Array_size; i++)
    {
        int edge_size;
        netAddress[i] = &netArray[i];
        Net &n = netArray[i];

        edge_size = n.edgeArray.size();
        n.score = double(n.pinArray.size()) / double(edge_size);
    }
    std::sort(netAddress.begin(), netAddress.end(), order_cp); // high to low
}


void Graph::COLA(bool print_to_screen)
{
    totalVia = 0;
    totalOF = 0;
    double weightOF;
    slope = 0.3;
    int net_Array_size = netArray.size();
    sort_net();
    if (print_to_screen)
        printf("LA assign net by net\n");
    postLA = true;
    for (int i = 0; i < net_Array_size; i++)
    {
        Net &n = *netAddress[i];
        singleNetLA(n);
    }
    totalOF = getTotalOverflow(maxOF, weightOF);
    if (print_to_screen)
        printf("IntialLA  totalOF=%d  maxOF=%d  weightOF=%.0lf  totalVia=%d\n", totalOF, maxOF, weightOF, totalVia);

    for (int refine = 0; refine < PostRound; refine++)
    {
        for (int i = 0; i < net_Array_size; i++)
        {
            Net &n = *netAddress[i];
            ripUp(n);
            singleNetLA(n);
        }
        totalOF = getTotalOverflow(maxOF, weightOF);
        if (print_to_screen)
            printf("PostLA  totalOF=%d  maxOF=%d  weightOF=%.0lf  totalVia=%d\n", totalOF, maxOF, weightOF, totalVia);
    }
    if (print_to_screen)
        printf("totalOF=%d  maxOF=%d  weightOF=%.0lf   Wlen2D=%d  Via=%d  totalWL(Wlen2D+Via*%d)=%d \n", totalOF, maxOF, weightOF, origiWL, totalVia, viaCost, totalVia * viaCost + origiWL);

}

int Graph::getSingleNetlVia(Net &cn)
{
    int totalVia = 0;
    for (int j = 0; j < cn.nodeArray.size(); j++)
    {
        int minLay = INT_MAX;
        int maxLay = -1;
        Node &cc = cn.nodeArray[j];

        if (cc.pin)
        {
            minLay = min(cc.pinMinLay, minLay);
            maxLay = max(cc.pinMaxLay, maxLay);
        }

        if (j != 0)
        {
            minLay = min(cn.edgeArray[j - 1].z, minLay);
            maxLay = max(cn.edgeArray[j - 1].z, maxLay);
        }

        for (int k = 0; k < cc.degree; k++)
        {
            minLay = min(cn.edgeArray[cc.chiIndex[k] - 1].z, minLay);
            maxLay = max(cn.edgeArray[cc.chiIndex[k] - 1].z, maxLay);
        }
        totalVia += (maxLay - minLay);
    }
    return totalVia;
}

bool Graph::singleNetLA(Net &nn)
{
    bool safe = true;
    int node_size = nn.nodeArray.size();
    for (int i = node_size - 1; i >= 0; i--)
    {
        Node &cn = nn.nodeArray[i];
        if (cn.degree == 0) // leaf
        {
            initialLeaf(nn, i, solArray);
        }
        else if (cn.degree == 1) // propagation
        {
            propagate(nn, i, solArray);
        }
        else // merge
        {
            mergeSubTree(nn, i, solArray);
        }
    }

    topDownAssignment(nn, solArray);

    nn.numVia = getSingleNetlVia(nn);
    totalVia += nn.numVia;

    return safe;
}

void Graph::topDownAssignment(Net &nn, vector<vector<SolVia>> &solArray)
{
    nn.nodeArray[0].bestSolVia = solArray[0][nn.nodeArray[0].pinMinLay];

    // top down assignment edge
    int node_size = nn.nodeArray.size();
    for (int i = 0; i < node_size; i++)
    {
        Node &cn = nn.nodeArray[i];
        for (int j = 0; j < cn.degree; j++)
        {
            Node &chiNode = nn.nodeArray[cn.chiIndex[j]];
            chiNode.bestSolVia = solArray[cn.chiIndex[j]][cn.bestSolVia.chiLayer[j]];

            // assign edge to corresponding layer
            Edge &ce = nn.edgeArray[cn.chiIndex[j] - 1];
            ce.z = chiNode.bestSolVia.vL;
            incr3DDem(ce.x, ce.y, ce.z, ce.hori);
        }
    }
}

void Graph::initialLeaf(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray)
{
    Node &cn = nn.nodeArray[nodeIndex];
    for (int i = 0; i < zNum; i++)
    {
        solArray[nodeIndex][i].vL = i;
        solArray[nodeIndex][i].cost = VIA_COST * (max(cn.pinMaxLay, i) - min(cn.pinMinLay, i));
    }
}

void Graph::mergeSubTree(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray)
{

    Node &cn = nn.nodeArray[nodeIndex];
    vector<SolVia> cSolArray, newSolArray;
    int edgeCost;
    int tempCost;

    // initial
    SolVia newSol;
    newSol.maxLay = (cn.pin) ? cn.pinMaxLay : -1;
    newSol.minLay = (cn.pin) ? cn.pinMinLay : INT_MAX;
    newSol.cost = 0;
    cSolArray.push_back(newSol);
    int minLay;
    for (int i = 0; i < cn.degree; i++)
    {
        int minCost;
        Edge &pEdge = nn.edgeArray[nn.nodeArray[nodeIndex].chiIndex[i] - 1];
        vector<SolVia> &chiSolSet = solArray[nn.nodeArray[nodeIndex].chiIndex[i]];

        minCost = INT_MAX;
        for (int k = 0; k < zNum; k++)
            if (cap3D[pEdge.hori][k] != 0)
            {
                edgeCost = getCost(pEdge, k);
                if (edgeCost < minCost)
                {
                    minCost = edgeCost;
                    minLay = k;
                }
            }

        for (int k = 0; k < zNum; k++)
            if (cap3D[pEdge.hori][k] != 0)
            {
                edgeCost = getCost(pEdge, k);
                if (edgeCost > (minCost + abs(k - minLay) * VIA_COST * 2))
                    continue;

                SolVia &subTree = chiSolSet[k];
                for (int j = 0; j < cSolArray.size(); j++)
                {
                    SolVia &cTree = cSolArray[j];

                    // insert no SP edge
                    newSol.maxLay = max(cTree.maxLay, k);
                    newSol.minLay = min(cTree.minLay, k);
                    newSol.cost = cTree.cost + subTree.cost + edgeCost;
                    for (int m = 0; m < i; m++)
                        newSol.chiLayer[m] = cTree.chiLayer[m];

                    newSol.chiLayer[i] = k;
                    newSolArray.push_back(newSol);
                }
            }

        cSolArray = newSolArray;
        newSolArray.clear();
    }

    for (int j = 0; j < zNum; j++)
    {
        solArray[nodeIndex][j].vL = j;
        solArray[nodeIndex][j].cost = INT_MAX;
    }

    for (int i = 0; i < cSolArray.size(); i++)
    {
        SolVia &newSol = cSolArray[i];
        for (int j = 0; j < zNum; j++)
        {
            tempCost = newSol.cost + VIA_COST * (max(j, newSol.maxLay) - min(j, newSol.minLay));
            if (tempCost < solArray[nodeIndex][j].cost)
            {
                solArray[nodeIndex][j] = newSol;
                solArray[nodeIndex][j].cost = tempCost;
                solArray[nodeIndex][j].vL = j;
            }
        }
    }
}

void Graph::propagate(Net &nn, int nodeIndex, vector<vector<SolVia>> &solArray)
{
    vector<SolVia> &chiSolSet = solArray[nn.nodeArray[nodeIndex].chiIndex[0]];
    Edge &pEdge = nn.edgeArray[nn.nodeArray[nodeIndex].chiIndex[0] - 1];
    Node &cn = nn.nodeArray[nodeIndex];
    int edgeCost;
    int tempCost;
    int pinMaxLay, pinMinLay;
    pinMaxLay = (cn.pin) ? cn.pinMaxLay : 0;
    pinMinLay = (cn.pin) ? cn.pinMinLay : INT_MAX;

    for (int i = 0; i < zNum; i++)
    {
        SolVia &newSol = solArray[nodeIndex][i];
        SolVia &fromSol = chiSolSet[i];

        if (cap3D[pEdge.hori][i] != 0)
        {
            edgeCost = getCost(pEdge, i);
        }
        else
            edgeCost = WIRE_CANT_PLACE;

        newSol.chiLayer[0] = i;
        newSol.vL = i;
        newSol.cost = fromSol.cost + edgeCost + VIA_COST * (max(i, pinMaxLay) - min(i, pinMinLay));
    }

    for (int i = 1; i < zNum; i++)
    {
        SolVia &bottomSol = solArray[nodeIndex][i - 1];
        tempCost = bottomSol.cost + (i > pinMaxLay) * VIA_COST;
        if (tempCost < solArray[nodeIndex][i].cost)
        {
            solArray[nodeIndex][i].cost = tempCost;
            solArray[nodeIndex][i].chiLayer[0] = bottomSol.chiLayer[0];
        }
    }

    for (int i = zNum - 2; i >= 0; i--)
    {
        SolVia &upSol = solArray[nodeIndex][i + 1];
        tempCost = upSol.cost + (i < pinMinLay) * VIA_COST;
        if (tempCost < solArray[nodeIndex][i].cost)
        {
            solArray[nodeIndex][i].cost = tempCost;
            solArray[nodeIndex][i].chiLayer[0] = upSol.chiLayer[0];
        }
    }
}

void Graph::solPrunning(vector<vector<Sol>> &set) const
{
    Sol temp;
    for (int i = 0; i < zNum; i++)
    {
        int minCost = INT_MAX;
        for (int j = 0; j < set[i].size(); j++)
        {
            if (set[i][j].cost < minCost)
            {
                minCost = set[i][j].cost;
                temp = set[i][j];
            }
        }
        set[i].clear();
        set[i].push_back(temp);
    }
}

void Graph::output3Dresult(const char *filename)
{
    FILE *fp = fopen(filename, "w");
    int netArray_size = netArray.size();
    int minLay, maxLay;
    vector<vector<int>> tmpInfo;
    tmpInfo.resize(xNum);
    for (int i = 0; i < xNum; i++)
        tmpInfo[i].resize(yNum, -1);
    vector<Line> lines;
    vector<Line> tmpLines;

    for (int i = 0; i < netArray_size; i++)
    {
        Net &cn = netArray[i];
        fprintf(fp, "%s %d\n", cn.name.c_str(), cn.netID);
        lines.clear();
        getStraight(lines, cn.edgeArray, tmpLines, tmpInfo);

        for (int j = 0; j < lines.size(); j++)
        {
            Line &le = lines[j];
            if (le.hori)
            {
                fprintf(fp, "(%d,%d,%d)-(%d,%d,%d)\n", le.x * cellW + blx, le.y * cellH + bly, le.z + 1,
                        (le.x + le.len) * cellW + blx, le.y * cellH + bly, le.z + 1);
            }
            else
            {
                fprintf(fp, "(%d,%d,%d)-(%d,%d,%d)\n", le.x * cellW + blx, le.y * cellH + bly, le.z + 1,
                        le.x * cellW + blx, (le.y + le.len) * cellH + bly, le.z + 1);
            }
        }

        int nodeArray_size = cn.nodeArray.size();
        for (int j = 0; j < nodeArray_size; j++)
        {
            minLay = INT_MAX;
            maxLay = -1;
            Node &cc = cn.nodeArray[j];

            if (cc.pin)
            {
                minLay = min(cc.pinMinLay, minLay);
                maxLay = max(cc.pinMaxLay, maxLay);
            }

            if (j != 0)
            {
                minLay = min(cn.edgeArray[j - 1].z, minLay);
                maxLay = max(cn.edgeArray[j - 1].z, maxLay);
            }

            for (int k = 0; k < cc.degree; k++)
            {
                minLay = min(cn.edgeArray[cc.chiIndex[k] - 1].z, minLay);
                maxLay = max(cn.edgeArray[cc.chiIndex[k] - 1].z, maxLay);
            }
            if (minLay != maxLay)
            {
                fprintf(fp, "(%d,%d,%d)-(%d,%d,%d)\n", cc.x * cellW + blx, cc.y * cellH + bly, minLay + 1,
                        cc.x * cellW + blx, cc.y * cellH + bly, maxLay + 1);
            }
        }
        fprintf(fp, "!\n");
    }
    fclose(fp);
}

inline void Graph::getStraight(vector<Line> &lines, vector<Edge> &edgeArray, vector<Line> &tmpLines, vector<vector<int>> &tmpInfo) const
{

    int size = edgeArray.size();
    tmpLines.resize(size);
    int calLen = 0;
    int horLength = 0, verLength = 0;
    for (int i = 0; i < size; i++)
    {
        Line &le = tmpLines[i];
        le.x = edgeArray[i].x;
        le.y = edgeArray[i].y;
        le.z = edgeArray[i].z;
        le.hori = edgeArray[i].hori;
        le.len = 1;
        if (le.hori == 0)
        {
            tmpInfo[le.x][le.y] = i;
            verLength++;
        }
        else
            horLength++;
    }

    for (int i = 0; i < size; i++)
        if (tmpLines[i].hori == 0)
        {
            Line &le = tmpLines[i];

            if (tmpInfo[le.x][le.y] != i)
                continue;

            if (le.y < yNum - 1 && tmpInfo[le.x][le.y + 1] > -1) // merge ae to le
                if (tmpInfo[le.x][le.y + 1] != i)
                {
                    Line &ae = tmpLines[tmpInfo[le.x][le.y + 1]];
                    if (ae.z == le.z)
                    {
                        tmpInfo[ae.x][ae.y] = tmpInfo[le.x][le.y];
                        tmpInfo[ae.x][ae.y + ae.len - 1] = tmpInfo[le.x][le.y];
                        le.len += ae.len;
                    }
                }

            if (le.y > 0 && tmpInfo[le.x][le.y - 1] > -1) // merge le to ae
            {
                Line &ae = tmpLines[tmpInfo[le.x][le.y - 1]];
                if (ae.z == le.z)
                {
                    tmpInfo[le.x][le.y] = tmpInfo[ae.x][ae.y];
                    tmpInfo[le.x][le.y + le.len - 1] = tmpInfo[ae.x][ae.y];
                    ae.len += le.len;
                }
            }
        }

    for (int i = 0; i < size; i++)
        if (tmpLines[i].hori == 0)
        {
            Line &le = tmpLines[i];
            if (tmpInfo[le.x][le.y] == i)
            {
                lines.push_back(le);
                calLen += le.len;
            }
            tmpInfo[le.x][le.y] = -1;
        }

    if (calLen != verLength)
    {
        printf("ERROR: %s %d  len=%d  verLength=%d\n", __FILE__, __LINE__, calLen, verLength);
        exit(1);
    }

    for (int i = 0; i < size; i++)
        if (tmpLines[i].hori == 1)
            tmpInfo[tmpLines[i].x][tmpLines[i].y] = i;

    for (int i = 0; i < size; i++)
        if (tmpLines[i].hori == 1)
        {
            Line &le = tmpLines[i];

            if (tmpInfo[le.x][le.y] != i)
                continue;

            if (le.x < xNum - 1 && tmpInfo[le.x + 1][le.y] > -1) // merge ae to le
                if (tmpInfo[le.x + 1][le.y] != i)
                {
                    Line &ae = tmpLines[tmpInfo[le.x + 1][le.y]];
                    if (ae.z == le.z)
                    {
                        tmpInfo[ae.x][ae.y] = tmpInfo[le.x][le.y];
                        tmpInfo[ae.x + ae.len - 1][ae.y] = tmpInfo[le.x][le.y];
                        le.len += ae.len;
                    }
                }

            if (le.x > 0 && tmpInfo[le.x - 1][le.y] > -1) // merge le to ae
            {
                Line &ae = tmpLines[tmpInfo[le.x - 1][le.y]];
                if (ae.z == le.z)
                {
                    tmpInfo[le.x][le.y] = tmpInfo[ae.x][ae.y];
                    tmpInfo[le.x + le.len - 1][le.y] = tmpInfo[ae.x][ae.y];
                    ae.len += le.len;
                }
            }
        }

    for (int i = 0; i < size; i++)
        if (tmpLines[i].hori == 1)
        {
            Line &le = tmpLines[i];
            if (tmpInfo[le.x][le.y] == i)
            {
                lines.push_back(le);
                calLen += le.len;
            }
            tmpInfo[le.x][le.y] = -1;
        }
    if (calLen != size)
    {
        printf("ERROR: %s %d  len=%d  size=%d\n", __FILE__, __LINE__, calLen, size);
        exit(1);
    }
}

}
