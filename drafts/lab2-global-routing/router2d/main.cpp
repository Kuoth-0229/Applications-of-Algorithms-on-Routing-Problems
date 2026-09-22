#include "ispdData.h"
#include "LayerAssignment.h"
#include "Router2D.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile> [timeLimit]");
    int timeLimit = (argc >= 4) ? std::atoi(argv[3]) : 1800;

    std::ifstream fp(argv[1]);
    assert(fp.is_open() && "Failed to open input file");
    ISPDParser::ispdData* ispdData = ISPDParser::parse(fp);
    fp.close();

    // Convert pin XY -> grid; populate pin2D / pin3D; drop big and trivial nets
    ispdData->nets.erase(
        std::remove_if(ispdData->nets.begin(), ispdData->nets.end(),
            [&](ISPDParser::Net* net) {
                for (auto& pin : net->pins) {
                    int x = (std::get<0>(pin) - ispdData->lowerLeftX) /
                            ispdData->tileWidth;
                    int y = (std::get<1>(pin) - ispdData->lowerLeftY) /
                            ispdData->tileHeight;
                    int z = std::get<2>(pin) - 1;
                    if (std::any_of(net->pin3D.begin(), net->pin3D.end(),
                            [x, y, z](const auto& p) {
                                return p.x == x && p.y == y && p.z == z;
                            })) continue;
                    net->pin3D.emplace_back(x, y, z);
                    if (std::any_of(net->pin2D.begin(), net->pin2D.end(),
                            [x, y](const auto& p) {
                                return p.x == x && p.y == y;
                            })) continue;
                    net->pin2D.emplace_back(x, y);
                }
                return net->pin3D.size() > 1000 || net->pin2D.size() <= 1;
            }),
        ispdData->nets.end());
    ispdData->numNet = ispdData->nets.size();

    Router::Router2D router(ispdData, timeLimit);
    router.run();

    LayerAssignment::Graph graph;
    graph.initialLA(*ispdData, 1);
    graph.convertGRtoLA(*ispdData, true);
    graph.COLA(true);
    graph.output3Dresult(argv[2]);

    delete ispdData;
    return 0;
}
