#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <string>

#ifndef ROUTER_HPP
#define ROUTER_HPP 1
class Router {
private:
    // You can set your data structure here, like:
    /*
    std::vector<std::vector<int>> nets_coor;
    std::vector<std::vector<int>> nets_layer;
    std::vector<std::string> nets_name;
    std::vector<std::vector<int>> nets_gridline_coor;
    std::vector<int> boundary = std::vector<int>(4);
    std::vector<int> x_coors, y_coors;
    int max_layer;
    int min_pitch_size;
    int via_cost;
    */
public:
    Router() = default;
    void readCircuitSpec(std::ifstream& inputFile);
    void generateGraph();
    std::string getSysCommand(int);
    void generateClauses(std::ofstream& outputFile);
    bool readSolverResult(std::ifstream& inputFile, int);
    void printRoutingResult(std::ofstream& outputFile);
};
#endif