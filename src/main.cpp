#include <algorithms/weighted_astar.hpp>
#include <algorithms/dijkstra.hpp>
#include <algorithms/bidirectional_astar.hpp>
#include <algorithms/thetastar.hpp>
#include <algorithms/basic_astar.hpp>
#include <astar_app.hpp>
#include <iostream>

using namespace astar_algorithms;

int main(int argc, char *argv[])
{
    // Parse command line arguments to select algorithm
    std::string algorithm_name = "astar"; // default
    if (argc > 1)
    {
        algorithm_name = argv[1];
    }

    std::vector<std::vector<double>> map = {
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0},
        {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0},
        {0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}};

    // Define start and goal positions
    std::pair<int, int> start = {0, 0};
    std::pair<int, int> goal = {13, 14};

    std::unique_ptr<IAStar> astar; 
    // Select algorithm based on input argument
    if (algorithm_name == "dijkstra")
    {
        astar = std::make_unique<Dijkstra>(map, start, goal);
    }
    else if (algorithm_name == "astar")
    {
        astar = std::make_unique<BasicAStar>(map, start, goal);
    }
    else if (algorithm_name == "weighted_astar")
    {
        astar = std::make_unique<WeightedAStar>(map, start, goal, 1.0);
    }
    else if (algorithm_name == "bidirectional_astar")
    {
        astar = std::make_unique<BidirectionalAStar>(map, start, goal);
    }
    else if (algorithm_name == "thetastar")
    {
        astar = std::make_unique<ThetaStar>(map, start, goal);
    }
    else
    {
        std::cerr << "Unknown algorithm: " << algorithm_name << std::endl;
        std::cerr << "Using default: astar" << std::endl;
        astar = std::make_unique<BasicAStar>(map, start, goal);
    }

    // Define A* App
    astar_app::AStarApp app(map, start, goal, std::move(astar), 10);

    app.run_viz();

    return 0;
}