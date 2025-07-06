#include <astar_viz.hpp>
#include <algorithms/basic_astar.hpp>

using namespace astar_algorithms;

int main()
{
    // Create a 15x15 map
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
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    // Define start and goal positions
    std::pair<int, int> start = {0, 0};
    std::pair<int, int> goal = {14, 14};

    BasicAStar astar(map, start, goal);
    astar.initializeSearch();

    astar_viz::AStarViz viz(map, start, goal, [&astar](std::vector<std::vector<double>> &updatedMap)
                            {
        // Update the visualization with the new map
        if (astar.getState() == AlgorithmState::RUNNING) {
            astar.stepSearch(); 
            auto closedSet = astar.getClosedSet();
            auto openSet = astar.getOpenSet();
            auto currentNode = astar.getCurrentNode();

            // synthesize openSet(3) and closedSet(4) into the map
            for (size_t i = 0; i < closedSet.size(); ++i) {
                for (size_t j = 0; j < closedSet[i].size(); ++j) {
                    if (openSet[i][j]) {
                        updatedMap[i][j] = 3; // Open set
                    } else if (closedSet[i][j]) {
                        updatedMap[i][j] = 4; // Closed set
                    } else if (i == currentNode.first && j == currentNode.second) {
                        updatedMap[i][j] = 2; // Current node
                    }
                }
            }
        }else if (astar.getState() == AlgorithmState::GOAL_FOUND) {
            auto goal = astar.getPath(); 

            for (auto pos: goal) {
                updatedMap[pos.first][pos.second] = 2; // Goal;
            }
        } }, 5.0);

    // Run the visualization
    viz.run();

    return 0;
}