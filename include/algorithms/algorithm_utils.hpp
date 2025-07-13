#pragma once

#include <vector>
#include <utility>
#include <cmath>

namespace astar_algorithms
{
    // heuristic function for A* algorithm
    extern double heuristic(
        const std::pair<int, int> &a,
        const std::pair<int, int> &b); 

    enum class AlgorithmState
    {
        NOT_STARTED,   // Algorithm has not started yet
        RUNNING,       // Algorithm is currently running
        GOAL_FOUND,    // Goal has been found
        NO_PATH_EXISTS // No path exists
    };

}