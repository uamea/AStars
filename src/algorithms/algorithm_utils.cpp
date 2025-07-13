#include <algorithms/algorithm_utils.hpp>

namespace astar_algorithms
{
    // Heuristic function for A* algorithm
    double heuristic(
        const std::pair<int, int> &a,
        const std::pair<int, int> &b)
    {
        // L1 norm (Manhattan distance)
        return std::abs(a.first - b.first) + std::abs(a.second - b.second);
    }
}