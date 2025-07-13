#pragma once

#include <memory>
#include <astar_viz.hpp>
#include <algorithms/algorithm_base.hpp>

namespace astar_app
{
    class AStarApp
    {
    public:
        AStarApp() = default;
        AStarApp(const AStarApp &) = delete;
        AStarApp &operator=(const AStarApp &) = delete;

        // Constructor that initializes the A* application with a map, start, and goal positions
        AStarApp(
            const std::vector<std::vector<double>> &map,
            const std::pair<int, int> &start,
            const std::pair<int, int> &goal,
            std::unique_ptr<astar_algorithms::IAStar> astar = nullptr,
            double refreshRate = 1.0);

        ~AStarApp() = default;

        void setAstar(std::unique_ptr<astar_algorithms::IAStar> astar);
        void run_viz();

    private: 
        std::unique_ptr<astar_algorithms::IAStar> astar_; 
        astar_viz::AStarViz viz_;
    };
}