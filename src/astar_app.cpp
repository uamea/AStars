#include <astar_app.hpp>

using namespace astar_algorithms;

namespace astar_app
{
    AStarApp::AStarApp(
        const std::vector<std::vector<double>> &map,
        const std::pair<int, int> &start,
        const std::pair<int, int> &goal,
        std::unique_ptr<astar_algorithms::IAStar> astar,
        double refreshRate)
        : astar_(std::move(astar)),
          viz_(map, start, goal, [this](std::vector<std::vector<double>> &updatedMap)
               {
        if (!this->astar_)
        {
            std::abort(); 
        }
        // Update the visualization with the new map
        if (this->astar_->getState() == AlgorithmState::RUNNING)
        {
            this->astar_->stepSearch();
            auto closedSet = this->astar_->getClosedSet();
            auto openSet = this->astar_->getOpenSet();
            auto currentNode = this->astar_->getCurrentNode();

            for (size_t i = 0; i < closedSet.size(); ++i)
            {
                for (size_t j = 0; j < closedSet[i].size(); ++j)
                {
                    if (openSet[i][j])
                    {
                        updatedMap[i][j] = static_cast<double>(astar_viz::MapStates::OPEN_SET); // Open set
                    }
                    else if (closedSet[i][j])
                    {
                        updatedMap[i][j] = static_cast<double>(astar_viz::MapStates::CLOSED_SET); // Closed set
                    }
                    else if (i == currentNode.first && j == currentNode.second)
                    {
                        updatedMap[i][j] = static_cast<double>(astar_viz::MapStates::PATH); // Current node
                    }
                }
            }
        }
        else if (this->astar_->getState() == AlgorithmState::GOAL_FOUND)
        {
            auto goal = this->astar_->getPath();

            for (auto pos : goal)
            {
                updatedMap[pos.first][pos.second] = static_cast<double>(astar_viz::MapStates::PATH); // Goal Path
            }
        } }, refreshRate) {}

    void AStarApp::setAstar(std::unique_ptr<astar_algorithms::IAStar> astar)
    {
        astar_ = std::move(astar);
    }

    void AStarApp::run_viz()
    {
        if (!astar_)
        {
            std::abort(); 
        }
        astar_->initializeSearch();
        viz_.run();
    }
}