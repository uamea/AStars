#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include <algorithms/algorithm_base.hpp>
#include <algorithms/algorithm_utils.hpp>

namespace astar_algorithms
{
    class BasicAStar : public IAStar
    {
    public:
        BasicAStar() = default;
        BasicAStar(const BasicAStar &) = delete;
        BasicAStar &operator=(const BasicAStar &) = delete;

        // Constructor that initializes the A* algorithm with a map, start, and goal positions
        BasicAStar(
            const std::vector<std::vector<double>> &map,
            const std::pair<int, int> &start,
            const std::pair<int, int> &goal)
            : map_(map), start_(start), goal_(goal), state_(AlgorithmState::NOT_STARTED)
        {
            // Initialize g_score with infinity
            g_score_ = std::vector<std::vector<double>>(
                map.size(), std::vector<double>(map[0].size(), std::numeric_limits<double>::infinity()));

            // Initialize visualization data structures
            closed_set_ = std::vector<std::vector<bool>>(map_.size(),
                                                         std::vector<bool>(map_[0].size(), false));
            came_from_ = std::vector<std::vector<std::pair<int, int>>>(map_.size(),
                                                                       std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            open_set_viz_ = std::vector<std::vector<bool>>(map_.size(),
                                                           std::vector<bool>(map_[0].size(), false));
        }

        // Initialize the algorithm for step-by-step execution
        void initializeSearch() override
        {
            // Clear previous state
            open_set_ = std::priority_queue<std::pair<double, std::pair<int, int>>,
                                            std::vector<std::pair<double, std::pair<int, int>>>,
                                            std::greater<>>();

            // Reset all data structures
            std::fill(closed_set_.begin(), closed_set_.end(),
                      std::vector<bool>(map_[0].size(), false));
            std::fill(came_from_.begin(), came_from_.end(),
                      std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            std::fill(open_set_viz_.begin(), open_set_viz_.end(),
                      std::vector<bool>(map_[0].size(), false));

            // Reset g_score
            for (auto &row : g_score_)
            {
                std::fill(row.begin(), row.end(), std::numeric_limits<double>::infinity());
            }

            // Initialize start node
            g_score_[start_.first][start_.second] = 0;
            double f_score = heuristic(start_, goal_);
            open_set_.push(std::make_pair(f_score, start_));
            open_set_viz_[start_.first][start_.second] = true;

            state_ = AlgorithmState::RUNNING;
            current_node_ = std::make_pair(-1, -1);
            path_.clear();
        }

        // Execute one step of the A* algorithm
        bool stepSearch() override
        {
            if (state_ != AlgorithmState::RUNNING || open_set_.empty())
            {
                if (open_set_.empty() && state_ == AlgorithmState::RUNNING)
                {
                    state_ = AlgorithmState::NO_PATH_EXISTS;
                }
                return false; // Algorithm finished
            }

            // Get node with lowest f_score
            current_node_ = open_set_.top().second;
            open_set_.pop();
            open_set_viz_[current_node_.first][current_node_.second] = false;

            // If goal reached
            if (current_node_ == goal_)
            {
                path_ = reconstructPath(came_from_, goal_);
                state_ = AlgorithmState::GOAL_FOUND;
                return false; // Algorithm finished
            }

            // Skip if already processed
            if (closed_set_[current_node_.first][current_node_.second])
            {
                return true; // Continue to next step
            }

            // Mark as visited
            closed_set_[current_node_.first][current_node_.second] = true;

            // Check all neighbors
            const std::vector<std::pair<int, int>> directions = {
                {-1, 0}, {0, 1}, {1, 0}, {0, -1}};

            for (const auto &dir : directions)
            {
                int next_x = current_node_.first + dir.first;
                int next_y = current_node_.second + dir.second;
                std::pair<int, int> neighbor = {next_x, next_y};

                // Check if valid position
                if (next_x < 0 || next_x >= static_cast<int>(map_.size()) ||
                    next_y < 0 || next_y >= static_cast<int>(map_[0].size()))
                {
                    continue;
                }

                // Skip obstacles
                if (map_[next_x][next_y] > 0)
                {
                    continue;
                }

                // Skip if already evaluated
                if (closed_set_[next_x][next_y])
                {
                    continue;
                }

                // Distance between current and neighbor is 1
                double tentative_g_score = g_score_[current_node_.first][current_node_.second] + 1;

                // If this path is better than previous one
                if (tentative_g_score < g_score_[next_x][next_y])
                {
                    came_from_[next_x][next_y] = current_node_;
                    g_score_[next_x][next_y] = tentative_g_score;
                    double f = tentative_g_score + heuristic(neighbor, goal_);
                    open_set_.push(std::make_pair(f, neighbor));
                    open_set_viz_[next_x][next_y] = true;
                }
            }

            return true; // Continue algorithm
        }

        // Run complete A* algorithm
        std::vector<std::pair<int, int>> findPath() override
        {
            initializeSearch();
            while (stepSearch())
            {
                // Continue until algorithm finishes
            }
            return path_;
        }

        // Getters for visualization
        AlgorithmState getState() const override { return state_; }
        std::pair<int, int> getCurrentNode() const override { return current_node_; }
        const std::vector<std::vector<bool>> &getClosedSet() const override { return closed_set_; }
        const std::vector<std::vector<bool>> &getOpenSet() const override { return open_set_viz_; }
        const std::vector<std::pair<int, int>> &getPath() const override { return path_; }
        bool isInOpenSet(int x, int y) const override { return open_set_viz_[x][y]; }
        bool isInClosedSet(int x, int y) const override { return closed_set_[x][y]; }

    private:
        std::vector<std::vector<double>> map_;
        std::pair<int, int> start_;
        std::pair<int, int> goal_;
        std::vector<std::vector<double>> g_score_;

        // State for step-by-step execution
        AlgorithmState state_;
        std::pair<int, int> current_node_;
        std::vector<std::pair<int, int>> path_;

        // Data structures for visualization
        std::priority_queue<std::pair<double, std::pair<int, int>>,
                            std::vector<std::pair<double, std::pair<int, int>>>,
                            std::greater<>>
            open_set_;
        std::vector<std::vector<bool>> closed_set_;
        std::vector<std::vector<std::pair<int, int>>> came_from_;
        std::vector<std::vector<bool>> open_set_viz_; // For visualization of open set

        // Helper function to reconstruct the path from start to goal
        std::vector<std::pair<int, int>> reconstructPath(
            const std::vector<std::vector<std::pair<int, int>>> &came_from,
            const std::pair<int, int> &current)
        {
            std::vector<std::pair<int, int>> path;
            std::pair<int, int> curr = current;

            while (curr != start_)
            {
                path.push_back(curr);
                curr = came_from[curr.first][curr.second];

                // Safety check to avoid infinite loop
                if (curr.first == -1 && curr.second == -1)
                {
                    return {};
                }
            }

            path.push_back(start_);
            std::reverse(path.begin(), path.end());
            return path;
        }
    };
}