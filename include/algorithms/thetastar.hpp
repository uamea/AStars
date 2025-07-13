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
    class ThetaStar : public IAStar
    {
    public:
        ThetaStar() = default;
        ThetaStar(const ThetaStar &) = delete;
        ThetaStar &operator=(const ThetaStar &) = delete;

        // Constructor that initializes the Theta* algorithm with a map, start, and goal positions
        ThetaStar(
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

        // Execute one step of the Theta* algorithm
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

                // Theta* specific logic: check line of sight
                updateVertex(current_node_, neighbor);
            }

            return true; // Continue algorithm
        }

        // Run complete Theta* algorithm
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

        // Theta* specific function to update vertex with line-of-sight optimization
        void updateVertex(const std::pair<int, int> &current, const std::pair<int, int> &neighbor)
        {
            std::pair<int, int> parent = came_from_[current.first][current.second];

            // If current node has a parent and there's line of sight from parent to neighbor
            if (parent.first != -1 && parent.second != -1 && lineOfSight(parent, neighbor))
            {
                // Path 2: Direct path from parent to neighbor
                double distance_parent_to_neighbor = euclideanDistance(parent, neighbor);
                double tentative_g_score = g_score_[parent.first][parent.second] + distance_parent_to_neighbor;

                if (tentative_g_score < g_score_[neighbor.first][neighbor.second])
                {
                    came_from_[neighbor.first][neighbor.second] = parent;
                    g_score_[neighbor.first][neighbor.second] = tentative_g_score;
                    double f = tentative_g_score + heuristic(neighbor, goal_);
                    open_set_.push(std::make_pair(f, neighbor));
                    open_set_viz_[neighbor.first][neighbor.second] = true;
                }
            }
            else
            {
                // Path 1: Normal A* path from current to neighbor
                double distance_current_to_neighbor = euclideanDistance(current, neighbor);
                double tentative_g_score = g_score_[current.first][current.second] + distance_current_to_neighbor;

                if (tentative_g_score < g_score_[neighbor.first][neighbor.second])
                {
                    came_from_[neighbor.first][neighbor.second] = current;
                    g_score_[neighbor.first][neighbor.second] = tentative_g_score;
                    double f = tentative_g_score + heuristic(neighbor, goal_);
                    open_set_.push(std::make_pair(f, neighbor));
                    open_set_viz_[neighbor.first][neighbor.second] = true;
                }
            }
        }

        // Check if there's a clear line of sight between two points
        bool lineOfSight(const std::pair<int, int> &from, const std::pair<int, int> &to) const
        {
            int x0 = from.first;
            int y0 = from.second;
            int x1 = to.first;
            int y1 = to.second;

            // Bresenham's line algorithm to check for obstacles
            int dx = std::abs(x1 - x0);
            int dy = std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;

            int x = x0;
            int y = y0;

            while (true)
            {
                // Check if current position is an obstacle
                if (x >= 0 && x < static_cast<int>(map_.size()) &&
                    y >= 0 && y < static_cast<int>(map_[0].size()) &&
                    map_[x][y] > 0)
                {
                    return false; // Obstacle found
                }

                // If we've reached the destination
                if (x == x1 && y == y1)
                {
                    break;
                }

                int e2 = 2 * err;

                if (e2 > -dy)
                {
                    err -= dy;
                    x += sx;
                }

                if (e2 < dx)
                {
                    err += dx;
                    y += sy;
                }
            }

            return true; // No obstacles found
        }

        // Calculate Euclidean distance between two points
        double euclideanDistance(const std::pair<int, int> &a, const std::pair<int, int> &b) const
        {
            double dx = static_cast<double>(a.first - b.first);
            double dy = static_cast<double>(a.second - b.second);
            return std::sqrt(dx * dx + dy * dy);
        }

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
