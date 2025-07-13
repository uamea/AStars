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
    class BidirectionalAStar : public IAStar
    {
    public:
        BidirectionalAStar() = default;
        BidirectionalAStar(const BidirectionalAStar &) = delete;
        BidirectionalAStar &operator=(const BidirectionalAStar &) = delete;

        // Constructor that initializes the bidirectional A* algorithm
        BidirectionalAStar(
            const std::vector<std::vector<double>> &map,
            const std::pair<int, int> &start,
            const std::pair<int, int> &goal)
            : map_(map), start_(start), goal_(goal), state_(AlgorithmState::NOT_STARTED), meeting_point_({-1, -1})
        {
            // Initialize g_scores for both searches
            g_score_forward_ = std::vector<std::vector<double>>(
                map.size(), std::vector<double>(map[0].size(), std::numeric_limits<double>::infinity()));
            g_score_backward_ = std::vector<std::vector<double>>(
                map.size(), std::vector<double>(map[0].size(), std::numeric_limits<double>::infinity()));

            // Initialize visualization data structures
            closed_set_forward_ = std::vector<std::vector<bool>>(map_.size(),
                                                               std::vector<bool>(map_[0].size(), false));
            closed_set_backward_ = std::vector<std::vector<bool>>(map_.size(),
                                                                std::vector<bool>(map_[0].size(), false));
            came_from_forward_ = std::vector<std::vector<std::pair<int, int>>>(map_.size(),
                                                                              std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            came_from_backward_ = std::vector<std::vector<std::pair<int, int>>>(map_.size(),
                                                                               std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            open_set_viz_forward_ = std::vector<std::vector<bool>>(map_.size(),
                                                                  std::vector<bool>(map_[0].size(), false));
            open_set_viz_backward_ = std::vector<std::vector<bool>>(map_.size(),
                                                                   std::vector<bool>(map_[0].size(), false));
            
            // Combined visualization (union of both searches)
            closed_set_ = std::vector<std::vector<bool>>(map_.size(),
                                                         std::vector<bool>(map_[0].size(), false));
            open_set_viz_ = std::vector<std::vector<bool>>(map_.size(),
                                                           std::vector<bool>(map_[0].size(), false));
        }

        // Initialize both searches
        void initializeSearch() override
        {
            // Clear previous state
            open_set_forward_ = std::priority_queue<std::pair<double, std::pair<int, int>>,
                                                   std::vector<std::pair<double, std::pair<int, int>>>,
                                                   std::greater<>>();
            open_set_backward_ = std::priority_queue<std::pair<double, std::pair<int, int>>,
                                                    std::vector<std::pair<double, std::pair<int, int>>>,
                                                    std::greater<>>();

            // Reset all data structures
            resetDataStructures();

            // Initialize forward search (start -> goal)
            g_score_forward_[start_.first][start_.second] = 0;
            double f_score_forward = heuristic(start_, goal_);
            open_set_forward_.push(std::make_pair(f_score_forward, start_));
            open_set_viz_forward_[start_.first][start_.second] = true;

            // Initialize backward search (goal -> start)
            g_score_backward_[goal_.first][goal_.second] = 0;
            double f_score_backward = heuristic(goal_, start_);
            open_set_backward_.push(std::make_pair(f_score_backward, goal_));
            open_set_viz_backward_[goal_.first][goal_.second] = true;

            state_ = AlgorithmState::RUNNING;
            current_node_ = std::make_pair(-1, -1);
            meeting_point_ = std::make_pair(-1, -1);
            path_.clear();
            
            updateCombinedVisualization();
        }

        // Execute one step of bidirectional A*
        bool stepSearch() override
        {
            if (state_ != AlgorithmState::RUNNING)
            {
                return false;
            }

            // Check if both searches have nodes to explore
            if (open_set_forward_.empty() && open_set_backward_.empty())
            {
                state_ = AlgorithmState::NO_PATH_EXISTS;
                return false;
            }

            // Alternate between forward and backward search
            bool stepped_forward = false;
            bool stepped_backward = false;

            // Step forward search
            if (!open_set_forward_.empty())
            {
                stepped_forward = stepForwardSearch();
            }

            // Step backward search
            if (!open_set_backward_.empty() && state_ == AlgorithmState::RUNNING)
            {
                stepped_backward = stepBackwardSearch();
            }

            updateCombinedVisualization();

            // Check if searches have met
            if (state_ == AlgorithmState::RUNNING)
            {
                checkForMeeting();
            }

            return state_ == AlgorithmState::RUNNING;
        }

        // Run complete bidirectional A* algorithm
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

        // Additional getters for bidirectional visualization
        const std::vector<std::vector<bool>> &getForwardClosedSet() const { return closed_set_forward_; }
        const std::vector<std::vector<bool>> &getBackwardClosedSet() const { return closed_set_backward_; }
        const std::vector<std::vector<bool>> &getForwardOpenSet() const { return open_set_viz_forward_; }
        const std::vector<std::vector<bool>> &getBackwardOpenSet() const { return open_set_viz_backward_; }
        std::pair<int, int> getMeetingPoint() const { return meeting_point_; }

    private:
        std::vector<std::vector<double>> map_;
        std::pair<int, int> start_;
        std::pair<int, int> goal_;
        std::pair<int, int> meeting_point_;

        // G-scores for both searches
        std::vector<std::vector<double>> g_score_forward_;
        std::vector<std::vector<double>> g_score_backward_;

        // State for step-by-step execution
        AlgorithmState state_;
        std::pair<int, int> current_node_;
        std::vector<std::pair<int, int>> path_;

        // Data structures for forward search
        std::priority_queue<std::pair<double, std::pair<int, int>>,
                            std::vector<std::pair<double, std::pair<int, int>>>,
                            std::greater<>>
            open_set_forward_;
        std::vector<std::vector<bool>> closed_set_forward_;
        std::vector<std::vector<std::pair<int, int>>> came_from_forward_;
        std::vector<std::vector<bool>> open_set_viz_forward_;

        // Data structures for backward search
        std::priority_queue<std::pair<double, std::pair<int, int>>,
                            std::vector<std::pair<double, std::pair<int, int>>>,
                            std::greater<>>
            open_set_backward_;
        std::vector<std::vector<bool>> closed_set_backward_;
        std::vector<std::vector<std::pair<int, int>>> came_from_backward_;
        std::vector<std::vector<bool>> open_set_viz_backward_;

        // Combined visualization data (union of both searches)
        std::vector<std::vector<bool>> closed_set_;
        std::vector<std::vector<bool>> open_set_viz_;

        void resetDataStructures()
        {
            // Reset forward search data
            std::fill(closed_set_forward_.begin(), closed_set_forward_.end(),
                      std::vector<bool>(map_[0].size(), false));
            std::fill(came_from_forward_.begin(), came_from_forward_.end(),
                      std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            std::fill(open_set_viz_forward_.begin(), open_set_viz_forward_.end(),
                      std::vector<bool>(map_[0].size(), false));

            // Reset backward search data
            std::fill(closed_set_backward_.begin(), closed_set_backward_.end(),
                      std::vector<bool>(map_[0].size(), false));
            std::fill(came_from_backward_.begin(), came_from_backward_.end(),
                      std::vector<std::pair<int, int>>(map_[0].size(), std::make_pair(-1, -1)));
            std::fill(open_set_viz_backward_.begin(), open_set_viz_backward_.end(),
                      std::vector<bool>(map_[0].size(), false));

            // Reset combined visualization
            std::fill(closed_set_.begin(), closed_set_.end(),
                      std::vector<bool>(map_[0].size(), false));
            std::fill(open_set_viz_.begin(), open_set_viz_.end(),
                      std::vector<bool>(map_[0].size(), false));

            // Reset g_scores
            for (auto &row : g_score_forward_)
            {
                std::fill(row.begin(), row.end(), std::numeric_limits<double>::infinity());
            }
            for (auto &row : g_score_backward_)
            {
                std::fill(row.begin(), row.end(), std::numeric_limits<double>::infinity());
            }
        }

        bool stepForwardSearch()
        {
            if (open_set_forward_.empty()) return false;

            current_node_ = open_set_forward_.top().second;
            open_set_forward_.pop();
            open_set_viz_forward_[current_node_.first][current_node_.second] = false;

            if (closed_set_forward_[current_node_.first][current_node_.second])
            {
                return true;
            }

            closed_set_forward_[current_node_.first][current_node_.second] = true;

            return exploreNeighbors(current_node_, true);
        }

        bool stepBackwardSearch()
        {
            if (open_set_backward_.empty()) return false;

            auto backward_current = open_set_backward_.top().second;
            open_set_backward_.pop();
            open_set_viz_backward_[backward_current.first][backward_current.second] = false;

            if (closed_set_backward_[backward_current.first][backward_current.second])
            {
                return true;
            }

            closed_set_backward_[backward_current.first][backward_current.second] = true;

            return exploreNeighbors(backward_current, false);
        }

        bool exploreNeighbors(const std::pair<int, int>& node, bool is_forward)
        {
            const std::vector<std::pair<int, int>> directions = {
                {-1, 0}, {0, 1}, {1, 0}, {0, -1}};

            auto& g_score = is_forward ? g_score_forward_ : g_score_backward_;
            auto& closed_set = is_forward ? closed_set_forward_ : closed_set_backward_;
            auto& came_from = is_forward ? came_from_forward_ : came_from_backward_;
            auto& open_set = is_forward ? open_set_forward_ : open_set_backward_;
            auto& open_set_viz = is_forward ? open_set_viz_forward_ : open_set_viz_backward_;
            auto& target = is_forward ? goal_ : start_;

            for (const auto &dir : directions)
            {
                int next_x = node.first + dir.first;
                int next_y = node.second + dir.second;
                std::pair<int, int> neighbor = {next_x, next_y};

                // Check bounds and obstacles
                if (next_x < 0 || next_x >= static_cast<int>(map_.size()) ||
                    next_y < 0 || next_y >= static_cast<int>(map_[0].size()) ||
                    map_[next_x][next_y] > 0 || closed_set[next_x][next_y])
                {
                    continue;
                }

                double tentative_g_score = g_score[node.first][node.second] + 1;

                if (tentative_g_score < g_score[next_x][next_y])
                {
                    came_from[next_x][next_y] = node;
                    g_score[next_x][next_y] = tentative_g_score;
                    double f = tentative_g_score + heuristic(neighbor, target);
                    open_set.push(std::make_pair(f, neighbor));
                    open_set_viz[next_x][next_y] = true;
                }
            }

            return true;
        }

        void checkForMeeting()
        {
            // Check if any node has been visited by both searches
            for (int i = 0; i < static_cast<int>(map_.size()); ++i)
            {
                for (int j = 0; j < static_cast<int>(map_[0].size()); ++j)
                {
                    if (closed_set_forward_[i][j] && closed_set_backward_[i][j])
                    {
                        meeting_point_ = {i, j};
                        path_ = reconstructBidirectionalPath(meeting_point_);
                        state_ = AlgorithmState::GOAL_FOUND;
                        return;
                    }
                }
            }
        }

        void updateCombinedVisualization()
        {
            for (int i = 0; i < static_cast<int>(map_.size()); ++i)
            {
                for (int j = 0; j < static_cast<int>(map_[0].size()); ++j)
                {
                    closed_set_[i][j] = closed_set_forward_[i][j] || closed_set_backward_[i][j];
                    open_set_viz_[i][j] = open_set_viz_forward_[i][j] || open_set_viz_backward_[i][j];
                }
            }
        }

        std::vector<std::pair<int, int>> reconstructBidirectionalPath(const std::pair<int, int>& meeting_point)
        {
            std::vector<std::pair<int, int>> path;

            // Reconstruct path from start to meeting point
            std::vector<std::pair<int, int>> forward_path;
            std::pair<int, int> curr = meeting_point;
            while (curr != start_)
            {
                forward_path.push_back(curr);
                curr = came_from_forward_[curr.first][curr.second];
                if (curr.first == -1 && curr.second == -1) break;
            }
            forward_path.push_back(start_);
            std::reverse(forward_path.begin(), forward_path.end());

            // Reconstruct path from meeting point to goal
            std::vector<std::pair<int, int>> backward_path;
            curr = meeting_point;
            while (curr != goal_)
            {
                curr = came_from_backward_[curr.first][curr.second];
                if (curr.first == -1 && curr.second == -1) break;
                backward_path.push_back(curr);
            }

            // Combine paths
            path = forward_path;
            path.insert(path.end(), backward_path.begin(), backward_path.end());

            return path;
        }
    };
}
