#pragma once

#include <vector>
#include <string>
#include <utility>
#include <functional>
#include <chrono>
#include <thread>

#include <ncurses.h>

namespace astar_viz
{

    enum class MapLetters
    {
        EMPTY = '.',
        WALL = '#',
        START = 'S',
        GOAL = 'G',
        PATH = '*',
        OPEN = 'O',
        CLOSED = 'X'
    };

    class AStarViz
    {
    public:
        AStarViz() = default;
        AStarViz(const AStarViz &) = delete;
        AStarViz &operator=(const AStarViz &) = delete;

        // Constructor that initializes the A* visualization with a map, start, and goal positions
        AStarViz(
            const std::vector<std::vector<double>> &map,
            const std::pair<int, int> &start,
            const std::pair<int, int> &goal,
            const std::function<void(std::vector<std::vector<double>> &)> &updateCallback,
            double refreshRate = 1.0);

        ~AStarViz() = default;

        // Method to run the A* visualization
        void run();

        // Method to update the visualization
        void update();

    private:
        std::vector<std::vector<double>> map_;                                   // The map to visualize
        std::pair<int, int> start_;                                              // The starting position
        std::pair<int, int> goal_;                                               // The goal position
        std::function<void(std::vector<std::vector<double>> &)> updateCallback_; // Callback function for updates
        double refreshRate_;                                                     // Refresh rate for the visualization
    };
}