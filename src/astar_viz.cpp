#include <astar_viz.hpp>

namespace astar_viz
{

    MapLetters mapStateToLetter(MapStates state)
    {
        switch (state)
        {
        case MapStates::EMPTY:
            return MapLetters::EMPTY;
        case MapStates::WALL:
            return MapLetters::WALL;
        case MapStates::START:
            return MapLetters::START;
        case MapStates::GOAL:
            return MapLetters::GOAL;
        case MapStates::PATH:
            return MapLetters::PATH;
        case MapStates::OPEN_SET:
            return MapLetters::OPEN;
        case MapStates::CLOSED_SET:
            return MapLetters::CLOSED;
        default:
            return MapLetters::EMPTY;
        }
    }

    AStarViz::AStarViz(
        const std::vector<std::vector<double>> &map,
        const std::pair<int, int> &start,
        const std::pair<int, int> &goal,
        const std::function<void(std::vector<std::vector<double>> &)> &updateCallback,
        double refreshRate) : map_(map), start_(start), goal_(goal), updateCallback_(updateCallback), refreshRate_(refreshRate)
    {
    }

    void AStarViz::run()
    {
        // Initialize ncurses
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);

        // Set up interval timer
        const int UPDATE_INTERVAL_MS = static_cast<int>(1 / refreshRate_ * 1000); // Convert seconds to milliseconds
        timeout(UPDATE_INTERVAL_MS);                                              // Set getch() timeout

        // Initial update
        update();
        refresh();

        // Main loop
        bool running = true;
        while (running)
        {
            // Wait for user input or timeout
            int ch = getch();

            // Check if user pressed any key
            if (ch != ERR)
            {
                running = false; // Exit the loop on any key press
                continue;
            }

            // Call the update callback and redraw the screen
            if (updateCallback_)
            {
                updateCallback_(map_);
            }
            update();
            refresh();
        }

        // Reset terminal timeout behavior
        timeout(-1); // Restore blocking mode before exiting

        // Clean up and exit ncurses
        endwin();
    }

    void AStarViz::update()
    {
        // Draw the map
        for (size_t i = 0; i < map_.size(); ++i)
        {
            for (size_t j = 0; j < map_[i].size(); ++j)
            {
                char ch;
                if (i == start_.first && j == start_.second)
                {
                    ch = static_cast<char>(MapLetters::START);
                }
                else if (i == goal_.first && j == goal_.second)
                {
                    ch = static_cast<char>(MapLetters::GOAL);
                }
                else
                {
                    ch = static_cast<char>(mapStateToLetter(static_cast<MapStates>(map_[i][j])));
                }
                mvaddch(i, j, ch);
            }
        }
    }
}