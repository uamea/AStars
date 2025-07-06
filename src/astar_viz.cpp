#include <astar_viz.hpp>

namespace astar_viz
{

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

        // Set up timer for 0.5 Hz updates (2 seconds interval)
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
                else if (map_[i][j] == 1)
                {
                    ch = static_cast<char>(MapLetters::WALL);
                }
                else if (map_[i][j] == 2)
                {
                    ch = static_cast<char>(MapLetters::PATH);
                }
                else if (map_[i][j] == 3)
                {
                    ch = static_cast<char>(MapLetters::OPEN);
                }
                else if (map_[i][j] == 4)
                {
                    ch = static_cast<char>(MapLetters::CLOSED);
                }
                else // Default to empty space
                {
                    ch = static_cast<char>(MapLetters::EMPTY);
                }
                mvaddch(i, j, ch);
            }
        }
    }

}