#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include <algorithms/algorithm_utils.hpp>

namespace astar_algorithms
{

    // Interface for A* algorithms
    class IAStar
    {
    public:
        IAStar() = default;
        IAStar(const IAStar &) = delete;
        IAStar &operator=(const IAStar &) = delete;

        virtual ~IAStar() = default;

        virtual void initializeSearch() = 0;
        virtual bool stepSearch() = 0;
        virtual std::vector<std::pair<int, int>> findPath() = 0;

        // Getters for visualization
        virtual AlgorithmState getState() const = 0;
        virtual std::pair<int, int> getCurrentNode() const = 0;
        virtual const std::vector<std::vector<bool>> &getClosedSet() const = 0;
        virtual const std::vector<std::vector<bool>> &getOpenSet() const = 0;
        virtual const std::vector<std::pair<int, int>> &getPath() const = 0;
        virtual bool isInOpenSet(int x, int y) const = 0;
        virtual bool isInClosedSet(int x, int y) const = 0;
    };

};