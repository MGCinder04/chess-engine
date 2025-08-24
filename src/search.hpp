#pragma once
#include "position.hpp"
#include <vector>

struct SearchResult
{
    Move bestMove{};
    int bestScore{0};
    std::vector<Move> pv;
};

// Iterative deepening to maxDepth
SearchResult search_iterative(Position &P, int maxDepth);
