#pragma once
#include "position.hpp"
#include <vector>
extern "C" void tt_clear();

struct SearchResult
{
    Move bestMove{};
    int bestScore{0};
    std::vector<Move> pv;
};

// Iterative deepening to maxDepth
SearchResult search_iterative(Position &P, int maxDepth);

