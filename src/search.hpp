#pragma once
#include "position.hpp"
#include <vector>
extern "C" void tt_clear();
// --- Search control hooks for UCI ---
extern "C" void tt_resize_mb(int mb);
extern "C" void search_set_stop(int v);  // 0/1
extern "C" void search_set_start_time(); // marks t0 and resets node counter
extern "C" unsigned long long search_get_nodes();

struct SearchResult
{
    Move bestMove{};
    int bestScore{0};
    std::vector<Move> pv;
};

// Iterative deepening to maxDepth
SearchResult search_iterative(Position &P, int maxDepth);

