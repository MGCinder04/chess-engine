#pragma once
#include "position.hpp"
#include <vector>

extern "C" void tt_clear();
extern "C" void tt_resize_mb(int mb);
extern "C" void search_set_stop(int v);  
extern "C" void search_set_start_time(); 
extern "C" unsigned long long search_get_nodes();
extern "C" void search_set_time_limit_ms(unsigned long long ms); 
extern "C" int search_root_tt_move(Position &P, Move *out);      

struct SearchResult
{
    Move bestMove{};
    int bestScore{0};
    std::vector<Move> pv;
};

SearchResult search_iterative(Position &P, int maxDepth);

