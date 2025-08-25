#pragma once
#include "position.hpp"

int evaluate(const Position &P);

// Basic piece values for eval and delta pruning
static const int piece_value[12] = {
    100, 320, 330, 500, 900, 20000, // white: P N B R Q K
    100, 320, 330, 500, 900, 20000  // black: p n b r q k
};
