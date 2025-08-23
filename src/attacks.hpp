#pragma once
#include "chess_types.hpp"

// Precomputed attacks
extern U64 KNIGHT_ATK[64];
extern U64 KING_ATK[64];
extern U64 PAWN_ATK[2][64]; // PAWN_ATK[color][from]

// Build attack tables and slider rays
void init_attacks();

// Sliding attacks (occupancy-dependent)
U64 rookAtt(int s, U64 occ);
U64 bishopAtt(int s, U64 occ);
U64 queenAtt(int s, U64 occ);
