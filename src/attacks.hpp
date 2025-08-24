#pragma once
#include "chess_types.hpp"

extern U64 KNIGHT_ATK[64];
extern U64 KING_ATK[64];
extern U64 PAWN_ATK[2][64];

void init_attacks();

U64 rookAtt(int s, U64 occ);
U64 bishopAtt(int s, U64 occ);
U64 queenAtt(int s, U64 occ);
