#pragma once
#include "position.hpp"
#include <cstdint>

extern uint64_t Z_PCS[12][64];
extern uint64_t Z_SIDE;
extern uint64_t Z_CASTLE[16]; // castle mask fits 4 bits (W_K,W_Q,B_K,B_Q -> 0..15)
extern uint64_t Z_EP[8];      // ep file a..h

void init_zobrist();

inline uint64_t compute_zobrist(const Position &P)
{
    uint64_t k = 0;
    for (int p = WP; p <= BK; ++p)
    {
        U64 b = P.bb12[p];
        while (b)
        {
            int s = countr_zero(b);
            b &= b - 1;
            k ^= Z_PCS[p][s];
        }
    }
    if (P.stm == BLACK)
        k ^= Z_SIDE;

    // castling: mask is already compatible with 0..15
    k ^= Z_CASTLE[P.castle & 0x0F];

    if (P.epSq != -1)
        k ^= Z_EP[P.epSq % 8];

    return k;
}
