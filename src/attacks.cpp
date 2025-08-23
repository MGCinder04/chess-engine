#include "attacks.hpp"

// Non-occupancy-dependent tables
U64 KNIGHT_ATK[64], KING_ATK[64], PAWN_ATK[2][64];

// Rays for sliders (internal)
static U64 RAY_N[64], RAY_S[64], RAY_E[64], RAY_W[64];
static U64 RAY_NE[64], RAY_NW[64], RAY_SE[64], RAY_SW[64];

void init_attacks()
{
    // Knights
    int df[8] = {+1, +2, +2, +1, -1, -2, -2, -1};
    int dr[8] = {+2, +1, -1, -2, -2, -1, +1, +2};
    for (int r = 0; r < 8; r++)
        for (int f = 0; f < 8; f++)
        {
            U64 m = 0;
            int s = SQ(f, r);
            for (int i = 0; i < 8; i++)
            {
                int nf = f + df[i], nr = r + dr[i];
                if (on(nf, nr))
                    m |= sqbb(SQ(nf, nr));
            }
            KNIGHT_ATK[s] = m;
        }
    // King
    int kf[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int kr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    for (int r = 0; r < 8; r++)
        for (int f = 0; f < 8; f++)
        {
            U64 m = 0;
            int s = SQ(f, r);
            for (int i = 0; i < 8; i++)
            {
                int nf = f + kf[i], nr = r + kr[i];
                if (on(nf, nr))
                    m |= sqbb(SQ(nf, nr));
            }
            KING_ATK[s] = m;
        }
    // Pawn captures (from-square)
    for (int r = 0; r < 8; r++)
        for (int f = 0; f < 8; f++)
        {
            int s = SQ(f, r);
            U64 w = 0, b = 0;
            if (on(f - 1, r + 1))
                w |= sqbb(SQ(f - 1, r + 1));
            if (on(f + 1, r + 1))
                w |= sqbb(SQ(f + 1, r + 1));
            if (on(f - 1, r - 1))
                b |= sqbb(SQ(f - 1, r - 1));
            if (on(f + 1, r - 1))
                b |= sqbb(SQ(f + 1, r - 1));
            PAWN_ATK[WHITE][s] = w;
            PAWN_ATK[BLACK][s] = b;
        }
    // Rays
    auto build = [&](int dfx, int dfy, U64 *R)
    {
        for (int r = 0; r < 8; r++)
            for (int f = 0; f < 8; f++)
            {
                int s  = SQ(f, r);
                U64 m  = 0;
                int nf = f + dfx, nr = r + dfy;
                while (on(nf, nr))
                {
                    m |= sqbb(SQ(nf, nr));
                    nf += dfx;
                    nr += dfy;
                }
                R[s] = m;
            }
    };
    build(0, +1, RAY_N);
    build(0, -1, RAY_S);
    build(+1, 0, RAY_E);
    build(-1, 0, RAY_W);
    build(+1, +1, RAY_NE);
    build(-1, +1, RAY_NW);
    build(+1, -1, RAY_SE);
    build(-1, -1, RAY_SW);
}

static inline U64 firstBlockNorth(U64 ray, U64 occ)
{
    U64 b = ray & occ;
    if (!b)
        return 0;
    int s = std::countr_zero(b);
    return sqbb(s);
}
static inline U64 firstBlockSouth(U64 ray, U64 occ)
{
    U64 b = ray & occ;
    if (!b)
        return 0;
    int s = 63 - std::countl_zero(b);
    return sqbb(s);
}

U64 rookAtt(int s, U64 occ)
{
    U64 a = 0, b;
    b     = firstBlockNorth(RAY_N[s], occ);
    a |= b ? RAY_N[s] ^ RAY_N[std::countr_zero(b)] : RAY_N[s];
    b = firstBlockSouth(RAY_S[s], occ);
    a |= b ? RAY_S[s] ^ RAY_S[63 - std::countl_zero(b)] : RAY_S[s];
    b = firstBlockNorth(RAY_E[s], occ);
    a |= b ? RAY_E[s] ^ RAY_E[std::countr_zero(b)] : RAY_E[s];
    b = firstBlockSouth(RAY_W[s], occ);
    a |= b ? RAY_W[s] ^ RAY_W[63 - std::countl_zero(b)] : RAY_W[s];
    return a;
}
U64 bishopAtt(int s, U64 occ)
{
    U64 a = 0, b;
    b     = firstBlockNorth(RAY_NE[s], occ);
    a |= b ? RAY_NE[s] ^ RAY_NE[std::countr_zero(b)] : RAY_NE[s];
    b = firstBlockNorth(RAY_NW[s], occ);
    a |= b ? RAY_NW[s] ^ RAY_NW[std::countr_zero(b)] : RAY_NW[s];
    b = firstBlockSouth(RAY_SE[s], occ);
    a |= b ? RAY_SE[s] ^ RAY_SE[63 - std::countl_zero(b)] : RAY_SE[s];
    b = firstBlockSouth(RAY_SW[s], occ);
    a |= b ? RAY_SW[s] ^ RAY_SW[63 - std::countl_zero(b)] : RAY_SW[s];
    return a;
}
U64 queenAtt(int s, U64 occ)
{
    return rookAtt(s, occ) | bishopAtt(s, occ);
}
