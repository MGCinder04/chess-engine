#include "eval.hpp"
#include <bit>
#include <cstdint>

using namespace std;

static inline int pcsqCount(U64 bb)
{
    return popcount(bb);
}

static constexpr int VAL_P = 100;
static constexpr int VAL_N = 320;
static constexpr int VAL_B = 330;
static constexpr int VAL_R = 500;
static constexpr int VAL_Q = 900;

int evaluate(const Position &P)
{
    int w = VAL_P * pcsqCount(P.bb12[WP]) + VAL_N * pcsqCount(P.bb12[WN]) + VAL_B * pcsqCount(P.bb12[WB]) +
            VAL_R * pcsqCount(P.bb12[WR]) + VAL_Q * pcsqCount(P.bb12[WQ]);

    int b = VAL_P * pcsqCount(P.bb12[BP]) + VAL_N * pcsqCount(P.bb12[BN]) + VAL_B * pcsqCount(P.bb12[BB]) +
            VAL_R * pcsqCount(P.bb12[BR]) + VAL_Q * pcsqCount(P.bb12[BQ]);

    return w - b;
}
