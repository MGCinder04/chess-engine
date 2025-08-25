#include "zobrist.hpp"
#include <random>

uint64_t Z_PCS[12][64];
uint64_t Z_SIDE;
uint64_t Z_CASTLE[16];
uint64_t Z_EP[8];

void init_zobrist()
{
    std::mt19937_64 rng(0x9E3779B97F4A7C15ULL);
    auto r = [&] { return rng(); };

    for (int p = 0; p < 12; ++p)
        for (int s = 0; s < 64; ++s)
            Z_PCS[p][s] = r();

    Z_SIDE = r();
    for (int i = 0; i < 16; ++i)
        Z_CASTLE[i] = r();
    for (int f = 0; f < 8; ++f)
        Z_EP[f] = r();
}
