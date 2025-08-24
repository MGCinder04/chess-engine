#pragma once
#include "position.hpp"
#include <cstdint>
#include <vector>

enum TTFlag : uint8_t
{
    TT_EXACT = 0,
    TT_LOWER = 1,
    TT_UPPER = 2
};

struct TTEntry
{
    uint64_t key{0};
    Move move{};               // best move from this node
    int16_t score{0};          // stored score (with mate distance encoded)
    int8_t depth{(int8_t) -1}; // search depth of this entry
    uint8_t flag{TT_EXACT};
};

class TT
{
    std::vector<TTEntry> table; // single-slot TT (simple & fast)
    uint64_t mask;

  public:
    explicit TT(size_t mb = 32)
    {
        size_t bytes = mb * 1024 * 1024;
        size_t n     = bytes / sizeof(TTEntry);
        // round to next power of two
        size_t pow2 = 1;
        while (pow2 < n)
            pow2 <<= 1;
        table.resize(pow2);
        mask = pow2 - 1;
    }
    inline TTEntry *probe(uint64_t key)
    {
        return &table[key & mask];
    }
    inline void store(uint64_t key, int depth, int score, TTFlag flag, const Move &m)
    {
        TTEntry &e = table[key & mask];
        if (e.key != key || depth >= e.depth)
        {
            e.key   = key;
            e.depth = (int8_t) depth;
            e.score = (int16_t) score;
            e.flag  = (uint8_t) flag;
            e.move  = m;
        }
    }
    inline void clear()
    {
        for (auto &e : table)
            e = TTEntry{};
    }
    size_t size() const
    {
        return table.size();
    }
};
