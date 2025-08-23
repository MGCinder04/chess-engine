#pragma once
#include <bit>
#include <cstdint>
#include <cstring>

using U64 = std::uint64_t;

enum Side
{
    WHITE = 0,
    BLACK = 1
};

enum Piece
{
    WP,
    WN,
    WB,
    WR,
    WQ,
    WK,
    BP,
    BN,
    BB,
    BR,
    BQ,
    BK,
    NO_PIECE
};

// Square indexing: A1=0 .. H8=63 (LSB=a1)
enum Sq
{
    A1,
    B1,
    C1,
    D1,
    E1,
    F1,
    G1,
    H1,
    A2,
    B2,
    C2,
    D2,
    E2,
    F2,
    G2,
    H2,
    A3,
    B3,
    C3,
    D3,
    E3,
    F3,
    G3,
    H3,
    A4,
    B4,
    C4,
    D4,
    E4,
    F4,
    G4,
    H4,
    A5,
    B5,
    C5,
    D5,
    E5,
    F5,
    G5,
    H5,
    A6,
    B6,
    C6,
    D6,
    E6,
    F6,
    G6,
    H6,
    A7,
    B7,
    C7,
    D7,
    E7,
    F7,
    G7,
    H7,
    A8,
    B8,
    C8,
    D8,
    E8,
    F8,
    G8,
    H8
};

// Castling rights
constexpr std::uint8_t W_K = 1; // White O-O
constexpr std::uint8_t W_Q = 2; // White O-O-O
constexpr std::uint8_t B_K = 4; // Black O-O
constexpr std::uint8_t B_Q = 8; // Black O-O-O

// Files / Ranks
constexpr U64 FILE_A = 0x0101010101010101ull;
constexpr U64 FILE_H = 0x8080808080808080ull;
constexpr U64 RANK_2 = 0x000000000000FF00ull;
constexpr U64 RANK_7 = 0x00FF000000000000ull;

// Helpers
static inline U64 sqbb(unsigned s)
{
    return 1ull << s;
}
static inline int poplsb(U64 &b)
{
    int s = std::countr_zero(b);
    b &= b - 1;
    return s;
}
static inline bool on(int f, int r)
{
    return 0 <= f && f < 8 && 0 <= r && r < 8;
}
static inline int SQ(int f, int r)
{
    return r * 8 + f;
}
