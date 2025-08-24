#include "search.hpp"
#include "eval.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

using namespace std;

static constexpr int INF     = 32000;
static constexpr int MATE    = 30000;
static constexpr int MAX_PLY = 64;

// ===== helpers (local, no header edits needed) =====
static inline int pieceAt(const Position &P, int s)
{
    U64 m = sqbb((unsigned) s);
    for (int p = WP; p <= BK; ++p)
        if (P.bb12[p] & m)
            return p;
    return NO_PIECE;
}
static inline void removeP(Position &P, int p, int s)
{
    P.bb12[p] &= ~sqbb((unsigned) s);
}
static inline void place(Position &P, int p, int s)
{
    P.bb12[p] |= sqbb((unsigned) s);
}

// ===== killers & history =====
static array<Move, MAX_PLY> killer1, killer2;
static uint32_t historyTable[12][64] = {};

// ===== PV storage =====
static Move pvTable[MAX_PLY][MAX_PLY];
static int pvLen[MAX_PLY];

// ===== MVV/LVA =====
static int pieceValue12[12] = {100, 320, 330, 500, 900, 10000, 100, 320, 330, 500, 900, 10000};
static inline int mvvLvaScore(int attacker, int victim)
{
    if (victim == NO_PIECE)
        return 0;
    return pieceValue12[victim] * 16 - (attacker != NO_PIECE ? pieceValue12[attacker] : 0);
}

// ===== make / unmake (copied from your perft block semantics) =====
struct UndoLocal
{
    Move m;
    int capPiece{NO_PIECE};
    int capSq{-1};
    Side stm;
    uint8_t prevCastle{0};
    int prevEp{-1};
    int movedPiece{NO_PIECE};
};

static void doMoveLocal(Position &P, const Move &m, UndoLocal &u)
{
    u.stm        = P.stm;
    u.m          = m;
    u.capPiece   = NO_PIECE;
    u.capSq      = -1;
    u.prevCastle = P.castle;
    u.prevEp     = P.epSq;

    int moving   = pieceAt(P, m.from);
    u.movedPiece = moving;
    int cap      = pieceAt(P, m.to);
    bool isPawn  = (moving == WP || moving == BP);
    bool isEP    = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);

    if (isEP)
    {
        int ts     = (P.stm == WHITE ? m.to - 8 : m.to + 8);
        u.capPiece = (P.stm == WHITE ? BP : WP);
        u.capSq    = ts;
        removeP(P, u.capPiece, ts);
    }
    else if (cap != NO_PIECE)
    {
        u.capPiece = cap;
        u.capSq    = m.to;
        removeP(P, cap, m.to);
    }

    removeP(P, moving, m.from);
    int placeAs = m.promo ? m.promo : moving;
    place(P, placeAs, m.to);

    // castling rook move
    if (moving == WK && abs(m.to - m.from) == 2)
    {
        if (m.to == G1)
        {
            removeP(P, WR, H1);
            place(P, WR, F1);
        }
        else if (m.to == C1)
        {
            removeP(P, WR, A1);
            place(P, WR, D1);
        }
    }
    if (moving == BK && abs(m.to - m.from) == 2)
    {
        if (m.to == G8)
        {
            removeP(P, BR, H8);
            place(P, BR, F8);
        }
        else if (m.to == C8)
        {
            removeP(P, BR, A8);
            place(P, BR, D8);
        }
    }

    // castle rights
    auto clearCastleOnRookSq = [&](int sq)
    {
        if (sq == H1)
            P.castle &= ~W_K;
        if (sq == A1)
            P.castle &= ~W_Q;
        if (sq == H8)
            P.castle &= ~B_K;
        if (sq == A8)
            P.castle &= ~B_Q;
    };
    if (moving == WK)
        P.castle &= ~(W_K | W_Q);
    if (moving == BK)
        P.castle &= ~(B_K | B_Q);
    clearCastleOnRookSq(m.from);
    clearCastleOnRookSq(m.to);
    if (u.capPiece != NO_PIECE && u.capSq != -1)
        clearCastleOnRookSq(u.capSq);

    // ep square
    P.epSq = -1;
    if (isPawn && abs((int) m.to - (int) m.from) == 16)
    {
        P.epSq = (P.stm == WHITE ? (m.from + 8) : (m.from - 8));
    }

    // side to move, occ
    P.stm = (P.stm == WHITE ? BLACK : WHITE);
    P.updateOcc();
}

static void undoMoveLocal(Position &P, const UndoLocal &u)
{
    // side and ep/castle
    P.stm    = u.stm;
    P.epSq   = u.prevEp;
    P.castle = u.prevCastle;

    int moved = u.movedPiece;
    // revert rook if it was castling
    if (moved == WK && abs(u.m.to - u.m.from) == 2)
    {
        if (u.m.to == G1)
        {
            removeP(P, WR, F1);
            place(P, WR, H1);
        }
        else if (u.m.to == C1)
        {
            removeP(P, WR, D1);
            place(P, WR, A1);
        }
    }
    if (moved == BK && abs(u.m.to - u.m.from) == 2)
    {
        if (u.m.to == G8)
        {
            removeP(P, BR, F8);
            place(P, BR, H8);
        }
        else if (u.m.to == C8)
        {
            removeP(P, BR, D8);
            place(P, BR, A8);
        }
    }

    // move piece back (un-promote if needed)
    removeP(P, moved, u.m.to);
    int orig = u.m.promo ? (u.stm == WHITE ? WP : BP) : moved;
    place(P, orig, u.m.from);

    // restore capture
    if (u.capPiece != NO_PIECE && u.capSq != -1)
        place(P, u.capPiece, u.capSq);

    P.updateOcc();
}

// ===== move ordering =====
static int scoreMove(const Position &P, const Move &m, int ply)
{
    int moving   = pieceAt(P, m.from);
    int captured = pieceAt(P, m.to);
    if (m.promo)
        captured = (captured == NO_PIECE ? (P.stm == WHITE ? BQ : WQ) : captured);

    if (captured != NO_PIECE)
        return 100000 + mvvLvaScore(moving, captured);

    // killers
    if (killer1[ply].from == m.from && killer1[ply].to == m.to && killer1[ply].promo == m.promo)
        return 90000;
    if (killer2[ply].from == m.from && killer2[ply].to == m.to && killer2[ply].promo == m.promo)
        return 80000;

    // history
    if (moving != NO_PIECE)
        return 1000 + (int) historyTable[moving][m.to];

    return 0;
}
static void orderMoves(Position &P, vector<Move> &mv, int ply)
{
    sort(mv.begin(), mv.end(),
         [&](const Move &a, const Move &b) { return scoreMove(P, a, ply) > scoreMove(P, b, ply); });
}

// ===== negamax =====
static int negamax(Position &P, int depth, int alpha, int beta, int ply)
{
    pvLen[ply] = 0;

    if (depth == 0)
        return (P.stm == WHITE ? +1 : -1) * evaluate(P);

    vector<Move> mv;
    legalMoves(P, mv);
    if (mv.empty())
    {
        bool inCheck = sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
        return inCheck ? -(MATE - ply) : 0;
    }

    orderMoves(P, mv, ply);

    int bestScore = -INF;
    Move bestMove{};
    for (const Move &m : mv)
    {
        UndoLocal u{};
        doMoveLocal(P, m, u);
        int sc = -negamax(P, depth - 1, -beta, -alpha, ply + 1);
        undoMoveLocal(P, u);

        if (sc > bestScore)
        {
            bestScore = sc;
            bestMove  = m;

            // update PV
            pvTable[ply][ply] = m;
            for (int i = 0; i < pvLen[ply + 1]; ++i)
                pvTable[ply][ply + 1 + i] = pvTable[ply + 1][ply + 1 + i];
            pvLen[ply] = 1 + pvLen[ply + 1];
        }

        if (bestScore > alpha)
            alpha = bestScore;
        if (alpha >= beta)
        {
            // killers + history
            if (!m.promo)
            {
                if (!(killer1[ply].from == m.from && killer1[ply].to == m.to && killer1[ply].promo == m.promo))
                {
                    killer2[ply] = killer1[ply];
                    killer1[ply] = m;
                }
                int moving = pieceAt(P, m.from);
                if (moving != NO_PIECE)
                    historyTable[moving][m.to] += depth * depth;
            }
            break;
        }
    }
    return bestScore;
}

// ===== Iterative deepening =====
SearchResult search_iterative(Position &P, int maxDepth)
{
    for (auto &k : killer1)
        k = Move{};
    for (auto &k : killer2)
        k = Move{};
    for (auto &row : historyTable)
        fill(begin(row), end(row), 0u);
    for (int i = 0; i < MAX_PLY; ++i)
        pvLen[i] = 0;

    SearchResult r;
    int alpha = -INF, beta = +INF;

    for (int d = 1; d <= maxDepth; ++d)
    {
        int score = negamax(P, d, alpha, beta, 0);

        vector<Move> line;
        for (int i = 0; i < pvLen[0]; ++i)
            line.push_back(pvTable[0][i]);
        if (!line.empty())
            r.bestMove = line[0];
        r.bestScore = score;
        r.pv        = line;

        // aspiration
        alpha = score - 50;
        beta  = score + 50;
        if (alpha < -INF + 100)
            alpha = -INF;
        if (beta > INF - 100)
            beta = INF;
    }
    return r;
}
