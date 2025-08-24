#include "search.hpp"
#include "eval.hpp"
#include "tt.hpp"
#include "zobrist.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

using namespace std;

// ---- forward decl for functions that call negamax ----
static int negamax(Position &P, int depth, int alpha, int beta, int ply);

// ---- TT instance ----
static TT g_tt(64); // 64 MB

// ---- constants ----
static constexpr int INF     = 32000;
static constexpr int MATE    = 30000;
static constexpr int MAX_PLY = 64;

// ---- mate-score encode/decode for TT ----
static inline int to_tt_score(int sc, int ply)
{
    if (sc > MATE - 1000)
        return sc + ply;
    if (sc < -MATE + 1000)
        return sc - ply;
    return sc;
}
static inline int from_tt_score(int sc, int ply)
{
    if (sc > MATE - 1000)
        return sc - ply;
    if (sc < -MATE + 1000)
        return sc + ply;
    return sc;
}

// ===== helpers =====
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

// ===== make / unmake (local) =====
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

    // ep
    P.epSq = -1;
    if (isPawn && abs((int) m.to - (int) m.from) == 16)
        P.epSq = (P.stm == WHITE ? (m.from + 8) : (m.from - 8));

    P.stm = (P.stm == WHITE ? BLACK : WHITE);
    P.updateOcc();
}

static void undoMoveLocal(Position &P, const UndoLocal &u)
{
    P.stm    = u.stm;
    P.epSq   = u.prevEp;
    P.castle = u.prevCastle;

    int moved = u.movedPiece;
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

    removeP(P, moved, u.m.to);
    int orig = u.m.promo ? (u.stm == WHITE ? WP : BP) : moved;
    place(P, orig, u.m.from);

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

    if (killer1[ply].from == m.from && killer1[ply].to == m.to && killer1[ply].promo == m.promo)
        return 90000;
    if (killer2[ply].from == m.from && killer2[ply].to == m.to && killer2[ply].promo == m.promo)
        return 80000;

    if (moving != NO_PIECE)
        return 1000 + (int) historyTable[moving][m.to];
    return 0;
}

// --- Quiescence search ---
static int quiesce(Position &P, int alpha, int beta)
{
    int standPat = (P.stm == WHITE ? +1 : -1) * evaluate(P);
    if (standPat >= beta)
        return beta;
    if (standPat > alpha)
        alpha = standPat;

    vector<Move> moves;
    legalMoves(P, moves);

    moves.erase(remove_if(moves.begin(), moves.end(),
                          [&](const Move &m) { return pieceAt(P, m.to) == NO_PIECE && m.promo == 0; }),
                moves.end());

    sort(moves.begin(), moves.end(),
         [&](const Move &a, const Move &b)
         {
             return mvvLvaScore(pieceAt(P, a.from), pieceAt(P, a.to)) >
                    mvvLvaScore(pieceAt(P, b.from), pieceAt(P, b.to));
         });

    for (const Move &m : moves)
    {
        UndoLocal u{};
        doMoveLocal(P, m, u);
        int score = -quiesce(P, -beta, -alpha);
        undoMoveLocal(P, u);

        if (score >= beta)
            return beta;
        if (score > alpha)
            alpha = score;
    }
    return alpha;
}

// ===== helpers for pruning/reductions =====
static bool inCheck(const Position &P)
{
    return sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
}

static int null_move_prune(Position &P, int depth, int alpha, int beta, int ply)
{
    (void) alpha; // suppress unused warning
    if (depth < 3 || inCheck(P))
        return -INF;

    int prevEp = P.epSq;
    P.stm      = (P.stm == WHITE ? BLACK : WHITE);
    P.epSq     = -1;
    P.updateOcc();

    int R     = 2 + (depth > 6);
    int score = -negamax(P, depth - 1 - R, -beta, -beta + 1, ply + 1);

    P.stm  = (P.stm == WHITE ? BLACK : WHITE);
    P.epSq = prevEp;
    P.updateOcc();

    return score;
}

static inline bool isCapture(const Position &P, const Move &m)
{
    if (m.promo)
        return true;
    U64 msk = sqbb((unsigned) m.to);
    for (int p = WP; p <= BK; ++p)
        if (P.bb12[p] & msk)
            return true;
    return false;
}
static bool givesCheck(Position &P, const Move &m)
{
    UndoLocal u{};
    doMoveLocal(P, m, u);
    bool chk = sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
    undoMoveLocal(P, u);
    return chk;
}

// ===== negamax =====
static int negamax(Position &P, int depth, int alpha, int beta, int ply)
{
    int origAlpha = alpha;
    pvLen[ply]    = 0;

    uint64_t key = compute_zobrist(P);

    // TT probe
    if (TTEntry *e = g_tt.probe(key); e->key == key)
    {
        int ttScore = from_tt_score(e->score, ply);
        if (e->depth >= depth)
        {
            if (e->flag == TT_EXACT)
                return ttScore;
            else if (e->flag == TT_LOWER && ttScore > alpha)
                alpha = ttScore;
            else if (e->flag == TT_UPPER && ttScore < beta)
                beta = ttScore;
            if (alpha >= beta)
                return ttScore;
        }
    }

    if (depth == 0)
        return quiesce(P, alpha, beta);

    // Null-move try
    int nm = null_move_prune(P, depth, alpha, beta, ply);
    if (nm >= beta)
    {
        g_tt.store(key, depth, to_tt_score(nm, ply), TT_LOWER, Move{});
        return nm;
    }

    vector<Move> mv;
    legalMoves(P, mv);

    Move ttMove{};
    if (TTEntry *e = g_tt.probe(key); e->key == key)
        ttMove = e->move;

    auto moveScore = [&](const Move &m, int plyIdx)
    {
        int base = scoreMove(P, m, plyIdx);
        if (ttMove.from == m.from && ttMove.to == m.to && ttMove.promo == m.promo)
            base += 200000;
        return base;
    };
    sort(mv.begin(), mv.end(), [&](const Move &a, const Move &b) { return moveScore(a, ply) > moveScore(b, ply); });

    if (mv.empty())
    {
        bool check = sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
        return check ? -(MATE - ply) : 0;
    }

    int bestScore = -INF;
    Move bestMove{};
    int moveIndex = 0;

    for (const Move &m : mv)
    {
        int d = depth - 1;

        // LMR for later, quiet, non-check moves
        bool cap      = isCapture(P, m);
        bool chk      = false;
        int reduction = 0;
        if (depth >= 3 && moveIndex >= 3 && !cap)
        {
            chk = givesCheck(P, m);
            if (!chk)
                reduction = 1 + (moveIndex > 8);
        }

        UndoLocal u{};
        doMoveLocal(P, m, u);

        int sc = -negamax(P, d - reduction, -alpha, -alpha - 1, ply + 1); // null-window first
        if (reduction && sc > alpha)
        {
            sc = -negamax(P, d, -beta, -alpha, ply + 1); // re-search
        }
        else if (!reduction)
        {
            if (moveIndex == 0)
                sc = -negamax(P, d, -beta, -alpha, ply + 1); // full for first
            else
            {
                sc = -negamax(P, d, -alpha - 1, -alpha, ply + 1);
                if (sc > alpha && sc < beta)
                    sc = -negamax(P, d, -beta, -alpha, ply + 1);
            }
        }

        undoMoveLocal(P, u);

        if (sc > bestScore)
        {
            bestScore = sc;
            bestMove  = m;

            pvTable[ply][ply] = m;
            for (int i = 0; i < pvLen[ply + 1]; ++i)
                pvTable[ply][ply + 1 + i] = pvTable[ply + 1][ply + 1 + i];
            pvLen[ply] = 1 + pvLen[ply + 1];
        }

        if (bestScore > alpha)
            alpha = bestScore;
        if (alpha >= beta)
        {
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
        ++moveIndex;
    }

    TTFlag flag = TT_EXACT;
    if (bestScore <= origAlpha)
        flag = TT_UPPER;
    else if (bestScore >= beta)
        flag = TT_LOWER;
    g_tt.store(key, depth, to_tt_score(bestScore, ply), flag, bestMove);
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

// allow UCI to clear TT on new game
extern "C" void tt_clear()
{
    g_tt.clear();
}
