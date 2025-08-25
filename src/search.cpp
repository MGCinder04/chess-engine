#include "search.hpp"
#include "eval.hpp"
#include "tt.hpp"
#include "zobrist.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>

using namespace std;

static int negamax(Position &P, int depth, int alpha, int beta, int ply);

static TT g_tt(64); 

static constexpr int INF     = 30000;
static constexpr int MATE    = 32000;

static constexpr int MAX_PLY = 64;

static std::atomic<bool> g_stop{false};
static std::atomic<unsigned long long> g_nodes{0};
static std::chrono::steady_clock::time_point g_t0;
static std::atomic<unsigned long long> g_timeLimitMs{0};

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

static array<Move, MAX_PLY> killer1, killer2;
static uint32_t historyTable[12][64] = {};

static Move pvTable[MAX_PLY][MAX_PLY];
static int pvLen[MAX_PLY];

static int pieceValue12[12] = {100, 320, 330, 500, 900, 10000, 100, 320, 330, 500, 900, 10000};
static inline int mvvLvaScore(int attacker, int victim)
{
    if (victim == NO_PIECE)
        return 0;
    return pieceValue12[victim] * 16 - (attacker != NO_PIECE ? pieceValue12[attacker] : 0);
}

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

// ================================================
// FIXED doMoveLocal / undoMoveLocal
// ================================================

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

    // Handle captures
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

    // Move the piece (handle promotions)
    removeP(P, moving, m.from);
    int placeAs = (m.promo ? m.promo : moving);
    place(P, placeAs, m.to);

    // Castling rook moves
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

    // Update castling rights
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

    // Update en-passant
    P.epSq = -1;
    if (isPawn && abs((int) m.to - (int) m.from) == 16)
        P.epSq = (P.stm == WHITE ? (m.from + 8) : (m.from - 8));

    // Switch turn
    P.stm = (P.stm == WHITE ? BLACK : WHITE);
    P.updateOcc();
}

static void undoMoveLocal(Position &P, const UndoLocal &u)
{
    // Restore turn / ep / castling
    P.stm    = u.stm;
    P.epSq   = u.prevEp;
    P.castle = u.prevCastle;

    int moved = u.movedPiece;

    // Undo castling rook moves
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

    // Remove piece from destination
    removeP(P, pieceAt(P, u.m.to), u.m.to);

    // Restore moving piece (handle promotions)
    int orig;
    if (u.m.promo)
        orig = (u.stm == WHITE ? WP : BP); // restore to pawn
    else
        orig = moved;
    place(P, orig, u.m.from);

    // Restore captured piece if any
    if (u.capPiece != NO_PIECE && u.capSq != -1)
        place(P, u.capPiece, u.capSq);

    P.updateOcc();
}

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

static int quiesce(Position &P, int alpha, int beta)
{
    if (g_stop.load(std::memory_order_relaxed))
        return alpha;
    g_nodes.fetch_add(1, std::memory_order_relaxed);
    if (g_timeLimitMs.load(std::memory_order_relaxed))
    {
        using namespace std::chrono;
        unsigned long long elapsed =
            (unsigned long long) duration_cast<milliseconds>(steady_clock::now() - g_t0).count();
        if (elapsed >= g_timeLimitMs.load(std::memory_order_relaxed))
        {
            g_stop.store(true, std::memory_order_relaxed);
            return alpha;
        }
    }

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
        int captured    = pieceAt(P, m.to);
        int gainBound   = standPat + piece_value[captured] + 100; // small margin
        const int Delta = 900;                                    // queen value margin
        if (gainBound + Delta < alpha)
            continue; // skip hopeless capture

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

static bool inCheck(const Position &P)
{
    return sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
}

static int null_move_prune(Position &P, int depth, int alpha, int beta, int ply)
{
    (void) alpha; 
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


static int negamax(Position &P, int depth, int alpha, int beta, int ply)
{
    if (g_stop.load(std::memory_order_relaxed))
        return 0;
    g_nodes.fetch_add(1, std::memory_order_relaxed);
    if (g_timeLimitMs.load(std::memory_order_relaxed))
    {
        using namespace std::chrono;
        unsigned long long elapsed =
            (unsigned long long) duration_cast<milliseconds>(steady_clock::now() - g_t0).count();
        if (elapsed >= g_timeLimitMs.load(std::memory_order_relaxed))
        {
            g_stop.store(true, std::memory_order_relaxed);
            return 0; 
        }
    }

    int origAlpha = alpha;
    pvLen[ply]    = 0;

    uint64_t key = compute_zobrist(P);

    if (TTEntry *e = g_tt.probe(key); e->key == key)
    {
        int ttScore = from_tt_score(e->score, ply);
        if (e->depth >= depth)
        {
            if (e->flag == TT_EXACT)
            {
                if (ply == 0 && (e->move.from | e->move.to))
                {
                    pvLen[0]      = 1;
                    pvTable[0][0] = e->move;
                }
                return ttScore;
            }
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

        // --- LMR decision (late quiets only) ---
        bool cap      = (pieceAt(P, m.to) != NO_PIECE) || m.promo;
        int reduction = 0;
        if (depth >= 3 && moveIndex >= 3 && !cap)
        {
            // Tentatively plan a 1–2 ply reduction for late quiets
            reduction = 1 + (moveIndex > 8);
        }

        UndoLocal u{};
        doMoveLocal(P, m, u);

        // If the move actually gives check, don't reduce it.
        if (reduction)
        {
            bool givesChk = sqAttacked(P, P.kingSq[P.stm == WHITE ? BLACK : WHITE], (P.stm == WHITE ? BLACK : WHITE));
            if (givesChk)
                reduction = 0;
        }

        int sc;

        // --- Reduced null-window probe for reduced moves ---
        if (reduction)
        {
            sc = -negamax(P, d - reduction, -alpha - 1, -alpha, ply + 1);
            // If promising, re-search at the proper depth/window
            if (sc > alpha)
                sc = -negamax(P, d, -beta, -alpha, ply + 1);
        }
        else
        {
            // PVS: full window for the first move; null-window for later moves
            if (moveIndex == 0)
            {
                sc = -negamax(P, d, -beta, -alpha, ply + 1);
            }
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

            pvTable[ply][0] = m;
            for (int i = 0; i < pvLen[ply + 1]; ++i)
                pvTable[ply][i + 1] = pvTable[ply + 1][i];
            pvLen[ply] = pvLen[ply + 1] + 1;
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
    int prevScore = 0;

    for (int d = 1; d <= maxDepth; ++d)
    {
        if (g_stop.load(std::memory_order_relaxed))
            break;

        int score;
        if (d == 1)
        {
            // First iteration: full window to seed prevScore
            score = negamax(P, d, -INF, +INF, 0);
        }
        else
        {
            int alpha = prevScore - 50;
            int beta  = prevScore + 50;

            // Re-search on fail-low/high, widening the window
            while (true)
            {
                if (g_stop.load(std::memory_order_relaxed))
                    break;
                score = negamax(P, d, alpha, beta, 0);

                if (score <= alpha)
                    alpha -= 200; // widen low
                else if (score >= beta)
                    beta += 200; // widen high
                else
                    break; // inside window
            }
        }

        // Extract PV
        std::vector<Move> line;
        for (int i = 0; i < pvLen[0]; ++i)
            line.push_back(pvTable[0][i]);
        if (!line.empty())
            r.bestMove = line[0];
        r.bestScore = score;
        r.pv        = line;

        // UCI "info" line
        std::ostringstream pvss;
        auto sqTo = [&](int s)
        {
            std::string r;
            r.push_back('a' + (s % 8));
            r.push_back('1' + (s / 8));
            return r;
        };
        for (size_t i = 0; i < r.pv.size(); ++i)
        {
            if (i)
                pvss << ' ';
            const Move &m = r.pv[i];
            std::string s = sqTo(m.from) + sqTo(m.to);
            if (m.promo)
            {
                char pc = 'q';
                if (m.promo == WN || m.promo == BN)
                    pc = 'n';
                else if (m.promo == WB || m.promo == BB)
                    pc = 'b';
                else if (m.promo == WR || m.promo == BR)
                    pc = 'r';
                s.push_back(pc);
            }
            pvss << s;
        }

        using namespace std::chrono;
        unsigned long long ms    = (unsigned long long) duration_cast<milliseconds>(steady_clock::now() - g_t0).count();
        unsigned long long nodes = search_get_nodes();
        unsigned long long nps   = ms ? (nodes * 1000ull / ms) : 0ull;

        cout << "info depth " << d << " score cp " << r.bestScore << " nodes " << nodes << " time " << ms << " nps "
             << nps << " pv " << pvss.str() << "\n"
             << flush;

        prevScore = score; // seed next iteration’s window
    }

    return r;
}

extern "C" void tt_clear()
{
    g_tt.clear();
}

extern "C" void tt_resize_mb(int mb)
{
    if (mb < 1)
        mb = 1;
    if (mb > 4096)
        mb = 4096;
    TT newtt((size_t) mb);
    g_tt = std::move(newtt);
}

extern "C" void search_set_stop(int v)
{
    g_stop.store(v != 0, std::memory_order_relaxed);
}

extern "C" void search_set_time_limit_ms(unsigned long long ms)
{
    g_timeLimitMs.store(ms, std::memory_order_relaxed);
}

extern "C" void search_set_start_time()
{
    g_nodes.store(0, std::memory_order_relaxed);
    g_stop.store(false, std::memory_order_relaxed);
    g_t0 = std::chrono::steady_clock::now();
}

extern "C" unsigned long long search_get_nodes()
{
    return g_nodes.load(std::memory_order_relaxed);
}

extern "C" int search_root_tt_move(Position &P, Move *out)
{
    uint64_t key = compute_zobrist(P);
    TTEntry *e   = g_tt.probe(key);
    if (e->key == key && (e->move.from | e->move.to))
    {
        if (out)
            *out = e->move;
        return 1;
    }
    return 0;
}
