#include "position.hpp"
#include <algorithm>
#include <iostream>

using namespace std;

static inline int pieceAt(const Position &P, int s)
{
    U64 m = sqbb((unsigned) s);
    for (int p = WP; p <= BK; p++)
        if (P.bb12[p] & m)
            return p;
    return NO_PIECE;
}
static inline void place(Position &P, int p, int s)
{
    P.bb12[p] |= sqbb((unsigned) s);
}
static inline void removeP(Position &P, int p, int s)
{
    P.bb12[p] &= ~sqbb((unsigned) s);
}
static inline bool sqEmpty(const Position &P, int sq)
{
    return (P.occAll & sqbb((unsigned) sq)) == 0;
}

void Position::updateOcc()
{
    occ[WHITE] = occ[BLACK] = 0ull;
    for (int p = WP; p <= WK; p++)
        occ[WHITE] |= bb12[p];
    for (int p = BP; p <= BK; p++)
        occ[BLACK] |= bb12[p];
    occAll        = occ[WHITE] | occ[BLACK];
    kingSq[WHITE] = bb12[WK] ? countr_zero(bb12[WK]) : E1;
    kingSq[BLACK] = bb12[BK] ? countr_zero(bb12[BK]) : E8;
}
void Position::setStart()
{
    memset(bb12, 0, sizeof(bb12));
    bb12[WP] = RANK_2;
    bb12[BP] = RANK_7;
    bb12[WR] = sqbb(A1) | sqbb(H1);
    bb12[BR] = sqbb(A8) | sqbb(H8);
    bb12[WN] = sqbb(B1) | sqbb(G1);
    bb12[BN] = sqbb(B8) | sqbb(G8);
    bb12[WB] = sqbb(C1) | sqbb(F1);
    bb12[BB] = sqbb(C8) | sqbb(F8);
    bb12[WQ] = sqbb(D1);
    bb12[BQ] = sqbb(D8);
    bb12[WK] = sqbb(E1);
    bb12[BK] = sqbb(E8);
    stm      = WHITE;
    castle   = (W_K | W_Q | B_K | B_Q);
    epSq     = -1;
    updateOcc();
}

bool sqAttacked(const Position &P, int sq, Side by)
{
    U64 occ = P.occAll;
    if (by == WHITE)
    {
        if (PAWN_ATK[BLACK][sq] & P.bb12[WP])
            return true; 
        if (KNIGHT_ATK[sq] & P.bb12[WN])
            return true;
        if (KING_ATK[sq] & P.bb12[WK])
            return true;
        if (rookAtt(sq, occ) & (P.bb12[WR] | P.bb12[WQ]))
            return true;
        if (bishopAtt(sq, occ) & (P.bb12[WB] | P.bb12[WQ]))
            return true;
    }
    else
    {
        if (PAWN_ATK[WHITE][sq] & P.bb12[BP])
            return true; 
        if (KNIGHT_ATK[sq] & P.bb12[BN])
            return true;
        if (KING_ATK[sq] & P.bb12[BK])
            return true;
        if (rookAtt(sq, occ) & (P.bb12[BR] | P.bb12[BQ]))
            return true;
        if (bishopAtt(sq, occ) & (P.bb12[BB] | P.bb12[BQ]))
            return true;
    }
    return false;
}

template <Side S> static void genPseudo(const Position &P, vector<Move> &out)
{
    constexpr Side T = (S == WHITE ? BLACK : WHITE);
    const U64 own = P.occ[S], opp = P.occ[T], empty = ~P.occAll;

    U64 PBB = (S == WHITE ? P.bb12[WP] : P.bb12[BP]);

    U64 single = (S == WHITE ? ((PBB << 8) & empty) : ((PBB >> 8) & empty));

    U64 dbl;
    if constexpr (S == WHITE)
    {
        dbl = (((PBB & RANK_2) << 16) & empty & (empty << 8));
    }
    else
    {
        dbl = (((PBB & RANK_7) >> 16) & empty & (empty >> 8));
    }

    U64 caps;
    if constexpr (S == WHITE)
    {
        U64 left  = ((PBB & ~FILE_A) << 7);
        U64 right = ((PBB & ~FILE_H) << 9);
        caps      = (left | right) & opp;
    }
    else
    {
        U64 left  = ((PBB & ~FILE_H) >> 7);
        U64 right = ((PBB & ~FILE_A) >> 9);
        caps      = (left | right) & opp;
    }

    U64 s = single;
    while (s)
    {
        int to     = poplsb(s);
        int from   = (S == WHITE ? to - 8 : to + 8);
        int rank   = to / 8;
        bool promo = (S == WHITE ? rank == 7 : rank == 0);
        if (promo)
        {
            uint8_t pr[4] = {(S == WHITE ? WQ : BQ), (S == WHITE ? WR : BR), (S == WHITE ? WB : BB),
                                  (S == WHITE ? WN : BN)};
            for (uint8_t p : pr)
                out.push_back({(uint16_t) from, (uint16_t) to, p});
        }
        else
        {
            out.push_back({(uint16_t) from, (uint16_t) to, 0});
        }
    }
    U64 d = dbl;
    while (d)
    {
        int to   = poplsb(d);
        int from = (S == WHITE ? to - 16 : to + 16);
        out.push_back({(uint16_t) from, (uint16_t) to, 0});
    }
    U64 c = caps;
    while (c)
    {
        int to    = poplsb(c);
        U64 pawns = PBB & (S == WHITE ? PAWN_ATK[BLACK][to] : PAWN_ATK[WHITE][to]);
        while (pawns)
        {
            int from   = poplsb(pawns);
            int rank   = to / 8;
            bool promo = (S == WHITE ? rank == 7 : rank == 0);
            if (promo)
            {
                uint8_t pr[4] = {(S == WHITE ? WQ : BQ), (S == WHITE ? WR : BR), (S == WHITE ? WB : BB),
                                      (S == WHITE ? WN : BN)};
                for (uint8_t p : pr)
                    out.push_back({(uint16_t) from, (uint16_t) to, p});
            }
            else
            {
                out.push_back({(uint16_t) from, (uint16_t) to, 0});
            }
        }
    }

    if (P.epSq != -1)
    {
        int to          = P.epSq;
        U64 epAttackers = PBB & (S == WHITE ? PAWN_ATK[BLACK][to] : PAWN_ATK[WHITE][to]);
        while (epAttackers)
        {
            int from = poplsb(epAttackers);
            out.push_back({(uint16_t) from, (uint16_t) to, 0});
        }
    }

    auto push = [&](int f, U64 t)
    {
        t &= ~own;
        while (t)
        {
            int to = poplsb(t);
            out.push_back({(uint16_t) f, (uint16_t) to, 0});
        }
    };

    U64 NBB = (S == WHITE ? P.bb12[WN] : P.bb12[BN]);
    U64 tmp = NBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, KNIGHT_ATK[f]);
    }
    U64 BBB = (S == WHITE ? P.bb12[WB] : P.bb12[BB]);
    tmp     = BBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, bishopAtt(f, P.occAll));
    }
    U64 RBB = (S == WHITE ? P.bb12[WR] : P.bb12[BR]);
    tmp     = RBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, rookAtt(f, P.occAll));
    }
    U64 QBB = (S == WHITE ? P.bb12[WQ] : P.bb12[BQ]);
    tmp     = QBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, queenAtt(f, P.occAll));
    }
    int kf = P.kingSq[S];
    push(kf, KING_ATK[kf]);

    if constexpr (S == WHITE)
    {
        if ((P.castle & W_K) && sqEmpty(P, F1) && sqEmpty(P, G1) && !sqAttacked(P, E1, BLACK) &&
            !sqAttacked(P, F1, BLACK) && !sqAttacked(P, G1, BLACK))
            out.push_back({(uint16_t) E1, (uint16_t) G1, 0});
        if ((P.castle & W_Q) && sqEmpty(P, D1) && sqEmpty(P, C1) && sqEmpty(P, B1) && !sqAttacked(P, E1, BLACK) &&
            !sqAttacked(P, D1, BLACK) && !sqAttacked(P, C1, BLACK))
            out.push_back({(uint16_t) E1, (uint16_t) C1, 0});
    }
    else
    {
        if ((P.castle & B_K) && sqEmpty(P, F8) && sqEmpty(P, G8) && !sqAttacked(P, E8, WHITE) &&
            !sqAttacked(P, F8, WHITE) && !sqAttacked(P, G8, WHITE))
            out.push_back({(uint16_t) E8, (uint16_t) G8, 0});
        if ((P.castle & B_Q) && sqEmpty(P, D8) && sqEmpty(P, C8) && sqEmpty(P, B8) && !sqAttacked(P, E8, WHITE) &&
            !sqAttacked(P, D8, WHITE) && !sqAttacked(P, C8, WHITE))
            out.push_back({(uint16_t) E8, (uint16_t) C8, 0});
    }
}

void legalMoves(Position &P, vector<Move> &out)
{
    out.clear();
    vector<Move> ps;
    (P.stm == WHITE ? genPseudo<WHITE>(P, ps) : genPseudo<BLACK>(P, ps));
    for (auto &m : ps)
    {
        Undo u; 
        u.stm        = P.stm;
        u.m          = m;
        u.capPiece   = NO_PIECE;
        u.capSq      = -1;
        u.prevCastle = P.castle;
        u.prevEp     = P.epSq;

        int moving  = pieceAt(P, m.from);
        int cap     = pieceAt(P, m.to);
        bool isPawn = (moving == WP || moving == BP);
        bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);
        if (isEP)
        {
            int takenSq = (P.stm == WHITE ? m.to - 8 : m.to + 8);
            u.capPiece  = (P.stm == WHITE ? BP : WP);
            u.capSq     = takenSq;
            removeP(P, u.capPiece, takenSq);
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
        if (moving == WR || moving == BR)
            clearCastleOnRookSq(m.from);
        if (u.capPiece == WR || u.capPiece == BR)
            clearCastleOnRookSq(u.capSq);
        P.epSq = -1;
        if (isPawn && abs(m.to - m.from) == 16)
            P.epSq = (m.to + m.from) / 2;

        P.updateOcc();
        P.stm = (P.stm == WHITE ? BLACK : WHITE);

        bool inCheck = sqAttacked(P, P.kingSq[(P.stm == WHITE) ? BLACK : WHITE], P.stm);
        if (!inCheck)
            out.push_back(m);

        P.stm     = u.stm;
        P.castle  = u.prevCastle;
        P.epSq    = u.prevEp;
        int moved = pieceAt(P, m.to);
        if ((moved == WK || moved == BK) && abs(m.to - m.from) == 2)
        {
            if (moved == WK)
            {
                if (m.to == G1)
                {
                    removeP(P, WR, F1);
                    place(P, WR, H1);
                }
                else if (m.to == C1)
                {
                    removeP(P, WR, D1);
                    place(P, WR, A1);
                }
            }
            else
            {
                if (m.to == G8)
                {
                    removeP(P, BR, F8);
                    place(P, BR, H8);
                }
                else if (m.to == C8)
                {
                    removeP(P, BR, D8);
                    place(P, BR, A8);
                }
            }
        }
        removeP(P, moved, m.to);
        int orig = m.promo ? (u.stm == WHITE ? WP : BP) : moved;
        place(P, orig, m.from);
        if (u.capPiece != NO_PIECE && u.capSq != -1)
            place(P, u.capPiece, u.capSq);
        P.updateOcc();
    }
}


void doMove(Position &P, const Move &m, Undo &u)
{
    u.stm        = P.stm;
    u.m          = m;
    u.capPiece   = NO_PIECE;
    u.capSq      = -1;
    u.prevCastle = P.castle;
    u.prevEp     = P.epSq;

    int moving  = pieceAt(P, m.from);
    int cap     = pieceAt(P, m.to);
    bool isPawn = (moving == WP || moving == BP);
    bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);

    if (isEP)
    {
        int takenSq = (P.stm == WHITE ? m.to - 8 : m.to + 8);
        u.capPiece  = (P.stm == WHITE ? BP : WP);
        u.capSq     = takenSq;
        removeP(P, u.capPiece, takenSq);
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

    if (moving == WK)
        P.kingSq[WHITE] = m.to;
    if (moving == BK)
        P.kingSq[BLACK] = m.to;

    if (moving == WK && std::abs(m.to - m.from) == 2)
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
    if (moving == BK && std::abs(m.to - m.from) == 2)
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

    if (moving == WR || moving == BR)
        clearCastleOnRookSq(m.from);
    if (u.capPiece == WR || u.capPiece == BR)
        clearCastleOnRookSq(u.capSq);

    P.epSq = -1;
    if (isPawn && std::abs(m.to - m.from) == 16)
        P.epSq = (m.to + m.from) / 2;

    P.updateOcc();
    P.stm = (P.stm == WHITE ? BLACK : WHITE);
}

void undoMove(Position &P, const Undo &u)
{
    const Move &m = u.m;

    P.stm    = u.stm;
    P.castle = u.prevCastle;
    P.epSq   = u.prevEp;

    int movedNow = pieceAt(P, m.to);

    if ((movedNow == WK || movedNow == BK) && std::abs(m.to - m.from) == 2)
    {
        if (movedNow == WK)
        {
            if (m.to == G1)
            {
                removeP(P, WR, F1);
                place(P, WR, H1);
            }
            else if (m.to == C1)
            {
                removeP(P, WR, D1);
                place(P, WR, A1);
            }
        }
        else
        {
            if (m.to == G8)
            {
                removeP(P, BR, F8);
                place(P, BR, H8);
            }
            else if (m.to == C8)
            {
                removeP(P, BR, D8);
                place(P, BR, A8);
            }
        }
    }

    removeP(P, movedNow, m.to);
    int orig = m.promo ? (u.stm == WHITE ? WP : BP) : movedNow;
    place(P, orig, m.from);

    if (orig == WK)
        P.kingSq[WHITE] = m.from;
    if (orig == BK)
        P.kingSq[BLACK] = m.from;

    if (u.capPiece != NO_PIECE && u.capSq != -1)
        place(P, u.capPiece, u.capSq);

    P.updateOcc();
}

uint64_t perft(Position &P, int d)
{
    if (d == 0)
        return 1;
    vector<Move> mv;
    legalMoves(P, mv);
    uint64_t n = 0;
    for (auto &m : mv)
    {
        Undo u;
        u.stm        = P.stm;
        u.m          = m;
        u.capPiece   = NO_PIECE;
        u.capSq      = -1;
        u.prevCastle = P.castle;
        u.prevEp     = P.epSq;
        int moving = pieceAt(P, m.from), cap = pieceAt(P, m.to);
        bool isPawn = (moving == WP || moving == BP);
        bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);
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
        auto cl = [&](int sq)
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
        if (moving == WR || moving == BR)
            cl(m.from);
        if (u.capPiece == WR || u.capPiece == BR)
            cl(u.capSq);
        int prevEp = P.epSq;
        P.epSq     = -1;
        if (isPawn && abs(m.to - m.from) == 16)
            P.epSq = (m.to + m.from) / 2;
        P.updateOcc();
        P.stm = (P.stm == WHITE ? BLACK : WHITE);

        n += perft(P, d - 1);

        P.stm     = u.stm;
        P.castle  = u.prevCastle;
        P.epSq    = u.prevEp;
        int moved = pieceAt(P, m.to);
        if ((moved == WK || moved == BK) && abs(m.to - m.from) == 2)
        {
            if (moved == WK)
            {
                if (m.to == G1)
                {
                    removeP(P, WR, F1);
                    place(P, WR, H1);
                }
                else if (m.to == C1)
                {
                    removeP(P, WR, D1);
                    place(P, WR, A1);
                }
            }
            else
            {
                if (m.to == G8)
                {
                    removeP(P, BR, F8);
                    place(P, BR, H8);
                }
                else if (m.to == C8)
                {
                    removeP(P, BR, D8);
                    place(P, BR, A8);
                }
            }
        }
        removeP(P, moved, m.to);
        int orig = m.promo ? (u.stm == WHITE ? WP : BP) : moved;
        place(P, orig, m.from);
        if (u.capPiece != NO_PIECE && u.capSq != -1)
            place(P, u.capPiece, u.capSq);
        P.updateOcc();
        (void) prevEp;
    }
    return n;
}

static inline string sqToStr(int s)
{
    string t;
    t += char('a' + (s % 8));
    t += char('1' + (s / 8));
    return t;
}

void perftSplit(Position &P, int depth)
{
    vector<Move> root;
    legalMoves(P, root);
    uint64_t total = 0;
    for (const auto &m : root)
    {
        Undo u;
        u.stm        = P.stm;
        u.m          = m;
        u.capPiece   = NO_PIECE;
        u.capSq      = -1;
        u.prevCastle = P.castle;
        u.prevEp     = P.epSq;
        int moving = pieceAt(P, m.from), cap = pieceAt(P, m.to);
        bool isPawn = (moving == WP || moving == BP);
        bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);
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
        auto cl = [&](int sq)
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
        if (moving == WR || moving == BR)
            cl(m.from);
        if (u.capPiece == WR || u.capPiece == BR)
            cl(u.capSq);
        
        P.epSq = -1;
        if (isPawn && abs(m.to - m.from) == 16)
            P.epSq = (m.to + m.from) / 2;
        P.updateOcc();
        P.stm = (P.stm == WHITE ? BLACK : WHITE);

        uint64_t n = perft(P, depth - 1);

        P.stm     = u.stm;
        P.castle  = u.prevCastle;
        P.epSq    = u.prevEp;
        int moved = pieceAt(P, m.to);
        if ((moved == WK || moved == BK) && abs(m.to - m.from) == 2)
        {
            if (moved == WK)
            {
                if (m.to == G1)
                {
                    removeP(P, WR, F1);
                    place(P, WR, H1);
                }
                else if (m.to == C1)
                {
                    removeP(P, WR, D1);
                    place(P, WR, A1);
                }
            }
            else
            {
                if (m.to == G8)
                {
                    removeP(P, BR, F8);
                    place(P, BR, H8);
                }
                else if (m.to == C8)
                {
                    removeP(P, BR, D8);
                    place(P, BR, A8);
                }
            }
        }
        removeP(P, moved, m.to);
        int orig = m.promo ? (u.stm == WHITE ? WP : BP) : moved;
        place(P, orig, m.from);
        if (u.capPiece != NO_PIECE && u.capSq != -1)
            place(P, u.capPiece, u.capSq);
        P.updateOcc();

        string ms = sqToStr(m.from) + sqToStr(m.to);
        if (m.promo)
        {
            char pc = 'q';
            if (m.promo == WN || m.promo == BN)
                pc = 'n';
            else if (m.promo == WB || m.promo == BB)
                pc = 'b';
            else if (m.promo == WR || m.promo == BR)
                pc = 'r';
            ms.push_back(pc);
        }
        cout << ms << ": " << n << "\n" << flush;
        total += n;
    }
    cout << "Total: " << total << "\n" << flush;
}
