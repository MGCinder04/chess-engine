// rookie: bitboard starter (UCI + perft). No castling/EP yet.
#include <bits/stdc++.h>
#include <bit>

using namespace std;

using U64 = uint64_t;

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

// A1=0 .. H8=63 (LSB = a1)
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

// square → bitboard
static inline U64 sqbb(unsigned s) { return 1ull << s; }

static inline int poplsb(U64 &b)
{
    int s = std::countr_zero(b);
    b &= b - 1;
    return s;
}

U64 KNIGHT_ATK[64], KING_ATK[64], PAWN_ATK[2][64];
U64 RAY_N[64], RAY_S[64], RAY_E[64], RAY_W[64], RAY_NE[64], RAY_NW[64], RAY_SE[64], RAY_SW[64];

static inline bool on(int f, int r) { return 0 <= f && f < 8 && 0 <= r && r < 8; }
static inline int SQ(int f, int r) { return r * 8 + f; }

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
    int kf[8] = {-1, 0, 1, -1, 1, -1, 0, 1}, kr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
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
    // Pawn captures
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
                int s = SQ(f, r);
                U64 m = 0;
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

struct Position
{
    U64 bb12[12]{};
    U64 occ[2]{}, occAll{};
    Side stm = WHITE;
    int kingSq[2]{E1, E8};

    void updateOcc()
    {
        occ[WHITE] = occ[BLACK] = 0ull;
        for (int p = WP; p <= WK; p++)
            occ[WHITE] |= bb12[p];
        for (int p = BP; p <= BK; p++)
            occ[BLACK] |= bb12[p];
        occAll = occ[WHITE] | occ[BLACK];
        kingSq[WHITE] = bb12[WK] ? std::countr_zero(bb12[WK]) : E1;
        kingSq[BLACK] = bb12[BK] ? std::countr_zero(bb12[BK]) : E8;
    }
    void setStart()
    {
        memset(bb12, 0, sizeof(bb12));
        bb12[WP] = 0x000000000000FF00ull;
        bb12[BP] = 0x00FF000000000000ull;
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
        stm = WHITE;
        updateOcc();
    }
};

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
    b = firstBlockNorth(RAY_N[s], occ);
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
    b = firstBlockNorth(RAY_NE[s], occ);
    a |= b ? RAY_NE[s] ^ RAY_NE[std::countr_zero(b)] : RAY_NE[s];
    b = firstBlockNorth(RAY_NW[s], occ);
    a |= b ? RAY_NW[s] ^ RAY_NW[std::countr_zero(b)] : RAY_NW[s];
    b = firstBlockSouth(RAY_SE[s], occ);
    a |= b ? RAY_SE[s] ^ RAY_SE[63 - std::countl_zero(b)] : RAY_SE[s];
    b = firstBlockSouth(RAY_SW[s], occ);
    a |= b ? RAY_SW[s] ^ RAY_SW[63 - std::countl_zero(b)] : RAY_SW[s];
    return a;
}
U64 queenAtt(int s, U64 occ) { return rookAtt(s, occ) | bishopAtt(s, occ); }

bool sqAttacked(const Position &P, int sq, Side by)
{
    U64 occ = P.occAll;
    if (by == WHITE)
    {
        if (PAWN_ATK[WHITE][sq] & P.bb12[WP])
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
        if (PAWN_ATK[BLACK][sq] & P.bb12[BP])
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

struct Move
{
    uint16_t from, to;
    uint8_t promo{0};
};
struct Undo
{
    Move m;
    int capPiece{NO_PIECE};
    Side stm;
};

int pieceAt(const Position &P, int s)
{
    U64 m = sqbb((unsigned)s);
    for (int p = WP; p <= BK; p++)
        if (P.bb12[p] & m)
            return p;
    return NO_PIECE;
}
static inline void place(Position &P, int p, int s) { P.bb12[p] |= sqbb((unsigned)s); }
static inline void removeP(Position &P, int p, int s) { P.bb12[p] &= ~sqbb((unsigned)s); }

template <Side S>
void genPseudo(const Position &P, vector<Move> &out)
{
    constexpr Side T = (S == WHITE ? BLACK : WHITE);
    const U64 own = P.occ[S], opp = P.occ[T], empty = ~P.occAll;

    // Pawns
    U64 PBB = (S == WHITE ? P.bb12[WP] : P.bb12[BP]);
    U64 single = (S == WHITE ? (PBB << 8) : (PBB >> 8)) & empty;
    U64 dbl = (S == WHITE ? ((single & 0x0000000000FF0000ull) << 8)
                          : ((single & 0x0000FF0000000000ull) >> 8)) &
              empty;
    U64 caps = (S == WHITE ? (((PBB << 7) & ~0x0101010101010101ull) | ((PBB << 9) & ~0x8080808080808080ull)) : (((PBB >> 9) & ~0x0101010101010101ull) | ((PBB >> 7) & ~0x8080808080808080ull))) & opp;

    U64 s = single;
    while (s)
    {
        int to = poplsb(s);
        int from = (S == WHITE ? to - 8 : to + 8);
        int rank = to / 8;
        bool promo = (S == WHITE ? rank == 7 : rank == 0);
        if (promo)
        {
            uint8_t pr[4] = {(S == WHITE ? WQ : BQ), (S == WHITE ? WR : BR), (S == WHITE ? WB : BB), (S == WHITE ? WN : BN)};
            for (uint8_t p : pr)
                out.push_back({(uint16_t)from, (uint16_t)to, p});
        }
        else
            out.push_back({(uint16_t)from, (uint16_t)to, 0});
    }
    U64 d = dbl;
    while (d)
    {
        int to = poplsb(d);
        int from = (S == WHITE ? to - 16 : to + 16);
        out.push_back({(uint16_t)from, (uint16_t)to, 0});
    }
    U64 c = caps;
    while (c)
    {
        int to = poplsb(c);
        U64 pawns = PBB & PAWN_ATK[S][to];
        while (pawns)
        {
            int from = poplsb(pawns);
            int rank = to / 8;
            bool promo = (S == WHITE ? rank == 7 : rank == 0);
            if (promo)
            {
                uint8_t pr[4] = {(S == WHITE ? WQ : BQ), (S == WHITE ? WR : BR), (S == WHITE ? WB : BB), (S == WHITE ? WN : BN)};
                for (uint8_t p : pr)
                    out.push_back({(uint16_t)from, (uint16_t)to, p});
            }
            else
                out.push_back({(uint16_t)from, (uint16_t)to, 0});
        }
    }

    auto push = [&](int f, U64 t)
    {
        t &= ~own;
        while (t)
        {
            int to = poplsb(t);
            out.push_back({(uint16_t)f, (uint16_t)to, 0});
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
    tmp = BBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, bishopAtt(f, P.occAll));
    }
    U64 RBB = (S == WHITE ? P.bb12[WR] : P.bb12[BR]);
    tmp = RBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, rookAtt(f, P.occAll));
    }
    U64 QBB = (S == WHITE ? P.bb12[WQ] : P.bb12[BQ]);
    tmp = QBB;
    while (tmp)
    {
        int f = poplsb(tmp);
        push(f, queenAtt(f, P.occAll));
    }
    int f = P.kingSq[S];
    push(f, KING_ATK[f]);
}

void makeMove(Position &P, const Move &m, Undo &u)
{
    u.stm = P.stm;
    u.m = m;
    u.capPiece = NO_PIECE;
    int moving = pieceAt(P, m.from), cap = pieceAt(P, m.to);
    if (cap != NO_PIECE)
    {
        u.capPiece = cap; // <-- remember what we captured
        removeP(P, cap, m.to);
    }

    removeP(P, moving, m.from);
    int placeAs = m.promo ? m.promo : moving;
    place(P, placeAs, m.to);
    P.updateOcc();
    P.stm = (P.stm == WHITE) ? BLACK : WHITE;
}
void unmakeMove(Position &P, const Undo &u)
{
    P.stm = u.stm;
    int moved = pieceAt(P, u.m.to);
    removeP(P, moved, u.m.to);
    int orig = u.m.promo ? (u.stm == WHITE ? WP : BP) : moved;
    place(P, orig, u.m.from);
    if (u.capPiece != NO_PIECE)
        place(P, u.capPiece, u.m.to);
    P.updateOcc();
}

void legalMoves(Position &P, vector<Move> &out)
{
    out.clear();
    vector<Move> ps;
    ps.reserve(256);
    (P.stm == WHITE ? genPseudo<WHITE>(P, ps) : genPseudo<BLACK>(P, ps));
    for (auto &m : ps)
    {
        Undo u;
        makeMove(P, m, u);
        // After makeMove, P.stm is the opponent. Check if the mover's king is attacked by the current side.
        bool inCheck = sqAttacked(P,
                                  P.kingSq[(P.stm == WHITE) ? BLACK : WHITE], // king of the side that just moved
                                  P.stm);                                     // attacked by the side to move now

        if (!inCheck)
            out.push_back(m);
        unmakeMove(P, u);
    }
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
        makeMove(P, m, u);
        n += perft(P, d - 1);
        unmakeMove(P, u);
    }
    return n;
}

// Minimal UCI loop: startpos + perft only
int main()
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    init_attacks();
    Position P;
    P.setStart();
    string line;
    while (getline(cin, line))
    {
        if (line == "uci")
        {
            cout << "id name ChessEngine\nid author You\nuciok\n";
        }

        else if (line == "isready")
        {
            cout << "readyok\n";
        }
        else if (line.rfind("position", 0) == 0)
        {
            if (line.find("startpos") != string::npos)
                P.setStart();
        }
        else if (line.rfind("go perft ", 0) == 0)
        {
            int d = stoi(line.substr(9));
            cout << perft(P, d) << "\n"
                 << flush;
        }
        else if (line == "quit")
        {
            break;
        }
    }
    return 0;
}
