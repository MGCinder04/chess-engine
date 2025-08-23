#include "fen.hpp"
#include "attacks.hpp" // for PAWN_ATK if needed (not here, but keep symmetric)
#include <cctype>
#include <sstream>
#include <vector>

// ---- helpers local to this file ---------------------------------

static int sqFromStr(const std::string &s)
{
    if (s.size() != 2)
        return -1;
    char f = s[0], r = s[1];
    if (f < 'a' || f > 'h')
        return -1;
    if (r < '1' || r > '8')
        return -1;
    int file = f - 'a';
    int rank = r - '1';
    return SQ(file, rank); // rank*8 + file  (A1=0..H8=63)
}

static int pieceFromChar(char c)
{
    switch (c)
    {
    case 'P':
        return WP;
    case 'N':
        return WN;
    case 'B':
        return WB;
    case 'R':
        return WR;
    case 'Q':
        return WQ;
    case 'K':
        return WK;
    case 'p':
        return BP;
    case 'n':
        return BN;
    case 'b':
        return BB;
    case 'r':
        return BR;
    case 'q':
        return BQ;
    case 'k':
        return BK;
    default:
        return NO_PIECE;
    }
}

// We need these small helpers from position.cpp; duplicate here as static:
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

// ---- FEN loader --------------------------------------------------

bool setFromFEN(Position &P, const std::string &fen)
{
    std::istringstream iss(fen);
    std::string board, active, castling, ep, hm, fm;
    if (!(iss >> board >> active >> castling >> ep))
        return false; // need 4 fields
    iss >> hm >> fm;  // optional, ignored

    // pieces
    std::memset(P.bb12, 0, sizeof(P.bb12));
    int r = 7, f = 0;
    for (size_t i = 0; i < board.size(); ++i)
    {
        char c = board[i];
        if (c == '/')
        {
            if (f != 8)
                return false;
            --r;
            f = 0;
            if (r < 0)
                return false;
            continue;
        }
        if (std::isdigit((unsigned char) c))
        {
            int cnt = c - '0';
            if (cnt < 1 || cnt > 8)
                return false;
            f += cnt;
            if (f > 8)
                return false;
            continue;
        }
        int p = pieceFromChar(c);
        if (p == NO_PIECE || f >= 8 || r < 0)
            return false;
        int sq = SQ(f, r);
        P.bb12[p] |= sqbb((unsigned) sq);
        ++f;
    }
    if (r != 0 || f != 8)
        return false;

    // side to move
    if (active == "w")
        P.stm = WHITE;
    else if (active == "b")
        P.stm = BLACK;
    else
        return false;

    // castling
    P.castle = 0;
    if (castling != "-")
    {
        for (char c : castling)
        {
            if (c == 'K')
                P.castle |= W_K;
            else if (c == 'Q')
                P.castle |= W_Q;
            else if (c == 'k')
                P.castle |= B_K;
            else if (c == 'q')
                P.castle |= B_Q;
            else
                return false;
        }
    }

    // en passant target
    if (ep == "-")
        P.epSq = -1;
    else
    {
        int s = sqFromStr(ep);
        if (s < 0)
            return false;
        P.epSq = s;
    }

    P.updateOcc();
    return true;
}

// ---- UCI move application ---------------------------------------

bool parseMove(const Position &P, const std::string &uci, Move &out)
{
    if (uci.size() != 4 && uci.size() != 5)
        return false;
    auto sqFromStr = [](const std::string &s) -> int
    {
        if (s.size() != 2)
            return -1;
        char f = s[0], r = s[1];
        if (f < 'a' || f > 'h' || r < '1' || r > '8')
            return -1;
        return SQ(f - 'a', r - '1');
    };
    int from = sqFromStr(uci.substr(0, 2));
    int to   = sqFromStr(uci.substr(2, 2));
    if (from < 0 || to < 0)
        return false;

    std::uint8_t promo = 0;
    if (uci.size() == 5)
    {
        char c = (char) std::tolower((unsigned char) uci[4]);
        if (c == 'q')
            promo = (P.stm == WHITE ? WQ : BQ);
        else if (c == 'r')
            promo = (P.stm == WHITE ? WR : BR);
        else if (c == 'b')
            promo = (P.stm == WHITE ? WB : BB);
        else if (c == 'n')
            promo = (P.stm == WHITE ? WN : BN);
        else
            return false;
    }
    out = {(std::uint16_t) from, (std::uint16_t) to, promo};
    return true;
}

static inline std::string sqToStr(int s)
{
    std::string t;
    t += char('a' + (s % 8));
    t += char('1' + (s / 8));
    return t;
}

std::string moveToUCI(const Move &m)
{
    std::string s = sqToStr(m.from) + sqToStr(m.to);
    if (m.promo)
    {
        char pc = 'q';
        // UCI uses lowercase n/b/r/q regardless of side
        if (m.promo == WN || m.promo == BN)
            pc = 'n';
        else if (m.promo == WB || m.promo == BB)
            pc = 'b';
        else if (m.promo == WR || m.promo == BR)
            pc = 'r';
        else
            pc = 'q';
        s.push_back(pc);
    }
    return s;
}

// Castling/EP/rights identical to your make-move logic, but without Undo
static void applyMoveNoUndo(Position &P, const Move &m)
{
    int moving = pieceAt(P, m.from);
    int cap    = pieceAt(P, m.to);

    bool isPawn = (moving == WP || moving == BP);
    bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);

    if (isEP)
    {
        int takenSq = (P.stm == WHITE ? m.to - 8 : m.to + 8);
        int takenPc = (P.stm == WHITE ? BP : WP);
        removeP(P, takenPc, takenSq);
    }
    else if (cap != NO_PIECE)
    {
        removeP(P, cap, m.to);
    }

    removeP(P, moving, m.from);
    int placeAs = m.promo ? m.promo : moving;
    place(P, placeAs, m.to);

    if (moving == WK && std::abs((int) m.to - (int) m.from) == 2)
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
    if (moving == BK && std::abs((int) m.to - (int) m.from) == 2)
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
    if (cap == WR || cap == BR)
        clearCastleOnRookSq(m.to);

    P.epSq = -1;
    if (isPawn && std::abs((int) m.to - (int) m.from) == 16)
        P.epSq = ((int) m.to + (int) m.from) / 2;

    P.updateOcc();
    P.stm = (P.stm == WHITE ? BLACK : WHITE);
}

bool applyUCIMove(Position &P, const std::string &uci)
{
    Move want{};
    if (!parseMove(P, uci, want))
        return false;

    std::vector<Move> legal;
    legalMoves(P, legal);
    for (const auto &m : legal)
    {
        if (m.from == want.from && m.to == want.to)
        {
            if ((want.promo && m.promo == want.promo) || (!want.promo && m.promo == 0))
            {
                applyMoveNoUndo(P, m);
                return true;
            }
        }
    }
    return false;
}

bool applyUCIMoves(Position &P, const std::vector<std::string> &ms)
{
    for (const auto &s : ms)
    {
        if (!applyUCIMove(P, s))
            return false;
    }
    return true;
}
