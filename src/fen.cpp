#include "fen.hpp"
#include "san.hpp"
#include <cctype>
#include <cstring>
#include <sstream>
#include <vector>

static int sqFromStr(const std::string &s)
{
    if (s.size() != 2)
        return -1;
    char f = s[0], r = s[1];
    if (f < 'a' || f > 'h' || r < '1' || r > '8')
        return -1;
    return SQ(f - 'a', r - '1');
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

bool setFromFEN(Position &P, const std::string &fen)
{
    std::istringstream iss(fen);
    std::string board, active, castling, ep, hm, fm;
    if (!(iss >> board >> active >> castling >> ep))
        return false;
    iss >> hm >> fm;

    memset(P.bb12, 0, sizeof(P.bb12));
    int r = 7, f = 0;
    for (char c : board)
    {
        if (c == '/')
        {
            --r;
            f = 0;
            continue;
        }
        if (isdigit((unsigned char) c))
        {
            f += c - '0';
            continue;
        }
        int p = pieceFromChar(c);
        if (p == NO_PIECE)
            return false;
        P.bb12[p] |= sqbb(SQ(f, r));
        ++f;
    }

    P.stm    = (active == "w" ? WHITE : BLACK);
    P.castle = 0;
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
    }
    P.epSq = (ep == "-" ? -1 : sqFromStr(ep));
    P.updateOcc();
    return true;
}

bool parseMove(const Position &P, const std::string &uci, Move &out)
{
    if (uci.size() != 4 && uci.size() != 5)
        return false;
    auto sqFrom2 = [](const std::string &s) -> int
    {
        if (s.size() != 2)
            return -1;
        char f = s[0], r = s[1];
        if (f < 'a' || f > 'h' || r < '1' || r > '8')
            return -1;
        return SQ(f - 'a', r - '1');
    };
    int from = sqFrom2(uci.substr(0, 2)), to = sqFrom2(uci.substr(2, 2));
    if (from < 0 || to < 0)
        return false;

    uint8_t promo = 0;
    if (uci.size() == 5)
    {
        char c = (char) tolower((unsigned char) uci[4]);
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
    out = {(uint16_t) from, (uint16_t) to, promo};
    return true;
}

std::string moveToUCI(const Move &m)
{
    auto sqToStr = [&](int s)
    {
        std::string t;
        t += (char) ('a' + (s % 8));
        t += (char) ('1' + (s / 8));
        return t;
    };
    std::string s = sqToStr(m.from) + sqToStr(m.to);
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
    return s;
}

// Apply move (no undo info)
static void applyMoveNoUndo(Position &P, const Move &m)
{
    int moving = pieceAt(P, m.from), cap = pieceAt(P, m.to);
    bool isPawn = (moving == WP || moving == BP);
    bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);

    if (isEP)
    {
        int sq = (P.stm == WHITE ? m.to - 8 : m.to + 8);
        int pc = (P.stm == WHITE ? BP : WP);
        removeP(P, pc, sq);
    }
    else if (cap != NO_PIECE)
        removeP(P, cap, m.to);

    removeP(P, moving, m.from);
    place(P, (m.promo ? m.promo : moving), m.to);

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

    auto clearCastle = [&](int sq)
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
        clearCastle(m.from);
    if (cap == WR || cap == BR)
        clearCastle(m.to);

    P.epSq = -1;
    if (isPawn && abs(m.to - m.from) == 16)
        P.epSq = (m.to + m.from) / 2;

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
    for (auto &m : legal)
        if (sameMove(m, want))
        {
            applyMoveNoUndo(P, m);
            return true;
        }
    return false;
}

bool applyUCIMoves(Position &P, const std::vector<std::string> &ms)
{
    for (auto &s : ms)
        if (!applyUCIMove(P, s))
            return false;
    return true;
}

bool applyUserMove(Position &P, const std::string &token)
{
    if (applyUCIMove(P, token))
        return true;
    Move m{};
    if (parseSAN(P, token, m))
    {
        std::vector<Move> legal;
        legalMoves(P, legal);
        for (auto &lm : legal)
            if (sameMove(lm, m))
            {
                applyMoveNoUndo(P, lm);
                return true;
            }
    }
    return false;
}
