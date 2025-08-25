#include "san.hpp"
#include "attacks.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>

using namespace std;

static inline U64 sqbbU(unsigned s)
{
    return 1ull << s;
}
static int pieceAt(const Position &P, int s)
{
    U64 m = sqbbU((unsigned) s);
    for (int p = WP; p <= BK; ++p)
        if (P.bb12[p] & m)
            return p;
    return NO_PIECE;
}
static inline string sqToStr(int s)
{
    string t;
    t += char('a' + (s % 8));
    t += char('1' + (s / 8));
    return t;
}
static inline bool isWhitePiece(int p)
{
    return p >= WP && p <= WK;
}
static inline bool isBlackPiece(int p)
{
    return p >= BP && p <= BK;
}

static bool isCapture(const Position &P, const Move &m)
{
    int capHere = pieceAt(P, m.to);
    if (capHere != NO_PIECE)
        return true;
    int moving  = pieceAt(P, m.from);
    bool isPawn = (moving == WP || moving == BP);
    if (isPawn && m.to == P.epSq)
        return true; 
    return false;
}

static void doMake(Position &P, const Move &m, Undo &u)
{
    u.stm        = P.stm;
    u.m          = m;
    u.capPiece   = NO_PIECE;
    u.capSq      = -1;
    u.prevCastle = P.castle;
    u.prevEp     = P.epSq;

    auto place = [&](int p, int s) { P.bb12[p] |= (1ull << s); };
    auto rem   = [&](int p, int s) { P.bb12[p] &= ~(1ull << s); };

    int moving  = pieceAt(P, m.from);
    int cap     = pieceAt(P, m.to);
    bool isPawn = (moving == WP || moving == BP);
    bool isEP   = isPawn && (m.to == P.epSq) && (cap == NO_PIECE);
    if (isEP)
    {
        int ts     = (P.stm == WHITE ? m.to - 8 : m.to + 8);
        u.capPiece = (P.stm == WHITE ? BP : WP);
        u.capSq    = ts;
        rem(u.capPiece, ts);
    }
    else if (cap != NO_PIECE)
    {
        u.capPiece = cap;
        u.capSq    = m.to;
        rem(cap, m.to);
    }
    rem(moving, m.from);
    int placeAs = m.promo ? m.promo : moving;
    place(placeAs, m.to);

    if (moving == WK && abs(m.to - m.from) == 2)
    {
        if (m.to == G1)
        {
            rem(WR, H1);
            place(WR, F1);
        }
        else if (m.to == C1)
        {
            rem(WR, A1);
            place(WR, D1);
        }
    }
    if (moving == BK && abs(m.to - m.from) == 2)
    {
        if (m.to == G8)
        {
            rem(BR, H8);
            place(BR, F8);
        }
        else if (m.to == C8)
        {
            rem(BR, A8);
            place(BR, D8);
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
}

static void doUnmake(Position &P, const Undo &u)
{
    auto place = [&](int p, int s) { P.bb12[p] |= (1ull << s); };
    auto rem   = [&](int p, int s) { P.bb12[p] &= ~(1ull << s); };

    P.stm    = u.stm;
    P.castle = u.prevCastle;
    P.epSq   = u.prevEp;

    int moved = pieceAt(P, u.m.to);
    if ((moved == WK || moved == BK) && abs(u.m.to - u.m.from) == 2)
    {
        if (moved == WK)
        {
            if (u.m.to == G1)
            {
                rem(WR, F1);
                place(WR, H1);
            }
            else if (u.m.to == C1)
            {
                rem(WR, D1);
                place(WR, A1);
            }
        }
        else
        {
            if (u.m.to == G8)
            {
                rem(BR, F8);
                place(BR, H8);
            }
            else if (u.m.to == C8)
            {
                rem(BR, D8);
                place(BR, A8);
            }
        }
    }
    rem(moved, u.m.to);
    int orig = u.m.promo ? (u.stm == WHITE ? WP : BP) : moved;
    place(orig, u.m.from);
    if (u.capPiece != NO_PIECE && u.capSq != -1)
        place(u.capPiece, u.capSq);

    P.updateOcc();
}

string moveToSAN(Position &P, const Move &m)
{
    int moving = pieceAt(P, m.from);
    if (moving == WK && abs(m.to - m.from) == 2)
    {
        return (m.to == G1 ? "O-O" : "O-O-O");
    }
    if (moving == BK && abs(m.to - m.from) == 2)
    {
        return (m.to == G8 ? "O-O" : "O-O-O");
    }

    string s;
    bool capture = isCapture(P, m);
    bool pawn    = (moving == WP || moving == BP);

    if (!pawn)
    {
        const char letter[12] = {'P', 'N', 'B', 'R', 'Q', 'K', 'p', 'n', 'b', 'r', 'q', 'k'};
        char L                = letter[moving];
        s += toupper(L);
        vector<Move> leg;
        legalMoves(const_cast<Position &>(P), leg);
        int count = 0;
        for (auto &x : leg)
        {
            if (x.to == m.to && x.promo == m.promo && x.from != m.from)
            {
                int tMoving = pieceAt(P, x.from);
                if (tMoving == moving)
                    ++count;
            }
        }

        if (count > 0)
        {
            bool anySameFile = false, anySameRank = false;
            for (auto &x : leg)
            {
                if (x.to == m.to && x.promo == m.promo)
                {
                    int tMoving = pieceAt(P, x.from);
                    if (tMoving == moving && x.from != m.from)
                    {
                        if ((x.from % 8) == (m.from % 8))
                            anySameFile = true;
                        if ((x.from / 8) == (m.from / 8))
                            anySameRank = true;
                    }
                }
            }
            if (anySameFile && anySameRank)
            {
                s += char('a' + (m.from % 8));
                s += char('1' + (m.from / 8));
            }
            else if (anySameFile)
            {
                s += char('1' + (m.from / 8));
            }
            else
            {
                s += char('a' + (m.from % 8));
            }
        }
    }
    else
    {
        if (capture)
            s += char('a' + (m.from % 8));
    }

    if (capture)
        s += 'x';
    s += sqToStr(m.to);

    if (m.promo)
    {
        char pc = 'Q';
        if (m.promo == WN || m.promo == BN)
            pc = 'N';
        else if (m.promo == WB || m.promo == BB)
            pc = 'B';
        else if (m.promo == WR || m.promo == BR)
            pc = 'R';
        s += '=';
        s += pc;
    }

    Undo u;
    doMake(P, m, u);
    bool oppInCheck = sqAttacked(P, P.kingSq[P.stm], (P.stm == WHITE ? BLACK : WHITE));
    vector<Move> opp;
    legalMoves(P, opp);
    bool oppHasMove = !opp.empty();
    doUnmake(P, u);

    if (oppInCheck)
    {
        s += (oppHasMove ? '+' : '#');
    }

    return s;
}

static string norm(const string &s)
{
    string t;
    for (char c : s)
    {
        if (c == '0')
            c = 'O'; 
        if (c == '+' || c == '#' || c == '!' || c == '?')
            continue; 
        if (!isspace((unsigned char) c))
            t += c;
    }
    return t;
}

bool parseSAN(const Position &P, const string &san, Move &out)
{
    string want = norm(san);
    vector<Move> leg;
    legalMoves(const_cast<Position &>(P), leg);
    for (auto &m : leg)
    {
        Position tmp  = P;
        string s = moveToSAN(tmp, m);
        if (norm(s) == want)
        {
            out = m;
            return true;
        }
    }
    return false;
}
