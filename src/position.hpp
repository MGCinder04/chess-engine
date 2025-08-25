#pragma once
#include "attacks.hpp"
#include "chess_types.hpp"
#include <string>
#include <vector>

using namespace std;

struct Move
{
    uint16_t from, to;
    uint8_t promo{0};
};

struct Undo
{
    Move m;
    int capPiece{NO_PIECE};
    int capSq{-1};
    Side stm;
    uint8_t prevCastle{0};
    int prevEp{-1};
};

struct Position
{
    U64 bb12[12]{};
    U64 occ[2]{};
    U64 occAll{};
    Side stm = WHITE;
    int kingSq[2]{E1, E8};
    uint8_t castle = (W_K | W_Q | B_K | B_Q);
    int epSq            = -1;

    void updateOcc();
    void setStart();
};

bool sqAttacked(const Position &P, int sq, Side by);
void legalMoves(Position &P, vector<Move> &out);

uint64_t perft(Position &P, int d);
void perftSplit(Position &P, int depth);


bool applyUCIMove(Position &P, const string &uci);              
bool applyUCIMoves(Position &P, const vector<string> &ms); 
