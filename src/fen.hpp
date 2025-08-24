#pragma once
#include "position.hpp"
#include <string>
#include <vector>

using namespace std;

bool setFromFEN(Position &P, const string &fen);

bool parseMove(const Position &P, const string &uci, Move &out);
string moveToUCI(const Move &m);

bool applyUCIMove(Position &P, const string &uci);
bool applyUCIMoves(Position &P, const vector<string> &ms);

bool applyUserMove(Position &P, const string &token);
bool applyUserMoves(Position &P, const vector<string> &tokens);
