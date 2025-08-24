#pragma once
#include "position.hpp"
#include <string>
#include <vector>

using namespace std;

// Convert an internal Move to SAN notation for the given position.
string moveToSAN(Position &P, const Move &m);

// Parse SAN like "e4", "Nf3", "exd5", "Qxe5+", "O-O", "O-O-O", "a8=Q".
// On success, fills `out` with a legal Move for the current position.
bool parseSAN(const Position &P, const string &san, Move &out);
