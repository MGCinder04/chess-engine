#pragma once
#include "position.hpp"
#include <string>
#include <vector>

using namespace std;

string moveToSAN(Position &P, const Move &m);

bool parseSAN(const Position &P, const string &san, Move &out);
