#pragma once
#include "position.hpp"
#include <string>
#include <vector>

// FEN loader: sets pieces, side-to-move, castling, ep square
bool setFromFEN(Position &P, const std::string &fen);

// Apply a single UCI move ("e2e4", "a7a8q") if it's legal
bool applyUCIMove(Position &P, const std::string &uci);

// Apply a list of UCI moves (returns false if any is illegal)
bool applyUCIMoves(Position &P, const std::vector<std::string> &ms);

// NEW: tiny UCI helpers
bool parseMove(const Position &P, const std::string &uci, Move &out); // "e2e4", "a7a8q"
std::string moveToUCI(const Move &m);