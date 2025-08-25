#include "attacks.hpp"
#include "eval.hpp"
#include "fen.hpp"
#include "position.hpp"
#include "search.hpp"   
extern void tt_clear(); 

#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

static int g_moveOverheadMs = 30; 

static inline void runUciLoop()
{
    Position P;
    P.setStart();

    string line;
    while (getline(cin, line))
    {
        if (line == "uci")
        {
            cout << "id name ChessEngine\nid author You\nuciok\n" << flush;
            cout << "option name Hash type spin default 64 min 1 max 4096\n" << flush;
            cout << "option name Threads type spin default 1 min 1 max 32\n" << flush;
            cout << "option name Move Overhead type spin default 30 min 0 max 1000\n" << flush;
        }
        else if (line == "isready")
        {
            cout << "readyok\n" << flush;
        }
        else if (line == "ucinewgame")
        {
            tt_clear(); 
            P.setStart();
            cout << "info string new game\n" << flush;
        }

        else if (line.rfind("position", 0) == 0)
        {

            auto trim = [](string s)
            {
                while (!s.empty() && isspace((unsigned char) s.front()))
                    s.erase(s.begin());
                while (!s.empty() && isspace((unsigned char) s.back()))
                    s.pop_back();
                return s;
            };

            auto apply_tokens = [&](vector<string> &ms)
            {
                for (size_t i = 0; i < ms.size(); ++i)
                {
                    if (!applyUserMove(P, ms[i]))
                    {
                        cout << "info string illegal move in sequence at token #" << (i + 1) << " = \"" << ms[i]
                                  << "\"\n"
                                  << flush;
                        return false;
                    }
                }
                return true;
            };

            if (line.find("startpos") != string::npos)
            {
                P.setStart();

                size_t mpos = line.find(" moves ");
                if (mpos != string::npos)
                {
                    string rest = line.substr(mpos + 7);
                    istringstream iss(rest);
                    vector<string> ms;
                    string tok;
                    while (iss >> tok)
                        ms.push_back(tok);
                    (void) apply_tokens(ms);

                    // === SIMPLE CHECK: make sure side-to-move is correct after 'position' ===
                    {
                        int ply       = (int) ms.size();
                        Side expected = (ply % 2 == 0 ? WHITE : BLACK);
                        if (P.stm != expected)
                        {
                            cout << "info string DEBUG: stm mismatch after position; fixing\n" << flush;
                            P.stm = expected; // force-correct it
                        }
                    }
                }
            }
            else
            {
                const string key = "position fen ";
                size_t pos            = line.find(key);
                if (pos == string::npos)
                {
                    cout << "info string unsupported position command\n" << flush;
                }
                else
                {
                    size_t fenStart = pos + key.size();
                    size_t mpos     = line.find(" moves ", fenStart);
                    string fen = (mpos == string::npos) ? trim(line.substr(fenStart))
                                                                  : trim(line.substr(fenStart, mpos - fenStart));
                    if (!setFromFEN(P, fen))
                    {
                        cout << "info string fen parse error\n" << flush;
                    }
                    else if (mpos != string::npos)
                    {
                        string rest = line.substr(mpos + 7);
                        istringstream iss(rest);
                        vector<string> ms;
                        string tok;
                        while (iss >> tok)
                            ms.push_back(tok);
                        (void) apply_tokens(ms);

                        // === SIMPLE CHECK: make sure side-to-move is correct after 'position' ===
                        {
                            int ply       = (int) ms.size();
                            Side expected = (ply % 2 == 0 ? WHITE : BLACK);
                            if (P.stm != expected)
                            {
                                cout << "info string DEBUG: stm mismatch after position; fixing\n" << flush;
                                P.stm = expected; // force-correct it
                            }
                        }
                    }
                }
            }
        }
        else if (line.rfind("go perft ", 0) == 0)
        {
            int d = stoi(line.substr(9));
            cout << perft(P, d) << "\n" << flush;
        }
        else if (line.rfind("perftsplit ", 0) == 0)
        {
            int d = stoi(line.substr(11));
            perftSplit(P, d);
        }
        else if (line == "eval")
        {
            int cp     = evaluate(P);
            int stm_cp = (P.stm == WHITE ? cp : -cp);
            cout << "info string eval " << cp << " cp (white POV), " << stm_cp << " cp (side-to-move POV)\n"
                      << flush;
        }
        else if (line.rfind("echo ", 0) == 0)
        {
            string mv = line.substr(5);
            Move m{};
            if (parseMove(P, mv, m))
                cout << "ok " << moveToUCI(m) << "\n" << flush;
            else
                cout << "bad move\n" << flush;
        }
        else if (line == "quit")
        {
            break;
        }
        else if (line.rfind("go", 0) == 0)
        {
            int depth          = -1;
            long long movetime = -1;

            // parse "go depth N" / "go movetime M"
            {
                std::istringstream ss(line);
                std::string tok;
                while (ss >> tok)
                {
                    if (tok == "depth")
                        ss >> depth;
                    else if (tok == "movetime")
                        ss >> movetime;
                }
            }

            search_set_start_time();

            if (movetime > 0)
            {
                long long budget = movetime - g_moveOverheadMs;
                if (budget < 1)
                    budget = 1;
                search_set_time_limit_ms((unsigned long long) budget);
            }
            else
            {
                search_set_time_limit_ms(0); // no limit
            }

            int callDepth = (depth > 0 ? depth : 99);

            auto res = search_iterative(P, callDepth);

            // collect current legal moves
            std::vector<Move> rootMoves;
            legalMoves(P, rootMoves);

            // fallback if search result is illegal or empty
            auto ensure_legal_best = [&](Move m) -> Move
            {
                for (auto &lm : rootMoves)
                    if (sameMove(lm, m))
                        return m;
                Move tt{};
                if (search_root_tt_move(P, &tt))
                    for (auto &lm : rootMoves)
                        if (sameMove(lm, tt))
                            return tt;
                if (!rootMoves.empty())
                {
                    cout << "info string filtered illegal bestmove; using first legal\n" << flush;
                    return rootMoves[0];
                }
                return Move{0, 0, 0}; // no legal moves
            };

            res.bestMove = ensure_legal_best(res.bestMove);
            if (res.pv.empty() && (res.bestMove.from | res.bestMove.to))
                res.pv = {res.bestMove};

#ifndef NDEBUG
            if (!rootMoves.empty())
            {
                bool found = false;
                for (auto &m : rootMoves)
                    if (sameMove(m, res.bestMove))
                    {
                        found = true;
                        break;
                    }
                if (!found && (res.bestMove.from | res.bestMove.to))
                    cout << "info string DEBUG: bestmove not in legal set after filtering\n" << flush;
            }
#endif

            // print move
            auto toUci = [&](const Move &m)
            {
                auto sqTo = [&](int s)
                {
                    std::string r;
                    r.push_back('a' + (s % 8));
                    r.push_back('1' + (s / 8));
                    return r;
                };
                std::string s = sqTo(m.from) + sqTo(m.to);
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
            };

            if ((res.bestMove.from | res.bestMove.to) == 0)
                cout << "bestmove 0000\n" << flush;
            else
                cout << "bestmove " << toUci(res.bestMove) << "\n" << flush;
        }

        else if (line.rfind("setoption", 0) == 0)
        {
            std::string name, value;
            std::istringstream ss(line);
            std::string tok;
            ss >> tok; 
            while (ss >> tok)
            {
                if (tok == "name")
                {
                    name.clear();
                    while (ss >> tok && tok != "value")
                    {
                        if (!name.empty())
                            name += ' ';
                        name += tok;
                    }
                    if (tok == "value")
                    {
                        std::getline(ss, value);
                        if (!value.empty() && value[0] == ' ')
                            value.erase(0, 1);
                    }
                    break;
                }
            }

            if (name == "Hash")
            {
                int mb = 64;
                try
                {
                    mb = std::stoi(value);
                }
                catch (...)
                {
                }
                tt_resize_mb(mb);
                tt_clear();
                cout << "info string Hash set to " << mb << " MB\n" << flush;
            }
            else if (name == "Threads")
            {
                int thr = 1;
                try
                {
                    thr = std::stoi(value);
                }
                catch (...)
                {
                }
                cout << "info string Threads set to " << thr << " (single-threaded)\n" << flush;
            }
            else if (name == "Move Overhead")
            {
                int mo = 30;
                try
                {
                    mo = std::stoi(value);
                }
                catch (...)
                {
                }
                cout << "info string Move Overhead set to " << mo << " ms\n" << flush;
            }
            else
            {
                cout << "info string unknown option '" << name << "'\n" << flush;
            }
        }
        else if (line == "stop")
        {
            search_set_stop(1);
            cout << "info string stop requested\n" << flush;
        }
    }
}

void runUci()
{
    runUciLoop();
}
