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
            tt_clear(); // clear TT between games
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

            // helper: apply tokens one-by-one and report the first failure
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

                // optional moves after startpos
                size_t mpos = line.find(" moves ");
                if (mpos != string::npos)
                {
                    string rest = line.substr(mpos + 7);
                    istringstream iss(rest);
                    vector<string> ms;
                    string tok;
                    while (iss >> tok)
                        ms.push_back(tok);
                    (void) apply_tokens(ms); // apply & report if anything fails
                }
            }
            else
            {
                // "position fen <FEN> [moves ...]"
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
                        (void) apply_tokens(ms); // apply & report if anything fails
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
        // optional tiny tester for parseMove/moveToUCI
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
        // uci.cpp (ADD inside runUciLoop command loop)
        else if (line.rfind("go", 0) == 0)
        {
            int depth = 6;
            {
                std::istringstream ss(line);
                std::string tok;
                while (ss >> tok)
                    if (tok == "depth")
                        ss >> depth;
            }

            search_set_start_time();
            search_set_stop(0);

            auto res = search_iterative(P, depth);

            auto toUci = [&](const Move &m)
            {
                auto sqTo = [&](int s)
                {
                    string r;
                    r.push_back('a' + (s % 8));
                    r.push_back('1' + (s / 8));
                    return r;
                };
                string s = sqTo(m.from) + sqTo(m.to);
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

            if (res.pv.empty())
            {
                cout << "bestmove 0000\n" << flush;
            }
            else
            {
                cout << "bestmove " << toUci(res.bestMove) << "\n" << flush;
            }
        }

        else if (line.rfind("setoption", 0) == 0)
        {
            // parse: setoption name XXX value YYY
            std::string name, value;
            std::istringstream ss(line);
            std::string tok;
            ss >> tok; // setoption
            while (ss >> tok)
            {
                if (tok == "name")
                {
                    name.clear();
                    // read until "value" or end
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
                // stored but unused (engine is single-threaded now)
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
