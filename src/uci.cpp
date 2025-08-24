#include "attacks.hpp"
#include "eval.hpp"
#include "fen.hpp"
#include "position.hpp"

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
        }
        else if (line == "isready")
        {
            cout << "readyok\n" << flush;
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
    }
}

void runUci()
{
    runUciLoop();
}
