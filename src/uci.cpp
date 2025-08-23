#include "attacks.hpp"
#include "fen.hpp" // 👈 new
#include "position.hpp"

#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static inline void runUciLoop()
{
    Position P;
    P.setStart();

    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line == "uci")
        {
            std::cout << "id name ChessEngine\nid author You\nuciok\n" << std::flush;
        }
        else if (line == "isready")
        {
            std::cout << "readyok\n" << std::flush;
        }
        else if (line.rfind("position", 0) == 0)
        {
            auto trim = [](std::string s)
            {
                while (!s.empty() && std::isspace((unsigned char) s.front()))
                    s.erase(s.begin());
                while (!s.empty() && std::isspace((unsigned char) s.back()))
                    s.pop_back();
                return s;
            };

            if (line.find("startpos") != std::string::npos)
            {
                P.setStart();
                // optional moves after startpos
                size_t mpos = line.find(" moves ");
                if (mpos != std::string::npos)
                {
                    std::string rest = line.substr(mpos + 7);
                    std::istringstream iss(rest);
                    std::vector<std::string> ms;
                    std::string tok;
                    while (iss >> tok)
                        ms.push_back(tok);
                    if (!applyUCIMoves(P, ms))
                    {
                        std::cout << "info string illegal move in sequence\n" << std::flush;
                    }
                }
            }
            else
            {
                // "position fen <FEN> [moves ...]"
                const std::string key = "position fen ";
                size_t pos            = line.find(key);
                if (pos == std::string::npos)
                {
                    std::cout << "info string unsupported position command\n" << std::flush;
                }
                else
                {
                    size_t fenStart = pos + key.size();
                    size_t mpos     = line.find(" moves ", fenStart);
                    std::string fen = (mpos == std::string::npos) ? trim(line.substr(fenStart))
                                                                  : trim(line.substr(fenStart, mpos - fenStart));

                    if (!setFromFEN(P, fen))
                    {
                        std::cout << "info string fen parse error\n" << std::flush;
                    }
                    else if (mpos != std::string::npos)
                    {
                        std::string rest = line.substr(mpos + 7);
                        std::istringstream iss(rest);
                        std::vector<std::string> ms;
                        std::string tok;
                        while (iss >> tok)
                            ms.push_back(tok);
                        if (!applyUCIMoves(P, ms))
                        {
                            std::cout << "info string illegal move in sequence\n" << std::flush;
                        }
                    }
                }
            }
        }
        else if (line.rfind("go perft ", 0) == 0)
        {
            int d = std::stoi(line.substr(9));
            std::cout << perft(P, d) << "\n" << std::flush;
        }
        else if (line.rfind("perftsplit ", 0) == 0)
        {
            int d = std::stoi(line.substr(11));
            perftSplit(P, d);
        }
        else if (line == "quit")
        {
            break;
        }
        // (You can temporarily add this in uci.cpp to test formatting)
        else if (line.rfind("echo ", 0) == 0)
        {
            std::string mv = line.substr(5);
            Move m{};
            if (parseMove(P, mv, m))
                std::cout << "ok " << moveToUCI(m) << "\n";
            else
                std::cout << "bad move\n";
        }
    }
}

void runUci()
{
    runUciLoop();
}
