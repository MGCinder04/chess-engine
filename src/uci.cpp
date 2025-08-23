#include "attacks.hpp"
#include "position.hpp"
#include <iostream>
#include <string>

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
            if (line.find("startpos") != std::string::npos)
                P.setStart();
            // (FEN + moves will be added later)
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
    }
}

void runUci()
{
    runUciLoop();
}
