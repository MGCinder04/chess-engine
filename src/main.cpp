#include "attacks.hpp"
#include <iostream>

// Defined in uci.cpp
void runUci();

int main()
{
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    init_attacks();
    runUci();
    return 0;
}
