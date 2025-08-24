#include "attacks.hpp"
#include "zobrist.hpp"

#include <iostream>

using namespace std;

void runUci();

int main()
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    init_attacks();
    init_zobrist();
    runUci();
    return 0;
}
