#include "attacks.hpp"
#include <iostream>

using namespace std;

void runUci();

int main()
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    init_attacks();
    runUci();
    return 0;
}
