#pragma once
#include <vector>
#include "p6/p6.h"


struct Obstacle {
    float               size;
    std::vector<double> position;

    Obstacle()
    {
        position = {p6::random::number(-1, 1), p6::random::number(-1, 1)};
        size     = p6::random::number(0, 0.3);
    }
};
