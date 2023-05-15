#pragma once
#include "p6/p6.h"

struct Food {
    float               size;
    std::vector<double> position;

    Food()
    {
        position = {p6::random::number(-1, 1), p6::random::number(-1, 1)};
        size     = {p6::random::number(0, 0.5)};
    }
};
