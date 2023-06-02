#pragma once
#include "boid.hpp"
#include "food.hpp"
#include "obstacle.hpp"
#include "p6/p6.h"

std::vector<Obstacle> initialise_positions_obstacle(int n);
void                  draw_obstacle(const std::vector<Obstacle>& obstacles, p6::Context& ctx);
