
#pragma once
#include "boid.hpp"
#include "food.hpp"
#include "p6/p6.h"

std::vector<Food> initialise_positions_food(int n);
void              draw_foods(const std::vector<Food>& foods, p6::Context& ctx);
void              update_size_food(std::vector<Food>& foods, const std::vector<Boid>& boids, float collisionThreshold);