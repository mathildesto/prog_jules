#include "food_helper.hpp"
#include "boid.hpp"
#include "food.hpp"
#include "p6/p6.h"

std::vector<Food> initialise_positions_food(int n)
{
    // Ajoute toutes les foods à une position de départ. Je les place toutes à des emplacements aléatoires.

    std::vector<Food> foods(n);
    for (auto& food : foods)
    {
        food = Food();
    }
    return foods;
}

void draw_foods(const std::vector<Food>& foods, p6::Context& ctx)
{
    for (const auto& food : foods)
    {
        ctx.circle({float(food.position[0]), float(food.position[1])}, p6::Radius{food.size});
    }
}

void update_size_food(std::vector<Food>& foods, const std::vector<Boid>& boids, float collisionThreshold)
{
    const float sizeReductionFactor = 0.2f;

    for (auto& food : foods)
    {
        bool collided = false;

        for (const auto& boid : boids)
        {
            float distance = compute_distance(boid, food);
            if (distance < collisionThreshold)
            {
                collided = true;
                break;
            }
        }

        if (collided)
        {
            food.size *= sizeReductionFactor;
        }
    }
}
