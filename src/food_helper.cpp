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
    const float sizeReductionFactor1 = 0.01f;   // Diminution normale
    const float sizeReductionFactor2 = 0.0008f; // Diminution réduite
    const float sizeThreshold        = 0.2f;    // Seuil de taille

    for (auto it = foods.begin(); it != foods.end();)
    {
        for (const auto& boid : boids)
        {
            float distance = compute_distance(boid, *it);
            if (distance < collisionThreshold)
            {
                if (it->size > sizeThreshold)
                {
                    it->size -= sizeReductionFactor1;
                }
                else
                {
                    it->size -= sizeReductionFactor2;
                }
            }
        }

        std::cout << "Food size: " << it->size << std::endl;

        if (it->size < 0)
        {
            it = foods.erase(it);
        }
        else if (it->size > 0.2) // Si la taille dépasse 1.0, appliquer une diminution plus importante
        {
            it->size -= (sizeReductionFactor1 * 10);
            ++it;
        }
        else
        {
            ++it;
        }
    }
}
void ensure_food_existence(std::vector<Food>& foods, int desiredCount)
{
    if (foods.empty())
    {
        foods = initialise_positions_food(desiredCount);
    }
}