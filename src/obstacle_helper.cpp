#include "obstacle_helper.hpp"

std::vector<Obstacle> initialise_positions_obstacle(int n)
{
    std::vector<Obstacle> obstacles(n);
    for (auto& obstacle : obstacles)
    {
        obstacle = Obstacle();
    }
    return obstacles;
}

void draw_obstacle(const std::vector<Obstacle>& obstacles, p6::Context& ctx)
{
    for (const auto& obstacle : obstacles)
    {
        ctx.circle({static_cast<float>(obstacle.position[0]), static_cast<float>(obstacle.position[1])}, p6::Radius{obstacle.size});
    }
}
