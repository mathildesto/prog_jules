#include "boid.hpp"
#include "food.hpp"
#include "p6/p6.h"

template<typename T1, typename T2>
float compute_distance(const T1& object1, const T2& object2)
{
    // distance = sqrt((x2 - x1)² + (y2 - y1)²)
    return sqrt(pow(object2.position[0] - object1.position[0], 2) + pow(object2.position[1] - object1.position[1], 2));
}

void keep_inside_boundaries(ParamBoids& param, Window& window, Boid& boid)
{
    if (boid.position[0] < window.WINDOW_MIN_X)
    {
        boid.velocity[0] += param.turnfactor; // reverse the x-component of the velocity
    }
    else if (boid.position[0] > window.WINDOW_MAX_X)
    {
        boid.velocity[0] += -param.turnfactor;
    }
    if (boid.position[1] < window.WINDOW_MIN_Y)
    {
        boid.velocity[1] += param.turnfactor;
    }
    else if (boid.position[1] > window.WINDOW_MAX_Y)
    {
        boid.velocity[1] += -param.turnfactor;
    }
}

void limit_speed(ParamBoids& param, Boid& boid)
{
    float speed = sqrt(boid.velocity[0] * boid.velocity[0] + boid.velocity[1] * boid.velocity[1]);

    if (speed < param.minspeed)
    {
        boid.velocity[0] = (boid.velocity[0] / speed) * param.minspeed;
        boid.velocity[1] = (boid.velocity[1] / speed) * param.minspeed;
    }
    if (speed > param.maxspeed)
    {
        boid.velocity[0] = (boid.velocity[0] / speed) * param.maxspeed;
        boid.velocity[1] = (boid.velocity[1] / speed) * param.maxspeed;
    }
}

void separation(std::vector<Boid>& boids, ParamBoids& param, Boid& boid)
{
    float close_dx = 0;
    float close_dy = 0;

    for (int j = 0; j < boids.size(); j++)
    {
        if (boid == boids[j])
            continue;

        float distance = compute_distance(boid, boids[j]);

        // Is squared distance less than the protected range?
        if (distance < param.protectedRange)
        {
            // If so, calculate difference in x/y-coordinates to nearfield boid
            close_dx += boid.position[0] - boids[j].position[0];
            close_dy += boid.position[1] - boids[j].position[1];
        }
    }

    boid.velocity[0] += (close_dx * param.avoidfactor); // Add the avoidance contribution to velocity
    boid.velocity[1] += (close_dy * param.avoidfactor);
}

void cohesion(std::vector<Boid>& boids, ParamBoids& param, Boid& boid)
{
    float xvel_avg          = 0;
    float yvel_avg          = 0;
    float neighboring_boids = 0;

    for (int j = 0; j < boids.size(); j++)
    {
        if (boid == boids[j])
            continue;

        float distance = compute_distance(boid, boids[j]);

        // Is squared distance less than the visual range?
        if (distance < param.visualRange)
        {
            // Add other boid's x/y-coord and x/y vel to accumulator variables
            xvel_avg += boids[j].velocity[0];
            yvel_avg += boids[j].velocity[1];

            // Increment number of boids within visual range
            neighboring_boids += 1;
        }
    }

    // If there were any boids in the visual range . . .
    if (neighboring_boids > 0)
    {
        // Divide accumulator variables by number of boids in visual range
        xvel_avg = xvel_avg / neighboring_boids;
        yvel_avg = yvel_avg / neighboring_boids;

        boid.velocity[0] = boid.velocity[0] + (xvel_avg - boid.velocity[0]) * param.matchingfactor; // Add the centering/matching contributions to velocity
        boid.velocity[1] = boid.velocity[1] + (yvel_avg - boid.velocity[1]) * param.matchingfactor;
    }
}

void alignment(std::vector<Boid>& boids, ParamBoids& param, Boid& boid)
{
    float xpos_avg          = 0;
    float ypos_avg          = 0;
    float neighboring_boids = 0;

    for (int j = 0; j < boids.size(); j++)
    {
        if (boid == boids[j])
            continue;

        float distance = compute_distance(boid, boids[j]);

        // Is squared distance less than the protected range?
        if (distance < param.visualRange)
        {
            // Add other boid's x/y-coord and x/y vel to accumulator variables
            xpos_avg += boids[j].position[0];
            ypos_avg += boids[j].position[1];

            // Increment number of boids within visual range
            neighboring_boids += 1;
        }
    }

    // If there were any boids in the visual range . . .
    if (neighboring_boids > 0)
    {
        // Divide accumulator variables by number of boids in visual range
        xpos_avg = xpos_avg / neighboring_boids;
        ypos_avg = ypos_avg / neighboring_boids;

        boid.velocity[0] += (xpos_avg - boid.position[0]) * param.centeringfactor; // Add the centering/matching contributions to velocity
        boid.velocity[1] += (ypos_avg - boid.position[1]) * param.centeringfactor;
    }
}

void food_attract(std::vector<Boid>& boids, const std::vector<Food>& foods, ParamBoids& param, Boid& boid)
{
    for (const auto& food : foods)
    {
        float distance = compute_distance(boid, food);

        if (distance < param.visualRange)
        {
            float dx = food.position[0] - boid.position[0];
            float dy = food.position[1] - boid.position[1];

            boid.velocity[0] += (dx * param.biasval);
            boid.velocity[1] += (dy * param.biasval);
        }
    }
}

void separation_food(const std::vector<Food>& foods, ParamBoids& param, Boid& boid)
{
    float close_dx = 0;
    float close_dy = 0;

    for (const auto& food : foods)
    {
        float distance = compute_distance(boid, food);

        // Est-ce que la distance est inférieure à la somme des rayons des boids et de la food?
        if (distance < (param.boidSize + food.size))
        {
            // Calculer la différence de coordonnées x/y par rapport à la food proche
            close_dx += boid.position[0] - food.position[0];
            close_dy += boid.position[1] - food.position[1];
        }
    }

    boid.velocity[0] += (close_dx * param.avoidfactor); // Ajouter la contribution d'évitement à la vélocité
    boid.velocity[1] += (close_dy * param.avoidfactor);
}

std::vector<Boid> initialise_positions(int n)
{
    // Adds all the boids at a starting position. I put them all at random locations

    std::vector<Boid> boids(n);
    for (auto& boid : boids)
    {
        boid = Boid();
    }
    return boids;
}

void update_position(std::vector<Boid>& boids, Window window, ParamBoids& param, const std::vector<Food>& foods)
{
    // for each boid
    for (auto& boid : boids)
    {
        alignment(boids, param, boid);

        cohesion(boids, param, boid);

        separation(boids, param, boid);
        food_attract(boids, foods, param, boid);
        separation_food(foods, param, boid);
        keep_inside_boundaries(param, window, boid);

        limit_speed(param, boid);

        // Update boid's position
        //  boid.position[0] += boid.velocity[0];
        //  boid.position[1] += boid.velocity[1];

        boid.add_velocity();
    }
}

void draw_boids(const std::vector<Boid>& boids, p6::Context& ctx, const ParamBoids& param)
{
    for (const auto& boid : boids)
    {
        ctx.circle({float(boid.position[0]), float(boid.position[1])}, p6::Radius{param.boidSize});
    }
}
