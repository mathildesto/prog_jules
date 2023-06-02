#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "boid.hpp"
#include "food.hpp"
#include "food_helper.hpp"
#include "imgui.h"
#include "obstacle.hpp"
#include "obstacle_helper.hpp"
#include "p6/p6.h"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

static void render_gui(ParamBoids& param)
{
    ImGui::SliderInt("numberOfBoids", &param.numberOfBoids, 2, 100);

    ImGui::SliderFloat("biasval", &param.visualRange, 0.f, 5.0f);
    ImGui::SliderFloat("visualRange", &param.visualRange, 0.f, 2.0f);
    ImGui::SliderFloat("ProtectedRange", &param.protectedRange, 0.f, 0.5f);

    ImGui::SliderFloat("turnfactor", &param.turnfactor, 0.f, 0.1f);
    ImGui::SliderFloat("centeringfactor", &param.centeringfactor, 0.f, 0.001f);
    ImGui::SliderFloat("avoidfactor", &param.avoidfactor, 0.f, 0.1f);
    ImGui::SliderFloat("matchingfactor", &param.matchingfactor, 0.f, 0.1f);

    ImGui::SliderFloat("MaxSpeed", &param.maxspeed, 0.f, 0.05f);
    ImGui::SliderFloat("MinSpeed", &param.minspeed, 0.f, 0.05f);

    ImGui::SliderFloat("Size", &param.boidSize, 0.f, 0.1f);
}

void resize(ParamBoids& param, std::vector<Boid>& boids)
{
    int dif = static_cast<int>(boids.size()) - param.numberOfBoids;
    if (dif < 0)
    {
        for (int i = 0; i < -dif; i++)
        {
            boids.push_back(Boid());
        }
    }
    else if (dif > 0)
    {
        for (int i = 0; i < dif; i++)
        {
            boids.pop_back();
        }
    }
}

int main(int argc, char* argv[])
{
    { // Run the tests
        if (doctest::Context{}.run() != 0)
            return EXIT_FAILURE;
        // The CI does not have a GPU so it cannot run the rest of the code.
        const bool no_gpu_available = argc >= 2 && strcmp(argv[1], "-nogpu") == 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (no_gpu_available)
            return EXIT_SUCCESS;
    }

    ParamBoids param1;

    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};

    ctx.imgui = [&]() {
        // Show a simple window
        ImGui::Begin("Test");
        render_gui(param1);

        ImGui::End();

        // Show the official ImGui demo window
        // It is very useful to discover all the widgets available in ImGui
        ImGui::ShowDemoWindow();
    };

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int                   randomFoodCount = std::rand() % 6;
    std::vector<Boid>     boids1          = initialise_positions(param1.numberOfBoids);
    std::vector<Food>     foods1          = initialise_positions_food(randomFoodCount);
    std::vector<Obstacle> obstacle1       = initialise_positions_obstacle(1);
    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::Pink);

        draw_boids(boids1, ctx, param1);
        float collisionThreshold = 0.1f; //  seuil de collision
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        int randomFoodCount = std::rand() % 6;
        ensure_food_existence(foods1, randomFoodCount);
        update_size_food(foods1, boids1, collisionThreshold);

        update_position(boids1, {}, param1, foods1, obstacle1);
        draw_foods(foods1, ctx);
        draw_obstacle(obstacle1, ctx);

        // Variation du nombre de boids
        resize(param1, boids1);
    };
    // Should be done last. It starts the infinite loop.
    ctx.start();
}
