#include <cmath>
#include <cstdlib>
#include "imgui.h"
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>
#include<boid.hpp>


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
    
    int n = 100; //nombre de boids
    param_boids param1;
    param_boids param2;

    std::vector<boid> boids1 = initialise_positions(n);
    std::vector<boid> boids2 = initialise_positions(n);


    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};

        ctx.imgui                 = [&]() {
        // Show a simple window
        ImGui::Begin("Test");
        ImGui::SliderInt("nombre de boids", &n, 0, 100);

        ImGui::SliderFloat("visualRange", &param1.visualRange, 0.f, 2.0f);
        ImGui::SliderFloat("ProtectedRange", &param1.protectedRange, 0.f, 0.5f);

        ImGui::SliderFloat("turnfactor", &param1.turnfactor, 0.f, 0.1f);
        ImGui::SliderFloat("centeringfactor", &param1.centeringfactor, 0.f, 0.001f);
        ImGui::SliderFloat("avoidfactor", &param1.avoidfactor, 0.f, 0.1f);
        ImGui::SliderFloat("matchingfactor", &param1.matchingfactor, 0.f, 0.1f);

        ImGui::SliderFloat("MaxSpeed", &param1.maxspeed, 0.f, 0.05f);
        ImGui::SliderFloat("MinSpeed", &param1.minspeed, 0.f, 0.05f);

        ImGui::SliderFloat("Size", &param1.boidSize, 0.f, 0.1f);

        //         ImGui::SliderFloat("visualRange", &param2.visualRange, 0.f, 2.0f);
        // ImGui::SliderFloat("ProtectedRange", &param2.protectedRange, 0.f, 0.5f);

        // ImGui::SliderFloat("turnfactor", &param2.turnfactor, 0.f, 0.1f);
        // ImGui::SliderFloat("centeringfactor", &param2.centeringfactor, 0.f, 0.001f);
        // ImGui::SliderFloat("avoidfactor", &param2.avoidfactor, 0.f, 0.1f);
        // ImGui::SliderFloat("matchingfactor", &param2.matchingfactor, 0.f, 0.1f);

        // ImGui::SliderFloat("MaxSpeed", &param2.maxspeed, 0.f, 0.05f);
        // ImGui::SliderFloat("MinSpeed", &param2.minspeed, 0.f, 0.05f);

        // ImGui::SliderFloat("Size", &param2.boidSize, 0.f, 0.1f);

        ImGui::End();

        // Show the official ImGui demo window
        // It is very useful to discover all the widgets available in ImGui
        ImGui::ShowDemoWindow();
    };

    // Declare your infinite update loop.
    ctx.update = [&]() {
        
        ctx.background(p6::NamedColor::Pink);

        draw_boids(boids1, n, ctx, param1);
        update_position(boids1, n, {}, param1);

        // draw_boids(boids2, n, ctx, param2);
        // update_position(boids2, n, {}, param2);

    };
        // Should be done last. It starts the infinite loop.
    ctx.start();

}







