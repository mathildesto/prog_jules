#include <cmath>
#include <cstdlib>
#include "imgui.h"
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>
#include<boid.hpp>

static void render_gui(ParamBoids& param){

        ImGui::SliderInt("numberOfBoids", &param.numberOfBoids, 2, 100);

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

void resize(ParamBoids& param, std::vector<Boid> & boids){
    int dif = static_cast<int>(boids.size()) - param.numberOfBoids;
    if (dif < 0) {
        for (int i = 0; i < -dif; i++) {
            boids.push_back(Boid());
        }
    } else if (dif > 0) {
        for (int i = 0; i < dif; i++) {
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

        ctx.imgui                 = [&]() {
        // Show a simple window
        ImGui::Begin("Test");
        render_gui(param1);

        ImGui::End();

        // Show the official ImGui demo window
        // It is very useful to discover all the widgets available in ImGui
        ImGui::ShowDemoWindow();
    };

    std::vector<Boid> boids1 = initialise_positions(param1.numberOfBoids);
    // Declare your infinite update loop.
    ctx.update = [&]() {
        
        ctx.background(p6::NamedColor::Pink);

        draw_boids(boids1, ctx, param1);
        update_position(boids1, {}, param1);

        //Variation du nombre de boids
        resize(param1, boids1);
    };
        // Should be done last. It starts the infinite loop.
    ctx.start();

}







