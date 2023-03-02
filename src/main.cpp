#include <cstdlib>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>

struct circle{
    float x, y;
    //std::vector<float> pos;
    // std::vector<float> vel;
    // std::vector<float> acc;

    // circle(); //default constructor

    // circle(std::vector<float> pos){
    //     pos = pos;
    //     // vel = vel;
    //     // acc = acc;
    //         }

    circle() : x(p6::random::number(-1, 1)), y(p6::random::number(-1, 1)) {}
    circle(float x, float y) : x(x), y(y) {}

    void update_position(){
         x += p6::random::number(-0.01, 0.01);
         y += p6::random::number(-0.01, 0.01);
    }
};

std::vector<circle> initialise_positions(int n){
// procedure puts all the boids at a starting position. I put them all at random 
// locations off-screen to start with, that way when the simulation starts they all 
// fly in towards the middle of the screen, rather than suddenly appearing in mid-air.

    std::vector<circle> boids(n);

    for (int i = 0; i<n ; i++){
        boids[i] = circle();
    }

    return boids;
}

// void move_all_boids_to_new_positions(std::vector<circle> boids, int n){
//     for (int i = 0; i<n ; i++){
//     boids[i].update_position();
// };
// }

void draw_boids(std::vector<circle> boids, int n){
        // Actual app
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    ctx.maximize_window();

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::AppleGreen);

        for (int i = 0; i<n ; i++){
        ctx.circle({boids[i].x, boids[i].y},
        p6::Radius{0.2f}
        );
        boids[i].update_position();
    }
    };
        // Should be done last. It starts the infinite loop.
    ctx.start();
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

    int n = 10; //nombre de boids
    float separationCoef = 0.2;
    float alignCoef = 0.5;
    float cohesionCoef = 0.5;

    float maxSpeed = 5; 
    float maxForce = 1;

    std::vector<circle> boids = initialise_positions(n);
    draw_boids(boids,10);

}