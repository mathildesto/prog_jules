#include <cmath>
#include <cstdlib>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>

struct circle{
    float x, y; //position 

    float dx, dy; //velocity

    circle() : x(p6::random::number(-1, 1)), y(p6::random::number(-1, 1)) {}
    circle(float x, float y) : x(x), y(y) {}

};

   float distance(const circle boid1, const circle boid2 ){
        //distance = sqrt((x2 - x1)² + (y2 - y1)²)
        return sqrt(pow(boid2.x - boid1.x,2) + pow(boid2.y - boid1.y,2));
    }



    std::vector<circle> initialise_positions(int n){
    // Adds all the boids at a starting position. I put them all at random locations

    std::vector<circle> boids(n);

    for (int i = 0; i<n ; i++){
        boids[i] = circle();
    }

    return boids;
}

void update_position(std::vector<circle>& boids, int n, const float WINDOW_MIN_X, const float WINDOW_MAX_X, const float WINDOW_MIN_Y, const float WINDOW_MAX_Y){
    
    // check if the boid is outside the window boundaries
    for(int i = 0; i<n ; i++){
    if (boids[i].x < WINDOW_MIN_X) {
        boids[i].x = WINDOW_MIN_X;
        boids[i].dx *= -1 ; // reverse the x-component of the velocity
    }
    else if (boids[i].x > WINDOW_MAX_X) {
        boids[i].x = WINDOW_MAX_X;
        boids[i].dx *= -1;
    }
    if (boids[i].y < WINDOW_MIN_Y) {
        boids[i].y = WINDOW_MIN_Y;
        boids[i].dy *= -1;
    }
    else if (boids[i].y > WINDOW_MAX_Y) {
        boids[i].y = WINDOW_MAX_Y;
        boids[i].dy *= -1;
    }
    }

    //rule 1 : Boids try to fly towards the centre of mass of neighbouring boids.
    float Sx = 0;
    float Sy = 0;

    for(int i = 0; i<n ; i++){
       Sx += boids[i].x;
       Sy += boids[i].y;
    }
    float Xm = Sx/n;
    float Ym = Sy/n; //vector that contains the x and y coordinates of the center of mass

    for(int i = 0; i<n ; i++){

            boids[i].dx += (Xm - boids[i].x)/1000; // move the boids 0,1% of the way towards the centre
            boids[i].dy += (Ym - boids[i].y)/1000;  
    }


    //rule 2 : Boids try to keep a small distance away from other objects (including other boids)

    for(int i = 0; i<n ; i++){
        for(int j = i+1; j<n ; j++)
            if (distance(boids[i],boids[j])<0.2){
                boids[i].dx -= abs(boids[i].x - boids[j].x);
                boids[i].dy -= abs(boids[i].y - boids[j].y); 
            }
        }


    // rule 3 Boids try to match velocity with near boids.

        float Vx = 0;
        float Vy = 0;
        for(int i = 0; i<n ; i++){
        Vx += boids[i].dx;
        Vy += boids[i].dy;
        }

        Vx/=n;
        Vy/=n; // x and y coordinates of the center of mass

        for(int i = 0; i<n ; i++){

                boids[i].dx += (Vx - boids[i].dx)/8; // move the boids 0,1% of the way towards the centre
                boids[i].dy += (Vy - boids[i].dy)/8;  
        }


    //move_all_boids_to_new_positions
        for(int i = 0; i<n ; i++){

            boids[i].x += boids[i].dx  ;
            boids[i].y += boids[i].dy  ; 

            boids[i].dx = 0; 
            boids[i].dy = 0;
    }

    }

 

void draw_boids(std::vector<circle> & boids, int n){
        // Actual app
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    //ctx.maximize_window();

    const float WINDOW_MIN_X = -ctx.aspect_ratio();
    const float WINDOW_MAX_X = ctx.aspect_ratio();
    const float WINDOW_MIN_Y = -1;
    const float WINDOW_MAX_Y = 1;

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::AppleGreen);

        for (int i = 0; i<n ; i++){
        ctx.circle({boids[i].x, boids[i].y},
        p6::Radius{0.05f}
        );
    }
    update_position(boids, n, WINDOW_MIN_X, WINDOW_MAX_X, WINDOW_MIN_Y, WINDOW_MAX_Y);
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
    draw_boids(boids, n);

}