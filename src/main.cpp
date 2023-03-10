#include <cmath>
#include <cstdlib>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>

struct circle{
    float x, y; //position 

    float vx, vy; //velocity

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

void update_position(std::vector<circle>& boids, int n, const float WINDOW_MIN_X, const float WINDOW_MAX_X, const float WINDOW_MIN_Y, const float WINDOW_MAX_Y, float turnfactor, float visual_range,float protectedRange,float centeringfactor,float avoidfactor,float matchingfactor,float maxspeed,float minspeed){
    
    // for each boid
    for(int i = 0; i<n ; i++){

        float xpos_avg = 0;
        float ypos_avg= 0;
        float xvel_avg= 0;
        float yvel_avg= 0;
        float neighboring_boids= 0;
        float close_dx= 0;
        float close_dy= 0;
        
        // for each other boid
        for(int j = 0; j<n ; j++){
            
            if (j != i ){

                //Compute differences in x and y coordinates
                float dx = boids[i].x - boids[j].x;
                float dy = boids[i].y - boids[j].y;

                //Are both those differences less than the visual range?
                if (abs(dx)<visual_range && abs(dy)<visual_range){
                
                    // If so, calculate the squared distance
                    float squared_distance = dx*dx + dy*dy;
                
                    //Is squared distance less than the protected range?
                    if (squared_distance < pow(protectedRange,2)){

                        //If so, calculate difference in x/y-coordinates to nearfield boid
                        close_dx += boids[i].x - boids[j].x; 
                        close_dy += boids[i].y - boids[j].y;
                    }

                    //If not in protected range, is the boid in the visual range?
                    else if (squared_distance < pow(visual_range,2)){

                        //Add other boid's x/y-coord and x/y vel to accumulator variables
                        xpos_avg += boids[j].x ;
                        ypos_avg += boids[j].y ;
                        xvel_avg += boids[j].vx;
                        yvel_avg += boids[j].vy;

                    //Increment number of boids within visual range
                        neighboring_boids += 1; 
                    }
                }
            }
        }

        //If there were any boids in the visual range . . .            
        if (neighboring_boids > 0){ 

            //Divide accumulator variables by number of boids in visual range
            xpos_avg = xpos_avg/neighboring_boids; 
            ypos_avg = ypos_avg/neighboring_boids;
            xvel_avg = xvel_avg/neighboring_boids;
            yvel_avg = yvel_avg/neighboring_boids;

            // ############ RULE 1 & 2 : COHESION and ALIGNEMENT #######################
            boids[i].vx = (boids[i].vx + 
                   (xpos_avg - boids[i].x)*centeringfactor + 
                   (xvel_avg - boids[i].vx)*matchingfactor); //Add the centering/matching contributions to velocity

            boids[i].vy = (boids[i].vy + 
                   (ypos_avg - boids[i].y)*centeringfactor + 
                   (yvel_avg - boids[i].vy)*matchingfactor);
        }

        // ################ RULE 3 : SEPARATION ####################
        boids[i].vx = boids[i].vx + (close_dx*avoidfactor); // Add the avoidance contribution to velocity
        boids[i].vy = boids[i].vy + (close_dy*avoidfactor);

        
        // ################# RULE 4 : check if the boid is outside the window boundaries ########################
        if (boids[i].x < WINDOW_MIN_X) {
            boids[i].vx += turnfactor ; // reverse the x-component of the velocity
        }
        else if (boids[i].x > WINDOW_MAX_X) {
            boids[i].vx += -turnfactor;
        }
        if (boids[i].y < WINDOW_MIN_Y) {
            boids[i].vy += turnfactor;
        }
        else if (boids[i].y > WINDOW_MAX_Y) {
            boids[i].vy += -turnfactor;
        }

    // ############ CHECK SPEED ####################
    float speed = sqrt(boids[i].vx*boids[i].vx + boids[i].vy*boids[i].vy);

    //Enforce min and max speeds
    if (speed < minspeed){
        boids[i].vx = (boids[i].vx/speed)*minspeed;
        boids[i].vy = (boids[i].vy/speed)*minspeed;
    }
    if (speed > maxspeed){
        boids[i].vx = (boids[i].vx/speed)*maxspeed;
        boids[i].vy = (boids[i].vy/speed)*maxspeed;
    }

    //Update boid's position
    boids[i].x = boids[i].x + boids[i].vx;
    boids[i].y = boids[i].y + boids[i].vy;  
    }
}

void draw_boids(std::vector<circle> & boids, int n){
        // Actual app
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    //ctx.maximize_window();

    const float WINDOW_MIN_X = -ctx.aspect_ratio() + 0.1;
    const float WINDOW_MAX_X = ctx.aspect_ratio() - 0.1;
    const float WINDOW_MIN_Y = -0.9;
    const float WINDOW_MAX_Y = 0.9;

    float turnfactor = 0.005 ; 
    float visualRange = 0.8 ; 
    float protectedRange = 0.2 ; 
    float centeringfactor = 0.0005;
    float avoidfactor =  0.05;
    float matchingfactor = 0.05;
    float maxspeed = 0.02;
    float minspeed = 0.009;

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::Pink);

        for (int i = 0; i<n ; i++){
        ctx.circle({boids[i].x, boids[i].y},
        p6::Radius{0.08f}
        );
    }
    update_position(boids, n, WINDOW_MIN_X, WINDOW_MAX_X, WINDOW_MIN_Y, WINDOW_MAX_Y, turnfactor, visualRange, protectedRange, centeringfactor, avoidfactor, matchingfactor, maxspeed, minspeed);
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
    std::vector<circle> boids = initialise_positions(n);
    draw_boids(boids, n);

}