#include <cmath>
#include <cstdlib>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include<vector>

struct boid{
    float x, y; //position 

    float vx, vy; //velocity

    float size;




    boid() : x(p6::random::number(-1, 1)), y(p6::random::number(-1, 1)), size(0.04f) {}
    //boid(float x, float y) : x(x), y(y) {}

};

   float distance(const boid boid1, const boid boid2 ){
        //distance = sqrt((x2 - x1)² + (y2 - y1)²)
        return sqrt(pow(boid2.x - boid1.x,2) + pow(boid2.y - boid1.y,2));
    };

struct Window {
    // const float WINDOW_MIN_X = -ctx.aspect_ratio() + 0.1;
    // const float WINDOW_MAX_X = ctx.aspect_ratio() - 0.1;

    const float WINDOW_MIN_X = -1.5;
    const float WINDOW_MAX_X = 1.5;

    const float WINDOW_MIN_Y = -0.8;
    const float WINDOW_MAX_Y = 0.8;
};

struct param_boids{

    float visualRange = 0.8 ; 
    float protectedRange = 0.1 ; 

    float turnfactor = 0.005 ; 
    float centeringfactor = 0.0005;
    float avoidfactor =  0.04;
    float matchingfactor = 0.05;
    
    float maxspeed = 0.02;
    float minspeed = 0.009;
};



    std::vector<boid> initialise_positions(int n){
    // Adds all the boids at a starting position. I put them all at random locations

    std::vector<boid> boids(n);

    for (int i = 0; i<n ; i++){
        boids[i] = boid();
    }

    return boids;
}

void update_position(std::vector<boid>& boids, int n, Window window, param_boids param){
    
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
                if (abs(dx)<param.visualRange && abs(dy)<param.visualRange){
                
                    // If so, calculate the squared distance
                    float squared_distance = dx*dx + dy*dy;
                
                    //Is squared distance less than the protected range?
                    if (squared_distance < pow(param.protectedRange,2)){

                        //If so, calculate difference in x/y-coordinates to nearfield boid
                        close_dx += boids[i].x - boids[j].x; 
                        close_dy += boids[i].y - boids[j].y;
                    }

                    //If not in protected range, is the boid in the visual range?
                    else if (squared_distance < pow(param.visualRange,2)){

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
                   (xpos_avg - boids[i].x)*param.centeringfactor + 
                   (xvel_avg - boids[i].vx)*param.matchingfactor); //Add the centering/matching contributions to velocity

            boids[i].vy = (boids[i].vy + 
                   (ypos_avg - boids[i].y)*param.centeringfactor + 
                   (yvel_avg - boids[i].vy)*param.matchingfactor);
        }

        // ################ RULE 3 : SEPARATION ####################
        boids[i].vx = boids[i].vx + (close_dx*param.avoidfactor); // Add the avoidance contribution to velocity
        boids[i].vy = boids[i].vy + (close_dy*param.avoidfactor);

        
        // ################# RULE 4 : check if the boid is outside the window boundaries ########################
        if (boids[i].x < window.WINDOW_MIN_X) {
            boids[i].vx += param.turnfactor ; // reverse the x-component of the velocity
        }
        else if (boids[i].x > window.WINDOW_MAX_X) {
            boids[i].vx += -param.turnfactor;
        }
        if (boids[i].y < window.WINDOW_MIN_Y) {
            boids[i].vy += param.turnfactor;
        }
        else if (boids[i].y > window.WINDOW_MAX_Y) {
            boids[i].vy += -param.turnfactor;
        }

    // ############ CHECK SPEED ####################
    float speed = sqrt(boids[i].vx*boids[i].vx + boids[i].vy*boids[i].vy);

    //Enforce min and max speeds
    if (speed < param.minspeed){
        boids[i].vx = (boids[i].vx/speed)*param.minspeed;
        boids[i].vy = (boids[i].vy/speed)*param.minspeed;
    }
    if (speed > param.maxspeed){
        boids[i].vx = (boids[i].vx/speed)*param.maxspeed;
        boids[i].vy = (boids[i].vy/speed)*param.maxspeed;
    }

    //Update boid's position
    boids[i].x += boids[i].vx;
    boids[i].y += boids[i].vy;  
    }
}


void draw_boids(std::vector<boid> & boids, int n, p6::Context& ctx){
        
        for (int i = 0; i<n ; i++){
        ctx.circle({boids[i].x, boids[i].y},
        p6::Radius{boids[i].size}
        );
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



    int n = 100; //nombre de boids
    std::vector<boid> boids = initialise_positions(n);

     // Actual app
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::Pink);

        draw_boids(boids, n, ctx);

        update_position(boids, n, Window(), param_boids() );
    };
        // Should be done last. It starts the infinite loop.
    ctx.start();

}