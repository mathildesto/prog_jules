#include "p6/p6.h"
#include "boid.hpp"

float compute_distance(const boid boid1, const boid boid2 ){
        //distance = sqrt((x2 - x1)² + (y2 - y1)²)
        return sqrt(pow(boid2.x - boid1.x,2) + pow(boid2.y - boid1.y,2));
    };

void check_boundaries(param_boids &param, Window &window, boid &boid){
    if (boid.x < window.WINDOW_MIN_X) {
        boid.vx += param.turnfactor ; // reverse the x-component of the velocity
    }
    else if (boid.x > window.WINDOW_MAX_X) {
        boid.vx += -param.turnfactor;
    }
    if (boid.y < window.WINDOW_MIN_Y) {
        boid.vy += param.turnfactor;
    }
    else if (boid.y > window.WINDOW_MAX_Y) {
        boid.vy += -param.turnfactor;
    }
}

void check_speed(param_boids &param, Window &window, boid &boid){

    float speed = sqrt(boid.vx*boid.vx + boid.vy*boid.vy);

    if (speed < param.minspeed){
        boid.vx = (boid.vx/speed)*param.minspeed;
        boid.vy = (boid.vy/speed)*param.minspeed;
    }
    if (speed > param.maxspeed){
        boid.vx = (boid.vx/speed)*param.maxspeed;
        boid.vy = (boid.vy/speed)*param.maxspeed;
    }
}

void separation(std::vector<boid>& boids, param_boids &param, Window &window, boid &boid){
    float close_dx= 0;
    float close_dy= 0;

    for(int j = 0; j<boids.size() ; j++){

        if (boid != boids[j] ){

            float distance = compute_distance(boid, boids[j]);
        
            //Is squared distance less than the protected range?
            if (distance < param.protectedRange){

                //If so, calculate difference in x/y-coordinates to nearfield boid
                close_dx += boid.x - boids[j].x; 
                close_dy += boid.y - boids[j].y;
            }
        }
    }

    boid.vx = boid.vx + (close_dx*param.avoidfactor); // Add the avoidance contribution to velocity
    boid.vy = boid.vy + (close_dy*param.avoidfactor);

}

void cohesion(std::vector<boid>& boids, param_boids &param, Window &window, boid &boid){

    float xvel_avg= 0;
    float yvel_avg= 0;
    float neighboring_boids= 0;

    for(int j = 0; j<boids.size() ; j++){

        if (boid != boids[j] ){

            float distance = compute_distance(boid, boids[j]);
        
            //Is squared distance less than the visual range?
            if (distance < param.visualRange){

                    //Add other boid's x/y-coord and x/y vel to accumulator variables
                    xvel_avg += boids[j].vx;
                    yvel_avg += boids[j].vy;

                //Increment number of boids within visual range
                    neighboring_boids += 1; 
                }
            }
        }

    //If there were any boids in the visual range . . .            
    if (neighboring_boids > 0){ 

        //Divide accumulator variables by number of boids in visual range
        xvel_avg = xvel_avg/neighboring_boids;
        yvel_avg = yvel_avg/neighboring_boids;

        boid.vx = boid.vx + (xvel_avg - boid.vx)*param.matchingfactor; //Add the centering/matching contributions to velocity
        boid.vy = boid.vy  + (yvel_avg - boid.vy)*param.matchingfactor;
    }
}

void alignment(std::vector<boid>& boids, param_boids &param, Window &window, boid &boid){

    float xpos_avg = 0;
    float ypos_avg= 0;
    float neighboring_boids= 0;

    for(int j = 0; j<boids.size() ; j++){

        if (boid != boids[j] ){

            float distance = compute_distance(boid, boids[j]);
        
            //Is squared distance less than the protected range?
            if (distance < param.visualRange){

                //Add other boid's x/y-coord and x/y vel to accumulator variables
                xpos_avg += boids[j].x ;
                ypos_avg += boids[j].y ;

            //Increment number of boids within visual range
                neighboring_boids += 1; 
            }
    }
}

    //If there were any boids in the visual range . . .            
    if (neighboring_boids > 0){ 

        //Divide accumulator variables by number of boids in visual range
        xpos_avg = xpos_avg/neighboring_boids; 
        ypos_avg = ypos_avg/neighboring_boids;

        boid.vx = boid.vx + (xpos_avg - boid.x)*param.centeringfactor; //Add the centering/matching contributions to velocity
        boid.vy = boid.vy + (ypos_avg - boid.y)*param.centeringfactor;
    }
}


std::vector<boid> initialise_positions(int n){
    // Adds all the boids at a starting position. I put them all at random locations

    std::vector<boid> boids(n);
    for (int i = 0; i<n ; i++){
        boids[i] = boid();
    }
    return boids;
}

void update_position(std::vector<boid>& boids, int n, Window window, param_boids &param){
    
    // for each boid
    for(int i = 0; i<n ; i++){

        alignment(boids, param, window, boids[i]);
        
        cohesion (boids, param, window, boids[i]);

        separation(boids, param, window, boids[i]);

        check_boundaries(param, window, boids[i]);

        check_speed(param, window, boids[i]);

    //Update boid's position
        boids[i].x += boids[i].vx;
        boids[i].y += boids[i].vy;  
    }
}


void draw_boids(std::vector<boid> & boids, int n, p6::Context& ctx, param_boids &param){
        
        for (int i = 0; i<n ; i++){
        ctx.circle({boids[i].x, boids[i].y},
        p6::Radius{param.boidSize}
        );
    }
}


// void update_position(std::vector<boid>& boids, int n, Window window, param_boids &param){
    
//     // for each boid
//     for(int i = 0; i<n ; i++){

//         float xpos_avg = 0;
//         float ypos_avg= 0;
//         float xvel_avg= 0;
//         float yvel_avg= 0;
//         float neighboring_boids= 0;
//         float close_dx= 0;
//         float close_dy= 0;
        
//         // for each other boid
//         for(int j = 0; j<n ; j++){
            
//             if (j != i ){

//                 //Compute differences in x and y coordinates
//                 float dx = boids[i].x - boids[j].x;
//                 float dy = boids[i].y - boids[j].y;

//                 //Are both those differences less than the visual range?
//                 if (abs(dx)<param.visualRange && abs(dy)<param.visualRange){
                
//                     // If so, calculate the squared distance
//                     float squared_distance = dx*dx + dy*dy;
                
//                     //Is squared distance less than the protected range?
//                     if (squared_distance < pow(param.protectedRange,2)){

//                         //If so, calculate difference in x/y-coordinates to nearfield boid
//                         close_dx += boids[i].x - boids[j].x; 
//                         close_dy += boids[i].y - boids[j].y;
//                     }

//                     //If not in protected range, is the boid in the visual range?
//                     else if (squared_distance < pow(param.visualRange,2)){

//                         //Add other boid's x/y-coord and x/y vel to accumulator variables
//                         xpos_avg += boids[j].x ;
//                         ypos_avg += boids[j].y ;
//                         xvel_avg += boids[j].vx;
//                         yvel_avg += boids[j].vy;

//                     //Increment number of boids within visual range
//                         neighboring_boids += 1; 
//                     }
//                 }
//             }
//         }

//         //If there were any boids in the visual range . . .            
//         if (neighboring_boids > 0){ 

//             //Divide accumulator variables by number of boids in visual range
//             xpos_avg = xpos_avg/neighboring_boids; 
//             ypos_avg = ypos_avg/neighboring_boids;
//             xvel_avg = xvel_avg/neighboring_boids;
//             yvel_avg = yvel_avg/neighboring_boids;

//             // ############ RULE 1 & 2 : COHESION and ALIGNEMENT #######################
//             boids[i].vx = (boids[i].vx + 
//                    (xpos_avg - boids[i].x)*param.centeringfactor + 
//                    (xvel_avg - boids[i].vx)*param.matchingfactor); //Add the centering/matching contributions to velocity

//             boids[i].vy = (boids[i].vy + 
//                    (ypos_avg - boids[i].y)*param.centeringfactor + 
//                    (yvel_avg - boids[i].vy)*param.matchingfactor);
//         }

//         // ################ RULE 3 : SEPARATION ####################
//         boids[i].vx = boids[i].vx + (close_dx*param.avoidfactor); // Add the avoidance contribution to velocity
//         boids[i].vy = boids[i].vy + (close_dy*param.avoidfactor);

        
//         // ################# RULE 4 : check if the boid is outside the window boundaries ########################
//         check_boundaries(param, window, boids[i]);

//         // ############ CHECK SPEED ####################
//         check_speed(param, window, boids[i]);

//     //Update boid's position
//     boids[i].x += boids[i].vx;
//     boids[i].y += boids[i].vy;  
//     }
// }
