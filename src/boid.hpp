#pragma once
#include "p6/p6.h"

struct boid{
    float x, y; //position 

    float vx, vy; //velocity

    float size;

    boid() : x(p6::random::number(-1, 1)), y(p6::random::number(-1, 1)), size(0.02f) {}

    bool operator !=(boid &boid) const{
        if (x!=boid.x and y!=boid.y){
            return true;
        }
    }

};


struct param_boids{

    float boidSize = 0.02f;

    float visualRange = 0.8 ; 
    float protectedRange = 0.1 ; 

    float turnfactor = 0.005 ; 
    float centeringfactor = 0.0005;
    float avoidfactor =  0.04;
    float matchingfactor = 0.05;
    
    float maxspeed = 0.02;
    float minspeed = 0.009;
};

struct Window {
    // const float WINDOW_MIN_X = -ctx.aspect_ratio() + 0.1;
    // const float WINDOW_MAX_X = ctx.aspect_ratio() - 0.1;

    const float WINDOW_MIN_X = -1.5;
    const float WINDOW_MAX_X = 1.5;

    const float WINDOW_MIN_Y = -0.8;
    const float WINDOW_MAX_Y = 0.8;
};

float distance(const boid boid1, const boid boid2 );

std::vector<boid> initialise_positions(int n);

void check_boundaries(param_boids &param, Window &window, boid &boid);

void update_position(std::vector<boid>& boids, int n, Window window, param_boids &param);

void draw_boids(std::vector<boid> & boids, int n, p6::Context& ctx, param_boids &param);




