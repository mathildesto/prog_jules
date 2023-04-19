#pragma once
#include "p6/p6.h"

struct Boid{

    float size;
    std::vector<double> position;
    std::vector<double> velocity;

    Boid() {
    position = { p6::random::number(-1, 1), p6::random::number(-1, 1) };
    velocity = { 0, 0 };
    }

    Boid(double x, double y, double vx, double vy) {
        position = { x, y };
        velocity = { vx, vy };
    }

    bool operator ==(const Boid &boid) const {
        return position == boid.position && velocity == boid.velocity;
    }

    void add_velocity(){
        position[0] += velocity[0];
        position[1] += velocity[1];
    }
};


struct ParamBoids{
    int numberOfBoids = 50;

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

float distance(const Boid boid1, const Boid boid2 );

std::vector<Boid> initialise_positions(int n);


void keep_inside_boundaries(ParamBoids &param, Window &window, Boid &boid);
void limit_speed(ParamBoids &param, Boid &boid);
void separation(std::vector<Boid>& boids, ParamBoids &param, Boid &boid);
void cohesion(std::vector<Boid>& boids, ParamBoids &param, Boid &boid);
void alignment(std::vector<Boid>& boids, ParamBoids &param, Boid &boid);

void update_position(std::vector<Boid>& boids, Window window, ParamBoids &param);

void draw_boids(const std::vector<Boid>& boids, p6::Context& ctx, const ParamBoids& param);




