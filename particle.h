#pragma once
#include <bits/stdc++.h>
using namespace std;
#include "vec3d.h"

vec3d gravity = vec3d(0,-9.8,0);

class particle{

private:
    float mass;
    point3d pos;
    vec3d vel;
    vec3d force;
    bool isClamped;
    color3d color;

public:

    // constructor to initialize the particle with mass, position and velocity
    particle(float m, point3d p, vec3d v=vec3d(0,0,0), bool is_clamped=false, color3d c = color3d(1,0,0)){
        this->mass = m;
        this->pos = p;
        this->vel = v;
        this->isClamped = is_clamped;
        this->force = vec3d(0,0,0);
        this->color = c;
    }

    // getter functions
    float get_mass(){return this->mass;}
    point3d get_pos(){return this->pos;}
    vec3d get_vel(){return this->vel;}
    bool get_isClamped(){return this->isClamped;}
    color3d get_color(){return this->color;}

    // setter functions
    void set_pos(point3d p){this->pos = p;}
    void set_vel(vec3d v){this->vel = v;}
    void set_force(vec3d f){this->force = f + this->mass*gravity;}

    // make a function to update the particle
    void update_particle(float dt){
        if(!this->isClamped){
            this->vel += this->force*dt/this->mass;
            this->pos += this->vel*dt;
        }
    }



};