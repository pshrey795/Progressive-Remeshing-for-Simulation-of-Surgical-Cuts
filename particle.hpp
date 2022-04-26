#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "common.hpp"

vec3 gravity = vec3(0,-9.8,0);

class particle{
    public:
        float mass;
        vec3 pos;
        vec3 vel;
        vec3 force;
        vec3 color;
        bool isClamped;
        particle(float m, vec3 p, vec3 v=vec3(0,0,0), bool is_clamped=false, vec3 c = vec3(0.9,0.5,0.5)){
            this->mass = m;
            this->pos = p;
            this->vel = v;
            this->isClamped = is_clamped;
            this->force = vec3(0,0,0);
            this->color = c;
        }
        void set_force(vec3 force);
        void update_particle(float dt);
};

void particle::set_force(vec3 force){
    this->force = force + this->mass*gravity;
}

void particle::update_particle(float dt){
    if(!this->isClamped){
        this->vel += this->force*dt/this->mass;
        this->pos += this->vel*dt;
    }
}

#endif