#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "particle.hpp"
#include "draw.hpp"
#include<bits/stdc++.h>
using namespace std;

struct spring{
    double k;
    double l;
    double l0;
};

class particle_system {
    public:   
        vector<particle*> particles;
        vector<vector<pair<int,spring*>>> springs;
        
        void setupGrid(int n,int m){
            //First add the particles
            //First row
            int num_x = n+1;
            int num_y = m+1;
            this->add_particle(new particle(1,vec3(-0.5,1.5,0),vec3(0,0,0),true));
            for(int i=1;i<n;i++){
                this->add_particle(new particle(1,vec3(-0.5+i*1.0/n,1.5,0),vec3(0,0,0),false));
            }
            this->add_particle(new particle(1,vec3(+0.5,1.5,0),vec3(0,0,0),true));

            //Other rows
            for(int j=1;j<m;j++){
                for(int i=0;i<=n;i++){
                    this->add_particle(new particle(1,vec3(-0.5+i*1.0/n,1.5-j*1.0/m,0),vec3(0,0,0),false));
                }
            }

            this->add_particle(new particle(1,vec3(-0.5,0.5,0),vec3(0,0,0),false));
            for(int i=1;i<n;i++){
                this->add_particle(new particle(1,vec3(-0.5+i*1.0/n,0.5,0),vec3(0,0,0),false));
            }
            this->add_particle(new particle(1,vec3(+0.5,0.5,0),vec3(0,0,0),false));

            //Second: Adding springs
            for(int j=0;j<num_y;j++){
                for(int i=0;i<num_x;i++){
                    if(j==m){
                        if(i!=n){
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,20000,0.5/n,0.5/n);
                        }
                    }else{
                        if(i!=n){
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,20000,0.5/n,0.5/n);
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x,20000,0.5/m,0.5/m);
                        }else{
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x,20000,0.5/m,0.5/m);
                        }
                    }
                }
            }
        }
        void update_springs(){
            for(int i=0;i<particles.size();i++){
                for(int j=0;j<springs[i].size();j++){
                    double dx = particles[i]->pos[0] - particles[springs[i][j].first]->pos[0];
                    double dy = particles[i]->pos[1] - particles[springs[i][j].first]->pos[1];
                    double dz = particles[i]->pos[2] - particles[springs[i][j].first]->pos[2];
                    double d = sqrt(dx*dx + dy*dy + dz*dz);
                    springs[i][j].second->l = d;
                }
            }
        }
        void update_particles(float dt){
            update_springs();
            for(int i=0;i<particles.size();i++){
                vec3 force = vec3(0,0,0);
                for (int j=0;j<springs[i].size();j++){
                    vec3 spring_force = (particles[springs[i][j].first]->pos - particles[i]->pos).normalized();
                    spring_force *= (springs[i][j].second->k * (springs[i][j].second->l - springs[i][j].second->l0));
                    force += spring_force;
                }
                particles[i]->set_force(force);
                particles[i]->update_particle(dt);
            }
        }
        void drawCloth(){
            setPointSize(10);
            for(int i=0;i<particles.size();i++){
                setColor(particles[i]->color);
                drawPoint(particles[i]->pos);
            }
        }

    protected:
        void add_particle(particle* p){
            particles.push_back(p);
            vector<pair<int,spring*>> temp;
            springs.push_back(temp);
        }
        void add_spring(int i, int j, double k, double l, double l0){
            springs[i].push_back(make_pair(j,new spring{k,l,l0}));
            springs[j].push_back(make_pair(i,new spring{k,l,l0}));
        }
};

#endif