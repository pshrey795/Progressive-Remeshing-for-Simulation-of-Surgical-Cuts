#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "particle.hpp"
#include "draw.hpp"
#include<bits/stdc++.h>
using namespace std;
using namespace Eigen;

struct spring{
    double k;
    double l;
    double l0;
};

class particle_system {
    public:   
        vector<particle*> particles;
        vector<vector<pair<int,spring*>>> springs;
        Matrix<float, Dynamic, Dynamic> M;
        Matrix<vec3, Dynamic, 1> x;
        Matrix<vec3, Dynamic, 1> v;
        Matrix<vec3, Dynamic, 1> f;
        Matrix<vec3, Dynamic, Dynamic> Jx;
        int n;
        int m;

        void setupGrid(int n,int m){
            //First add the particles
            //First row
            this->n = n;
            this->m = m;
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
                    this->add_particle(new particle(1,vec3(-0.5+i*1.0/n,1.5,j*1.0/m),vec3(0,0,0),false));
                }
            }

            this->add_particle(new particle(1,vec3(-0.5,1.5,1.0),vec3(0,0,0),true));
            for(int i=1;i<n;i++){
                this->add_particle(new particle(1,vec3(-0.5+i*1.0/n,1.5,1.0),vec3(0,0,0),false));
            }
            this->add_particle(new particle(1,vec3(+0.5,1.5,1.0),vec3(0,0,0),true));

            //Adding springs
            for(int j=0;j<num_y;j++){
                for(int i=0;i<num_x;i++){
                    if(j==m){
                        if(i!=n){
                            //Horizontal structural spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,50,0.5/n,0.5/n);
                        }
                    }else{
                        if(i!=n){
                            //Horizontal structural spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,50,0.5/n,0.5/n);
                            //Right shear spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x+1,50,0.5/m,0.5/m);
                        }
                        if(i!=0){
                            //Left shear spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x-1,50,0.5/m,0.5/m);
                        }
                        //Vertical structural spring
                        this->add_spring((num_x)*j+i,(num_x)*j+i+num_x,200,0.5/m,0.5/m);
                    }
                }
            }
        }
        void initialise(){
            M.resize(particles.size(),particles.size());
            x.resize(particles.size());
            v.resize(particles.size());
            f.resize(particles.size());
            Jx.resize(particles.size(),particles.size());
            for(int i=0;i<particles.size();i++){
                x(i) = particles[i]->pos;
                v(i) = particles[i]->vel;
                vec3 force = vec3(0,0,0);
                f(i) = net_force(i);
                for(int j=0;j<particles.size();j++){
                    M(i,j) = 0;
                }
                M(i,i) = particles[i]->mass;
            }
        }
        void update_particles(float dt){
            for(int i=0;i<particles.size();i++){
                f(i) = particles[i]->mass*gravity + net_force(i);
            }

            //Forward Euler
            //Update velocities
            v = v + (dt * matrix_vector_mult(M.inverse(),f,particles.size()));

            //Explicit clamping   
            for(int i=0;i<particles.size();i++){
                if(particles[i]->isClamped){
                    v(i) = vec3(0,0,0);
                }
            }

            //dx = (v + dv) * dt
            x = x + (dt * v);


            //Backward Euler


            //Storing computed x,v,f values for rendering
            this->storeValues();
        }
        void drawCloth(){
            setPointSize(10);
            for(int j=0;j<m;j++){
                for(int i=0;i<n;i++){
                    setColor(particles[(n+1)*j+i]->color);
                    drawTri(particles[(n+1)*j+i]->pos,particles[(n+1)*j+i+1]->pos,particles[(n+1)*j+i+n+1]->pos);
                    drawTri(particles[(n+1)*j+i+n+1]->pos,particles[(n+1)*j+i+1]->pos,particles[(n+1)*j+i+n+2]->pos);
                }
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
        vec3 spring_force(int i,int j){
            vec3 xi = particles[i]->pos;
            vec3 xj = particles[springs[i][j].first]->pos;
            vec3 xij = (xj-xi).normalized();
            springs[i][j].second->l = (xj-xi).norm();
            vec3 fij = xij*(springs[i][j].second->k*(springs[i][j].second->l-springs[i][j].second->l0));
            return fij;
        }
        vec3 net_force(int i){
            vec3 force = vec3(0,0,0);
            for(int j=0;j<springs[i].size();j++){
                force += spring_force(i,j);
            }
            return force;
        }
        void storeValues(){
            for(int i=0;i<particles.size();i++){
                particles[i]->pos = x(i);
                particles[i]->vel = v(i);
                particles[i]->force = net_force(i);
            }
        }
};

#endif