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
        vector<tuple<int,int,double,double>> list_of_springs;
        Matrix<float, Dynamic, Dynamic> M;
        Matrix<vec3, Dynamic, 1> x;
        Matrix<vec3, Dynamic, 1> v;
        Matrix<vec3, Dynamic, 1> f;
        Matrix<mat3, Dynamic, Dynamic> Jx;
        int n;
        int m;

        void setupGrid(int n,int m){
            //First add the particles
            //First row
            this->n = n;
            this->m = m;
            int num_x = n+1;
            int num_y = m+1;
            this->add_particle(new particle(0.8,vec3(-0.5,1.5,0),vec3(0,0,0),true));
            for(int i=1;i<n;i++){
                this->add_particle(new particle(0.8,vec3(-0.5+i*1.0/n,1.5,0),vec3(0,0,0),false));
            }
            this->add_particle(new particle(0.8,vec3(+0.5,1.5,0),vec3(0,0,0),true));

            //Other rows
            for(int j=1;j<m;j++){
                for(int i=0;i<=n;i++){
                    if(i==0 || i==n){
                        this->add_particle(new particle(0.8,vec3(-0.5+i*1.0/n,1.5,j*1.0/m),vec3(0,0,0),false));
                    }else{
                        this->add_particle(new particle(0.8,vec3(-0.5+i*1.0/n,1.5,j*1.0/m),vec3(0,0,0),false));
                    }
                    
                }
            }

            this->add_particle(new particle(0.8,vec3(-0.5,1.5,1.0),vec3(0,0,0),true));
            for(int i=1;i<n;i++){
                this->add_particle(new particle(0.8,vec3(-0.5+i*1.0/n,1.5,1.0),vec3(0,0,0),false));
            }
            this->add_particle(new particle(0.8,vec3(+0.5,1.5,1.0),vec3(0,0,0),true));

            //Adding springs
            for(int j=0;j<num_y;j++){
                for(int i=0;i<num_x;i++){
                    if(j==m){
                        if(i!=n){
                            //Horizontal structural spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,10,0.5/n,0.5/n);
                        }
                    }else{
                        if(i!=n){
                            //Horizontal structural spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+1,10,0.5/n,0.5/n);
                            //Right shear spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x+1,10,0.5/m,0.5/m);
                        }
                        if(i!=0){
                            //Left shear spring
                            this->add_spring((num_x)*j+i,(num_x)*j+i+num_x-1,10,0.5/m,0.5/m);
                        }
                        //Vertical structural spring
                        this->add_spring((num_x)*j+i,(num_x)*j+i+num_x,10,0.5/m,0.5/m);
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
        void update_particles(float dt,int id){
            resetJ();
            int num = particles.size();
            for(int i=0;i<num;i++){
                f(i) = particles[i]->mass*gravity + net_force(i);
            }

            //Update Jacobian
            for(int k=0;k<list_of_springs.size();k++){
                int i = get<0>(list_of_springs[k]);
                int j = get<1>(list_of_springs[k]);
                vec3 xij = particles[i]->pos - particles[j]->pos;
                double l = xij.norm();
                vec3 dij = xij.normalized();
                vec3 dji = -dij;
                double ks = get<2>(list_of_springs[k]);
                double l0 = get<3>(list_of_springs[k]);
                double m = maxd(0,1 - l0/l);

                mat3 d1 = dij * dij.transpose();
                mat3 d2 = dji * dji.transpose();
                mat3 Jii = -(ks * (m*(mat3::Identity() - d1) + d1));
                mat3 Jjj = -(ks * (m*(mat3::Identity() - d2) + d2));
                mat3 Jij = -Jii;
                mat3 Jji = -Jjj;

                Jx(i,i) += Jii;
                Jx(j,j) += Jjj;
                Jx(i,j) += Jij;
                Jx(j,i) += Jji;
            }

            if(id==0){
                //Update velocities(Forward Euler)
                v = v + (dt * matrix_mult(convert(M.inverse(),num),f,num));
            }else{
                //Update velocities(Backward Euler)
                Matrix<mat3,Dynamic,Dynamic> K;
                K = combine((explode(convert(M,num),3*num) - dt * dt * explode(Jx,3*num)).inverse(),num);
                v = v + (dt * matrix_mult(K,f + matrix_mult(Jx,v,num),num));
            }

            //Explicit clamping   
            for(int i=0;i<num;i++){
                if(particles[i]->isClamped){
                    v(i) = vec3(0,0,0);
                }
            }

            //dx = (v + dv) * dt
            x = x + (dt * v);          

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

        double maxd(double a,double b){
            if(a>b){
                return a;
            }else{
                return b;
            }
        }

    protected:
        void resetJ(){
            for(int i=0;i<particles.size();i++){
                for(int j=0;j<particles.size();j++){
                    Jx(i,j) = mat3::Zero();
                }
            }
        }
        void add_particle(particle* p){
            particles.push_back(p);
            vector<pair<int,spring*>> temp;
            springs.push_back(temp);
        }
        void add_spring(int i, int j, double k, double l, double l0){
            springs[i].push_back(make_pair(j,new spring{k,l,l0}));
            springs[j].push_back(make_pair(i,new spring{k,l,l0}));
            list_of_springs.push_back(make_tuple(i,j,k,l0));
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