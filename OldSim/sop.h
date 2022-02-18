#pragma once
#include <bits/stdc++.h>
using namespace std;
#include "particle.h"
#include<GL/glut.h>   

class sop{

private:

    // we are assuming particles are indexed according to the below vector
    vector <particle*> particles;

    struct spring{
        double k;
        double l;
        double l0;
    };

    vector<vector<pair<int,spring*>>> springs;

public:

    // make a function to add a particle
    void add_particle(particle* p){
        particles.push_back(p);
        vector<pair<int,spring*>> temp;
        springs.push_back(temp);
    }

    // make a function to add a spring
    void add_spring(int i, int j, double k, double l, double l0){
        springs[i].push_back(make_pair(j,new spring{k,l,l0}));
        springs[j].push_back(make_pair(i,new spring{k,l,l0}));
    }

    // make a function to update springs
    void update_springs(){
        for(int i=0;i<particles.size();i++){
            for(int j=0;j<springs[i].size();j++){
                double dx = particles[i]->get_pos().x() - particles[springs[i][j].first]->get_pos().x();
                double dy = particles[i]->get_pos().y() - particles[springs[i][j].first]->get_pos().y();
                double dz = particles[i]->get_pos().z() - particles[springs[i][j].first]->get_pos().z();
                double d = sqrt(dx*dx + dy*dy + dz*dz);
                springs[i][j].second->l = d;
            }
        }
    }

    // make a function to update particles
    void update_particles(float dt){
        this->update_springs();
        for(int i=0;i<particles.size();i++){
            vec3d force = vec3d(0,0,0);
            for (int j=0;j<springs[i].size();j++){
                vec3d spring_force = unit_vec( particles[springs[i][j].first]->get_pos() - particles[i]->get_pos() );
                spring_force *= (springs[i][j].second->k * (springs[i][j].second->l - springs[i][j].second->l0));
                force += spring_force;
            }
            particles[i]->set_force(force);
            particles[i]->update_particle(dt);
        }
    }

    void show(){
        glColor3f(1,0,0);
        glPointSize(25);
        glBegin(GL_POINTS);
        for (int i=0;i<particles.size();i++){
            glVertex2f(particles[i]->get_pos().x(),particles[i]->get_pos().y());
        }
        glEnd();
        for (int i=0;i<particles.size();i++){
            for (int j=0;j<springs[i].size();j++){
                glBegin(GL_LINES);
                glColor3f(0,0,0);
                glLineWidth(5);
                glVertex2f(particles[i]->get_pos().x(),particles[i]->get_pos().y());
                glVertex2f(particles[springs[i][j].first]->get_pos().x(),particles[springs[i][j].first]->get_pos().y());
                glEnd();
            }
        }
        glFlush();
    }


};