#include <bits/stdc++.h>
using namespace std;
#include "sop.h"
#include<GL/glut.h>

sop* s;

void init(){
    glClearColor(1.0,1.0,1.0,0.0);  //Alpha
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0.0,10.0,0.0,10.0);
}


void simulate(void){
    while(true){
        glClear(GL_COLOR_BUFFER_BIT);
        s->update_springs();
        s->update_particles(0.001);
        s->show();
    }  
}


int main(int argc, char** argv){

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowPosition(1000,1000);
    glutInitWindowSize(1000,1000);
    glutCreateWindow("First Window");
    init();

    s = new sop();
    s->add_particle(new particle(1,vec3d(5,10,0),vec3d(0,0,0), true));
    s->add_particle(new particle(1,vec3d(5,8,0),vec3d(0,0,0)));
    s->add_particle(new particle(1,vec3d(6,8,0),vec3d(0,0,0)));
    s->add_particle(new particle(1,vec3d(6,10,0),vec3d(0,0,0),true));
    s->add_spring(0,1,10,2,2);
    s->add_spring(1,2,10,1,1);
    s->add_spring(2,3,10,2,2);

    glutDisplayFunc(simulate);
    glutMainLoop();

    return 0;
}