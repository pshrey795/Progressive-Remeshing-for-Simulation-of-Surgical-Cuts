#ifndef DRAW_HPP
#define DRAW_HPP

#include <eigen3/Eigen/Dense>

#if __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

typedef Eigen::Vector3f vec3;

void clear(vec3 c);
void setColor(vec3 c);
void setPointSize(float s);
void drawPoint(vec3 x);
void setLineWidth(float w);
void drawLine(vec3 x0, vec3 x1);
void drawTri(vec3 x0, vec3 x1, vec3 x2);
void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3);
void drawArrow(vec3 base, vec3 vector, float thick);

void clear(vec3 c) {
    glClearColor(c[0], c[1], c[2], 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void setColor(vec3 c) {
    glColor3f(c[0], c[1], c[2]);
}

void setPointSize(float s) {
    glPointSize(s);
}

void drawPoint(vec3 x) {
    glBegin(GL_POINTS);
    glVertex3f(x[0], x[1], x[2]);
    glEnd();
}

void setLineWidth(float w) {
    glLineWidth(w);
}

void drawLine(vec3 x0, vec3 x1) {
    glBegin(GL_LINES);
    glVertex3f(x0[0], x0[1], x0[2]);
    glVertex3f(x1[0], x1[1], x1[2]);
    glEnd();
}

void drawTri(vec3 x0, vec3 x1, vec3 x2) {
    vec3 n = (x1-x0).cross(x2-x0).normalized();
    glBegin(GL_TRIANGLES);
    glNormal3f(n[0], n[1], n[2]);
    glVertex3f(x0[0], x0[1], x0[2]);
    glVertex3f(x1[0], x1[1], x1[2]);
    glVertex3f(x2[0], x2[1], x2[2]);
    glEnd();
}

void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3) {
    vec3 n = ((x1-x0).cross(x2-x0) + (x2-x0).cross(x3-x0)).normalized();
    glBegin(GL_QUADS);
    glNormal3f(n[0], n[1], n[2]);
    glVertex3f(x0[0], x0[1], x0[2]);
    glVertex3f(x1[0], x1[1], x1[2]);
    glVertex3f(x2[0], x2[1], x2[2]);
    glVertex3f(x3[0], x3[1], x3[2]);
    glEnd();
}

void drawArrow(vec3 base, vec3 vector, float thick) {
    float vnorm = vector.norm();
    if (vnorm == 0)
        return;
    glPushMatrix();
    glTranslatef(base[0],base[1],base[2]);
    vec3 vhat = vector/vnorm;
    vec3 w = vec3(0,0,1).cross(vhat);
    float angle = acos(vhat[2]);
    if (w.norm() > 0)
        glRotatef(angle*180/M_PI, w[0],w[1],w[2]);
    GLUquadric* quadric = gluNewQuadric();
    gluCylinder(quadric, 0,thick/3, 0, 30,1);
    gluCylinder(quadric, thick/3,thick/3, vnorm-3*thick, 30,1);
    glTranslatef(0,0,vnorm-3*thick);
    gluCylinder(quadric, thick/3,thick, 0, 30,1);
    gluCylinder(quadric, thick,0, 3*thick, 30,1);
    gluDeleteQuadric(quadric);
    glPopMatrix();
}

#endif
