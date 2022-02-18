#ifndef DRAW_HPP
#define DRAW_HPP

#include "common.hpp"

void clear(vec3 c);
void setColor(vec3 c);
void setPointSize(float s);
void drawPoint(vec3 x);
void setLineWidth(float w);
void drawLine(vec3 x0, vec3 x1);
void drawArrow(vec3 base, vec3 vector, float thick);
void drawAxes();

void drawTri(vec3 x0, vec3 x1, vec3 x2);
void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3);

// for smooth shading, provide surface normal at each vertex
void drawTri(vec3 x0, vec3 x1, vec3 x2,
             vec3 n0, vec3 n1, vec3 n2);
void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3,
              vec3 n0, vec3 n1, vec3 n2, vec3 n3);

void drawBox(vec3 xmin, vec3 xmax);
void drawSphere(vec3 center, float radius);

// -=-=-=-=-=-=-=- //

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

void drawTri(vec3 x0, vec3 x1, vec3 x2, vec3 n0, vec3 n1, vec3 n2) {
    glBegin(GL_TRIANGLES);
    glNormal3f(n0[0], n0[1], n0[2]);
    glVertex3f(x0[0], x0[1], x0[2]);
    glNormal3f(n1[0], n1[1], n1[2]);
    glVertex3f(x1[0], x1[1], x1[2]);
    glNormal3f(n2[0], n2[1], n2[2]);
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

void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3,
              vec3 n0, vec3 n1, vec3 n2, vec3 n3) {
    glBegin(GL_QUADS);
    glNormal3f(n0[0], n0[1], n0[2]);
    glVertex3f(x0[0], x0[1], x0[2]);
    glNormal3f(n1[0], n1[1], n1[2]);
    glVertex3f(x1[0], x1[1], x1[2]);
    glNormal3f(n2[0], n2[1], n2[2]);
    glVertex3f(x2[0], x2[1], x2[2]);
    glNormal3f(n3[0], n3[1], n3[2]);
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

void drawAxes() {
    setColor(vec3(0.8,0.2,0.2));
    drawArrow(vec3(0,0,0), vec3(1,0,0), 0.05);
    setColor(vec3(0.2,0.6,0.2));
    drawArrow(vec3(0,0,0), vec3(0,1,0), 0.05);
    setColor(vec3(0.2,0.2,0.8));
    drawArrow(vec3(0,0,0), vec3(0,0,1), 0.05);
}

void drawBox(vec3 xmin, vec3 xmax) {
    float x0 = xmin[0], x1 = xmax[0];
    float y0 = xmin[0], y1 = xmax[0];
    float z0 = xmin[0], z1 = xmax[0];
    drawQuad(vec3(x0,y0,z0), vec3(x0,y1,z0), vec3(x0,y1,z1), vec3(x0,y0,z1));
    drawQuad(vec3(x1,y0,z0), vec3(x1,y1,z0), vec3(x1,y1,z1), vec3(x1,y0,z1));
    drawQuad(vec3(x0,y0,z0), vec3(x0,y0,z1), vec3(x1,y0,z1), vec3(x1,y0,z0));
    drawQuad(vec3(x0,y1,z0), vec3(x0,y1,z1), vec3(x1,y1,z1), vec3(x1,y1,z0));
    drawQuad(vec3(x0,y0,z0), vec3(x1,y0,z0), vec3(x1,y1,z0), vec3(x0,y1,z0));
    drawQuad(vec3(x0,y0,z1), vec3(x1,y0,z1), vec3(x1,y1,z1), vec3(x0,y1,z1));
}

void drawSphere(vec3 center, float radius) {
    glPushMatrix();
    glTranslatef(center[0],center[1],center[2]);
    GLUquadric* quadric = gluNewQuadric();
    gluSphere(quadric, radius, 30,30);
    gluDeleteQuadric(quadric);
    glPopMatrix();
}
    
#endif
