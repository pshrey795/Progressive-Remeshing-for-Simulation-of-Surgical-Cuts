#include<bits/stdc++.h>
#include<GL/glut.h>       

using namespace std;
typedef GLint vertex3[3];

GLenum errorCheck(){
    GLenum error = glGetError();
    if(error != GL_NO_ERROR){
        cout << "Error: " << error << endl;
        return error;
    }
    return GL_NO_ERROR;
}

void init(){
    glClearColor(1.0,1.0,1.0,0.0);  //Alpha
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0.0,500.0,0.0,500.0);
}

void lineSegment(void){
    glClear(GL_COLOR_BUFFER_BIT);
    glShadeModel(GL_SMOOTH);
    glLineWidth(10.0);
    glBegin(GL_LINES);
        glColor3f(0.0f,0.0f,1.0f);
        glVertex2i(0,0);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex2i(500,500);
    glEnd();
    glFlush();
}

void triangles(void){
    glClear(GL_COLOR_BUFFER_BIT);
    glShadeModel(GL_SMOOTH);
    glBegin(GL_TRIANGLES);
        glColor3f(1.0f,0.0f,0.0f);
        glVertex2i(0,0);
        //glColor3f(0.0f,1.0f,0.0f);
        glVertex2i(500,0);
        //glColor3f(0.0f,0.0f,1.0f);
        glVertex2i(0,500);
    glEnd();
    glFlush();
}

void points(void){
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f (1.0, 0.0, 0.0);
    glPointSize(10.0);
    glBegin (GL_POINTS);
        glVertex2i (50, 100);
        glPointSize (20.0);
        glColor3f (0.0, 1.0, 0.0);
        glVertex2i (75, 150);
        glPointSize (30.0);
        glColor3f (0.0, 0.0, 1.0);
        glVertex2i (100, 200);
    glEnd ( );
    glFlush ( );
}

void cube(void){
    vertex3 pt[8] = {
        {0,0,0},
        {0,100,0},
        {100,0,0},
        {100,100,0},
        {0,0,100},
        {0,100,100},
        {100,0,100},
        {100,100,100}
    };
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.0f,0.4f,0.2f);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3,GL_INT,0,pt);
    GLubyte vertIndex[] = {6, 2, 3, 7, 5, 1, 0, 4, 7, 3, 1, 5,4, 0, 2, 6, 2, 0, 1, 3, 7, 5, 4, 6};
    glDrawElements(GL_QUADS,24,GL_UNSIGNED_BYTE,vertIndex);
    glDisableClientState(GL_VERTEX_ARRAY);
    glFlush();
}

void testDisplayList(void){
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    GLuint regHex = glGenLists(1);
    GLint x,y,k;
    GLdouble theta;
    glNewList(regHex,GL_COMPILE);
        glBegin(GL_POLYGON);
            for(k=0;k<6;k++){
                theta = TWO_PI * k / 6.0;
                x = 200 + 150 * cos(theta);
                y = 200 + 150 * sin(theta);
                glVertex2i(x,y);
            }
        glEnd();    
    glEndList();
    glCallList(regHex);
    glFlush();
}

int main(int argc, char** argv){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowPosition(1000,1000);
    glutInitWindowSize(1000,1000);
    glutCreateWindow("First Window");

    init();
    glutDisplayFunc(points);
    //glutDisplayFunc(lineSegment);
    //glutDisplayFunc(cube);
    //glutDisplayFunc(testDisplayList);
    //glutDisplayFunc(triangles);
    glutMainLoop();

    return 0;
}