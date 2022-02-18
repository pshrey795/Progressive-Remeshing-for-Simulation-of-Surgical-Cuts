#include "camera.hpp"
#include "draw.hpp"
#include "gui.hpp"
#include "lighting.hpp"
#include "text.hpp"

#include <bits/stdc++.h>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Text text;

float dt = 1/60.;
float t = 0;
float n = 202;
bool paused = false;
vec3 points[151][101];

bool isInside(int a,int b){
    int x = abs(a-75);
    int y = abs(b-50);
    return (sqrt(x*x + y*y))<20;
}

void drawStuff() {
    setColor(vec3(0.8,0.2,0.2));
    drawArrow(vec3(0,-0.5,0), vec3(0,1.5,0), 0.05);
    setPointSize(5);
    for(int i=0;i<150;i++){
        setColor(vec3(0.1,0.9,0.4));
        for(int j=0;j<29;j++){
            drawQuad(points[i][j],points[i][j+1],points[i+1][j+1],points[i+1][j]);
        }
        for(int j=29;j<70;j++){
            if(isInside(i,j)){
                setColor(vec3(0.2,0,0.7));
            }else{
                setColor(vec3(1,1,1));
            }
            drawQuad(points[i][j],points[i][j+1],points[i+1][j+1],points[i+1][j]);
        }
        setColor(vec3(1,0.45,0.01));
        for(int j=70;j<100;j++){
            drawQuad(points[i][j],points[i][j+1],points[i+1][j+1],points[i+1][j]);
        }
    }
}

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    setColor(vec3(0.7,0.7,0.7));
    drawQuad(vec3(-1,-0.5,-1), vec3(-1,-0.5,1), vec3(1,-0.5,1), vec3(1,-0.5,-1));
    drawStuff();
    setColor(vec3(0,0,0));
    text.draw("WASD and LShift/LCtrl to move camera", -0.9, 0.90);
    text.draw("Mouse to rotate view", -0.9, 0.85);
    text.draw("Space to play/pause animation", -0.9, 0.80);
}

void update(float dt) {
    t += dt;
    for(int x=0;x<151;x++){
        for(int y=0;y<101;y++){
            float u = (float)x/n;
            float v = (float)y/n;
            points[x][y] = vec3(u,v+0.5,(u * sin(6 * M_PI * (u + v - t)))/(20));
        }
    }
}

void keyPressed(int key) {
    // See http://www.glfw.org/docs/latest/group__keys.html for key codes
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

int main(int argc, char **argv) {
    window.create("Animation", 1024, 768);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(1,1.5,5), vec3(0,0.5,0));
    lighting.createDefault();
    text.initialize();

    //Initialise points
    

    while (!window.shouldClose()) {
        camera.processInput(window);
        if (!paused)
            update(dt);
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay();
        window.waitForNextFrame(dt);
    }
}
