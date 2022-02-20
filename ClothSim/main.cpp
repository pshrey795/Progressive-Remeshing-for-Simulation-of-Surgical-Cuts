#include "camera.hpp"
#include "draw.hpp"
#include "gui.hpp"
#include "lighting.hpp"
#include "text.hpp"
#include "particle_system.hpp"

#include <bits/stdc++.h>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Text text;

float dt = 1/60.0;
float t = 0;
bool paused = false;
particle_system psys;

void drawStuff() {
    //Poles to support cloth
    setColor(vec3(0,0,0));
    drawArrow(vec3(-1,0,0), vec3(0.5,1.5,0),0.05);
    drawArrow(vec3(1,0,0), vec3(-0.5,1.5,0),0.05);
    drawArrow(vec3(-1,0,1), vec3(0.5,1.5,0),0.05);
    drawArrow(vec3(1,0,1), vec3(-0.5,1.5,0),0.05);
    //Draw the cloth
    psys.drawCloth();
}

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    setColor(vec3(0.7,0.7,0.7));
    //drawQuad(vec3(-3,-0,-3), vec3(-3,0,3), vec3(3,0,3), vec3(3,0,-3));
    drawStuff();
    setColor(vec3(0,0,0));
    text.draw("WASD and LShift/LCtrl to move camera", -0.9, 0.90);
    text.draw("Mouse to rotate view", -0.9, 0.85);
    text.draw("Space to play/pause animation", -0.9, 0.80);
}

void update(float dt) {
    t += dt;
    psys.update_particles(dt,1);
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
    psys.setupGrid(5,5);
    psys.initialise();

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
