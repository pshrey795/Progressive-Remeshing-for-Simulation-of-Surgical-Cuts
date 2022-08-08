#include "camera.hpp"
#include "draw.hpp"
#include "gui.hpp"
#include "lighting.hpp"
#include "mesh.hpp"

#include <bits/stdc++.h>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Mesh mesh;

float dt = 1/60.0;
float t = 0;
bool paused = false;

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    setColor(vec3(0.7,0.7,0.7));
    mesh.renderMesh();
    setColor(vec3(0,0,0));
}

void update(float dt) {
    t += dt;
}

void keyPressed(int key) {
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

int main(int argc, char **argv) {
    window.create("Test Window", 1920, 1080);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(12.5,12.5,50), vec3(12.5,12.5,0));
    lighting.createDefault();

    while (!window.shouldClose()) {
        camera.processInput(window);
        mesh.processInput(window);
        if (!paused){
            update(dt);
            mesh.update(dt);
        }
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay();
        window.waitForNextFrame(dt);
    }
}
