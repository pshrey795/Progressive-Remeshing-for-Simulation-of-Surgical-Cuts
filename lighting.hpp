#ifndef LIGHTING_HPP
#define LIGHTING_HPP

#include "common.hpp"
#include "gui.hpp"

#include <vector>

class Lighting {
public:
    void addLight(vec3 dir, vec3 color);
    void createDefault();
    void apply();
protected:
    struct Light {
        vec3 dir, color;
        Light (vec3 dir, vec3 color): dir(dir), color(color) {}
    };
    std::vector<Light> lights;
};

void Lighting::addLight(vec3 dir, vec3 color) {
    lights.push_back(Light(dir,color));
}

void Lighting::createDefault() {
    addLight(vec3(1,0.5,1), 0.8*vec3(1,1,1));
    addLight(vec3(-1,0.5,1), 0.4*vec3(1,1,1));
    addLight(vec3(-1,0.5,-1), 0.6*vec3(1,1,1));
    addLight(vec3(0,-1,0), 0.4*vec3(1,1,1));
}

void Lighting::apply() {
    for (int i = 0; i < lights.size(); i++) {
        Light l = lights[i];
        GLfloat pos[4] = {l.dir[0], l.dir[1], l.dir[2], 0};
        GLfloat color[4] = {l.color[0], l.color[1], l.color[2], 0};
        glLightfv(GL_LIGHT0+i, GL_POSITION, pos);
        glLightfv(GL_LIGHT0+i, GL_DIFFUSE, color);
        glLightfv(GL_LIGHT0+i, GL_SPECULAR, color);
        glEnable(GL_LIGHT0+i);
    }
}

#endif
