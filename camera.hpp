#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "common.hpp"
#include "gui.hpp"

class Camera {
public:
    vec3 target;
    float yaw, pitch, dist;
    float fovy, moveSpeed, zoomSpeed, turnSpeed;
    float lastInputTime;
    vec2 lastMousePos;
    Camera();
    void lookAt(vec3 eye, vec3 target);
    void processInput(Window &window);
    void apply(Window &window);
protected:
    vec3 getForwardVector();
    vec3 getRightVector();
    vec3 getUpVector();
};

Camera::Camera() :
    target(vec3(0,0,0)),
    yaw(0), pitch(0), dist(2),
    fovy(M_PI/3), moveSpeed(1), zoomSpeed(1), turnSpeed(0.2),
    lastInputTime(glfwGetTime()) {
}

void Camera::lookAt(vec3 eye, vec3 target) {
    this->target = target;
    this->dist = (target - eye).norm();
    vec3 t = (target - eye).normalized();
    this->yaw = atan2(-t[0], -t[2]);
    this->pitch = asin(t[1]);
}

void Camera::processInput(Window &window) {
    float currentTime = glfwGetTime();
    float dt = currentTime - lastInputTime;
    GLFWwindow *win = window.window;
    if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS)
        dist /= 1 + zoomSpeed*dt;
    if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS)
        dist *= 1 + zoomSpeed*dt;
    if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS)
        target -= getRightVector()*moveSpeed*dt;
    if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS)
        target += getRightVector()*moveSpeed*dt;
    if (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        target += getUpVector()*moveSpeed*dt;
    if (glfwGetKey(win, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        target -= getUpVector()*moveSpeed*dt;
    vec2 mousePos = window.mousePos();
    if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        yaw -= (mousePos[0] - lastMousePos[0])*turnSpeed*M_PI/180;
        pitch -= (mousePos[1] - lastMousePos[1])*turnSpeed*M_PI/180;
    }
    lastInputTime = currentTime;
    lastMousePos = mousePos;
}

void Camera::apply(Window &window) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)window.width/window.height;
    gluPerspective(180/M_PI*fovy/2, aspect, 0.01, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -dist);
    glRotatef(-180/M_PI*pitch, 1, 0, 0);
    glRotatef(-180/M_PI*yaw, 0, 1, 0);
    glTranslatef(-target[0], -target[1], -target[2]);
}

vec3 Camera::getForwardVector() {
    return vec3(-sin(yaw)*cos(pitch),sin(pitch),-cos(yaw)*cos(pitch));
}

vec3 Camera::getRightVector() {
    return vec3(cos(yaw),0,-sin(yaw));
}
vec3 Camera::getUpVector() {
    return vec3(sin(yaw)*sin(pitch),cos(pitch),cos(yaw)*sin(pitch));
}

#endif
