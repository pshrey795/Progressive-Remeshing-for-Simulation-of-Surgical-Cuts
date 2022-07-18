#ifndef CURVE_HPP
#define CURVE_CPP

#include "common.hpp"

using namespace std;

class Curve {

    public:
        vec3 b0;
        vec3 b1;
        vec3 b2;
        vec3 b3;
        int axis;

        Curve(vec3 b0, vec3 b1, vec3 b2, vec3 b3, int axis){
            this->b0 = b0;
            this->b1 = b1;
            this->b2 = b2;
            this->b3 = b3;
            this->axis = axis;
        }

        vec3 getPoint(float t){
            float J0 = (1.0f - t) * (1.0f - t) * (1.0f - t);
            float J1 = 3.0f * t * (1.0f - t) * (1.0f - t);
            float J2 = 3.0f * t * t * (1.0f - t);
            float J3 = t * t * t;
            return J0 * this->b0 + J1 * this->b1 + J2 * this->b2 + J3 * this->b3;
        }

};

#endif