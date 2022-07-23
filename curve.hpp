#ifndef CURVE_HPP
#define CURVE_CPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;

bool double_eq(double a, double b){
    return (abs(a-b) <= 0.01f);
}

long long factorial(int n){
    long long res = 1;
    for(int i=1;i<=n;i++){
        res *= (long long)i;
    }
    return res;
}

double C(int n, int r){
    return (double)factorial(n)/((double)(factorial(r)*factorial(n-r)));
}

double power(double x, int n){
    if(n==0){
        return 1;
    }else{
        return pow(x,n);
    }
}

class Curve {

    private:
        int size;
        vector<vec3> controlPts;
        int axis;

    public:
        Curve(vector<vec3> inputPts, int axis){
            this->size = inputPts.size();
            for(int i = 0; i < this->size; i++){
                this->controlPts.push_back(inputPts[i]);
            }
            this->axis = axis;
        }

        vec3 getPoint(float t){
            vec3 pt = vec3(0,0,0);
            int n = size - 1;
            for(int i=0;i<size;i++){
                float Ji = C(n,i) * power(t,i) * power(1.0f-t,n-i);
                pt += (controlPts[i] * Ji);
            }
            return pt;
        }

        vec3 getTangent(float t){
            vec3 tangent = vec3(0,0,0);
            int n = size - 1;
            for(int i=0;i<size;i++){
                float Ji = i * C(n,i) * power(t,i-1) * power(1.0f-t,n-i);
                Ji -= (size-i) * C(n,i) * power(t,i) * power(1.0f-t,n-i-1);
                tangent += controlPts[i] * Ji; 
            }
            return tangent;
        }

};

#endif