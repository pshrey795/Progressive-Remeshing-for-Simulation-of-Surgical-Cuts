#include "common.hpp"
#include<bits/stdc++.h>
using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    Matrix<vec3,Dynamic,Dynamic> x;
    x.resize(3,1);
    x(0) = vec3(1,1,1);
    x(1) = vec3(1,1,1);
    x(2) = vec3(1,1,1);
    float y = 10;
    x = y * x;
    //cout << vec3(1,1,1) << endl;
    cout << x(0,0) << endl;
    cout << x(0,1) << endl;
    cout << x(1,0) << endl;
    cout << x(1,1) << endl;
}