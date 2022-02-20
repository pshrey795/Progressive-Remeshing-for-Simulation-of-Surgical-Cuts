#ifndef COMMON_HPP
#define COMMON_HPP

#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>

#if __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;

using namespace Eigen;

Matrix<vec3, Dynamic, Dynamic> operator*(float dt,Matrix<vec3, Dynamic, Dynamic> m){
    Matrix<vec3, Dynamic, Dynamic> res(m.rows(),m.cols());
    for(int i=0;i<m.rows();i++){
        for(int j=0;j<m.cols();j++){
            res(i,j) = dt*m(i,j);
        }
    }
    return res;
} 

Matrix<vec3, Dynamic, Dynamic> matrix_vector_mult(Matrix<float, Dynamic, Dynamic> A, Matrix<vec3, Dynamic, 1> B, int size){
    Matrix<vec3, Dynamic, 1> res;
    res.resize(size);
    for(int i=0;i<size;i++){
        res(i) = vec3(0,0,0);
        for(int j=0;j<size;j++){
            res(i) += A(i,j)*B(j);
        }
    }
    return res;
}

#endif
