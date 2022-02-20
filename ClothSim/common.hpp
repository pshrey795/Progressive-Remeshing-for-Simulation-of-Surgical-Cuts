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
typedef Eigen::Matrix<float, 3, 3> mat3;

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

Matrix<float,Dynamic,1> matrix_vector_mult(Matrix<float, Dynamic, Dynamic> A, Matrix<float,Dynamic,1> B, int size){
    Matrix<float,Dynamic,1> res;
    res.resize(size);
    for(int i=0;i<size;i++){
        res(i) = 0;
        for(int j=0;j<size;j++){
            res(i) += A(i,j)*B(j);
        }
    }
    return res;
}

Matrix<vec3, Dynamic, Dynamic> matrix_mult(Matrix<mat3,Dynamic,Dynamic> A,Matrix<vec3,Dynamic,1> B,int size){
    Matrix<vec3, Dynamic, 1> res;
    res.resize(size);
    for(int i=0;i<size;i++){
        res(i) = vec3(0,0,0);
        for(int j=0;j<size;j++){
            res(i) = res(i) + matrix_vector_mult(A(i,j),B(j),3);
        }
    }
    return res;
}

Matrix<mat3, Dynamic, Dynamic> convert(Matrix<float,Dynamic,Dynamic> A, int size){
    Matrix<mat3, Dynamic, Dynamic> res;
    res.resize(size,size);
    for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
            if(i==j){
                res(i,j) = A(i,j) * mat3::Identity();
            }else{
                res(i,j) = mat3::Zero();
            }
        }
    }
    return res;
}

Matrix<float,Dynamic, Dynamic> explode(Matrix<mat3,Dynamic,Dynamic> A, int size){
    Matrix<float,Dynamic,Dynamic> res;
    res.resize(size,size);
    for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
            res(i,j) = A(i/3,j/3)(i%3,j%3);
        }
    }
    return res;
}

Matrix<mat3,Dynamic,Dynamic> implode(Matrix<float,Dynamic,Dynamic> A,int size){
    Matrix<mat3,Dynamic,Dynamic> res;
    res.resize(size,size);
    for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
            res(i,j)(i%3,j%3) = A(i,j);
        }
    }
    return res;
}

#endif
