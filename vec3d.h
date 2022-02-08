#pragma once
#include <bits/stdc++.h>
using namespace std;

class vec3d{

    public:
        //Constructors
        vec3d(){
            this->vec[0] = 0;
            this->vec[1] = 0;
            this->vec[2] = 0;
            this->magnitude = 0;
        }
        vec3d(double x,double y,double z){
            this->vec[0] = x;
            this->vec[1] = y;
            this->vec[2] = z;
            this->magnitude = sqrt(x*x+y*y+z*z);
        }

        //Access Methods
        double x(){return vec[0];}
        double y(){return vec[1];}
        double z(){return vec[2];}
        double length(){return magnitude;}

        //Operator Overloading on 3D vectors
        double operator[](int t){
            return vec[t];
        }
        vec3d operator-(){
            return vec3d(-vec[0],-vec[1],-vec[2]);
        }
        vec3d& operator+=(vec3d v){
            vec[0] += v[0];
            vec[1] += v[1];
            vec[2] += v[2];
            return *this;
        }
        vec3d& operator-=(vec3d v){
            vec[0] -= v[0];
            vec[1] -= v[1];
            vec[2] -= v[2];
            return *this;
        }
        vec3d& operator*=(double t){
            vec[0] *= t;
            vec[1] *= t;
            vec[2] *= t;
            return *this;
        }
        vec3d& operator/=(double t){
            return *this *= (1/t);
        }
        bool operator==(vec3d v){
            return (this->vec[0]==v[0] && this->vec[1]==v[1] && this->vec[2]==v[2]);
        }

    private:
        double vec[3];
        double magnitude;

};

//Alternate aliases
using point3d = vec3d;
using color3d = vec3d;

//Static vector methods
ostream& operator<<(ostream& out,vec3d v){
    return out << "(" <<v[0]<<","<<v[1]<<","<<v[2]<<") ";
}
vec3d operator+(vec3d u,vec3d v){
    return vec3d(u[0]+v[0],u[1]+v[1],u[2]+v[2]);
}
vec3d operator-(vec3d u,vec3d v){
    return vec3d(u[0]-v[0],u[1]-v[1],u[2]-v[2]);
}
vec3d operator*(vec3d v,double t){
    return vec3d(v[0]*t,v[1]*t,v[2]*t);
}
vec3d operator*(double t,vec3d v){
    return vec3d(v[0]*t,v[1]*t,v[2]*t);
}
vec3d operator*(vec3d u,vec3d v){
    return vec3d(u[0]*v[0],u[1]*v[1],u[2]*v[2]);
}
vec3d operator/(vec3d v,double t){
    return v * (1/t);
}
vec3d operator/(vec3d u,vec3d v){
    return vec3d(u[0]/v[0],u[1]/v[1],u[2]/v[2]);
}
vec3d unit_vec(vec3d v){
    return v / v.length();
}
double dot(vec3d u,vec3d v){
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]; 
}
vec3d cross(vec3d u,vec3d v){
    return vec3d(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);
}

double area(point3d p1,point3d p2,point3d p3){
    double base = (p2-p3).length();
    double height = abs(cross(p1-p2,unit_vec(p3-p2)).length());
    return 0.5*base*height;
}

//Reflection/Refraction vectors
vec3d reflect(vec3d incident_ray, vec3d normal){
    vec3d norm = unit_vec(normal);
    double t = 2 * dot(incident_ray,norm);
    return incident_ray - t * norm;
}
vec3d refract(vec3d incident_ray, vec3d normal, double myu){
    vec3d norm = unit_vec(normal);
    auto cosine = dot(unit_vec(incident_ray),norm);
    auto mag = incident_ray.length();
    auto t = sqrt(1 - cosine*cosine) / myu;
    if(t<=1){
        //Normal refraction
        vec3d t_perpendicular = (incident_ray - mag * cosine * norm) / myu;
        vec3d t_parallel = (-sqrt(mag * mag - t_perpendicular.length() * t_perpendicular.length())) * norm; 
        return t_perpendicular + t_parallel;
    }else{
        //Total internal reflection
        return reflect(incident_ray,normal);
    }
}

//Utilities
double random_double(){
    return rand() / (RAND_MAX + 1.0);
}
