#ifndef MESH_HPP
#define MESH_HPP

#include<bits/stdc++.h>
#include "common.hpp"
#include "draw.hpp"
#include "camera.hpp"
#include "gui.hpp"
#include "half_edge.hpp"
using namespace std;
using namespace Eigen;

#define EPSILON 0.01

//Cut to be defined as a plane with normal n and point p
struct cut{
    vec3 seedPoint;
    vec3 normal;
    vec3 dir;
};

//Tear to be defined as a sequence of cuts to approximate a general curve 
struct tear{
    //Cuts to be maintained in order of application
    vector<cut> cuts;
    int currentCut;
    tear(){
        currentCut = 0;
    }
};

class Mesh {
    private:
        HalfEdge *mesh;
        
    public:
        Mesh();
        void setupMesh(vector<vec3> Vertices, vector<int> Indices);
        void renderMesh();
        void processInput(Camera cam, Window win);
        void updateMesh(vec2 mousePos);   

};

//Mesh constructor
Mesh::Mesh(){
    //Add sample geometry
    vector<vec3> vertices;
    vector<int> indices;

    vertices = {
        vec3(-1, 1, 0),
        vec3(-1, -1, -1),
        vec3(1, -1, -1),
        vec3(2, 1, 0),
        vec3(0, 1, 0)
        // {1, -1, 0}
    };
    indices = {
        1, 2, 4,
        2, 3, 4,
        1, 4, 0
    };

    //Fill data structures
    this->mesh = new HalfEdge(vertices, indices);

    //Testing
    auto intersectInfo = this->mesh->Intersect(Plane(vec3(0.0,0.0,0.0),vec3(1.0,1.0,0.0)));
    for(auto x:intersectInfo){
        vec3 points = get<0>(x);
        cout << points[0] << " " << points[1] << " " << points[2] << " " << "\n";
    }
}

//Rendering the mesh
void Mesh::renderMesh(){
    int n = mesh->face_list.size();
    for(int i = 0; i < n; i++){
        Face* f = mesh->face_list[i];
        vec3 v1 = mesh->vertex_list[f->indices[0]]->position;
        vec3 v2 = mesh->vertex_list[f->indices[1]]->position;
        vec3 v3 = mesh->vertex_list[f->indices[2]]->position;
        drawTri(v1,v2,v3);
        setColor(vec3(0,0,0));
        drawLine(v1,v2);
        drawLine(v2,v3);
        drawLine(v3,v1);
    }
}

#endif