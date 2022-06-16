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

class Mesh {
    private:
        HalfEdge *mesh;

    public:
        Mesh();
        void Cut(vec3 seedPoint, vec3 normal);
        void Tear(vec3 startPoint, vec3 endPoint);
        void setupMesh(vector<vec3> Vertices, vector<int> Indices);
        void renderMesh(); 

};

//Mesh constructor
Mesh::Mesh(){
    //Add sample geometry
    vector<vec3> vertices;
    vector<int> indices;

    vertices = {
        vec3(0,0,0),
        vec3(0,1,0),
        vec3(-1,0,0),
        vec3(-1,1,0),
        vec3(1,-1,0)
    };
    indices = {
        0, 1, 3,
        0, 3, 2,
        0, 2, 4,
        0, 4, 1
    };

    //Fill data structures
    this->mesh = new HalfEdge(vertices, indices);

    //Testing
    this->Cut(vec3(-2.0,2.0,0.0),vec3(1.0,1.0,0.0));
}

//Rendering the mesh
void Mesh::renderMesh(){
    int n = mesh->face_list.size();
    for(int i = 0; i < n; i++){
        Face* f = mesh->face_list[i];
        vec3 v1 = mesh->vertex_list[f->indices[0]]->position;
        vec3 v2 = mesh->vertex_list[f->indices[1]]->position;
        vec3 v3 = mesh->vertex_list[f->indices[2]]->position;
        setColor(vec3(1.0f,0.0f,0.0f));
        drawTri(v1,v2,v3);
        setColor(vec3(0,0,0));
        setLineWidth(1);
        drawLine(v1,v2);
        drawLine(v2,v3);
        drawLine(v3,v1);
    }
}

//Cutting
void Mesh::Cut(vec3 seedPoint, vec3 normal){
    //Create plane for cut and intersect it with mesh
    Plane plane(seedPoint, normal);
    vector<tuple<vec3, int, int>> intersections = mesh->Intersect(plane);
    if(intersections.size() == 0){
        cout << "No intersections" << endl;
        return;
    }
    vec3 direction = (get<0>(intersections[0]) - seedPoint).normalized();

    auto directionSort = [seedPoint, direction] (tuple<vec3, int, int> a, tuple<vec3, int, int> b) -> bool
    {
        double x = (get<0>(a) - seedPoint).dot(direction);
        double y = (get<0>(b) - seedPoint).dot(direction);
        return x <= y;  
    };

    //Sort intersections by direction
    sort(intersections.begin(), intersections.end(), directionSort);

    //Testing
    for(auto x:intersections){
        vec3 points = get<0>(x);
        cout << points[0] << " " << points[1] << " " << points[2] << " " << "\n";
    }

    //Remeshing starts here
    Vertex* vertexLast;
    Edge* crossEdgeLeft;
    Edge* crossEdgeRight;
    Edge* sideEdgeLeft;
    Edge* sideEdgeRight;

    //Re-meshing for each intersection point
    assert(intersections.size() >= 2);
    for(int i=0;i<intersections.size();i++){
        if(i==0){
            this->mesh->reMesh(intersections[i], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }else if(i==intersections.size()-1){
            this->mesh->reMesh(intersections[i], intersections[i-1], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }else{
            this->mesh->reMesh(intersections[i], intersections[i-1], intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }
        cout << i << endl;
    }


}

//Tearing
void Mesh::Tear(vec3 startPoint, vec3 endPoint){
} 

#endif