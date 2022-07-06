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
        void setupMesh(vector<vec3> Vertices, vector<int> Indices);
        vector<tuple<vec3, int, int>> filterAndSort(vector<tuple<vec3, int, int>> intersections, vec3 startPoint, vec3 endPoint, bool first);

    public:
        Mesh();
        void update(float dt);
        void Cut(vec3 seedPoint, vec3 normal);
        void Tear(vector<vec3> startPoints, vector<vec3> endPoints, vector<vec3> normals);
        void renderMesh(); 

};

//Mesh constructor
Mesh::Mesh(){
    //Add sample geometry
    vector<vec3> vertices;
    vector<int> indices;

    vertices = {
        vec3(-1,1,0),
        vec3(0,1,0),
        vec3(1,1,0),
        vec3(-1,0,0),
        vec3(0,0,0),
        vec3(1,0,0),
        vec3(-1,-1,0),
        vec3(0,-1,0),
        vec3(1,-1,0),
    };
    indices = {
        0, 3, 4,
        0, 4, 1,
        1, 4, 5,
        1, 5, 2,
        3, 6, 4,
        4, 6, 7,
        4, 7, 5,
        5, 7, 8,
    };

    // for(int i = 0;i < 50;i++){
    //     for(int j = 0;j < 50;j++){
    //         vertices.push_back(vec3(i*0.5,j*0.5,0));
    //     }
    // }

    // for(int i = 0;i < 49;i++){
    //     for(int j = 0;j < 49;j++){
    //         indices.push_back(i*50+j);
    //         indices.push_back((i+1)*50 + j);
    //         indices.push_back((i+1)*50 + j+1);
    //         indices.push_back(i*50+j);
    //         indices.push_back((i+1)*50 + j+1);
    //         indices.push_back(i*50 + j + 1);
    //     }
    // }

    //Fill data structures
    this->mesh = new HalfEdge(vertices, indices);

    //Testing
    // this->Cut(vec3(-2.0,14.0,0.0),vec3(1.0,1.5,0.0));
    vector<vec3> startPoints = {
        vec3(-0.875,-0.75,0),
        vec3(0.25,-0.25,0)
    };
    vector<vec3> endPoints = {
        vec3(0.25,-0.25,0),
        vec3(0.75,0.75,0)
    };
    vector<vec3> normals = {
        vec3(-4,9,0),
        vec3(-2,1,0)
    };
    this->Tear(startPoints, endPoints, normals);
}

//Mesh Update
void Mesh::update(float dt){
    
}

//Filter intersection points
vector<tuple<vec3,int,int>> Mesh::filterAndSort(vector<tuple<vec3,int,int>> intersections, vec3 startPoint, vec3 endPoint,bool first){
    vector<tuple<vec3,int,int>> filteredIntersections;
    vec3 direction = endPoint - startPoint;
    double lowerBound = direction.dot(startPoint - startPoint);
    double upperBound = direction.dot(endPoint - startPoint);
    int currentSize = this->mesh->vertex_list.size();
    if(first){
        this->mesh->vertex_list.push_back(new Vertex(startPoint));
        filteredIntersections.push_back(make_tuple(startPoint,2,currentSize++));
    }
    for(auto x : intersections){
        vec3 point = get<0>(x);
        double dist = direction.dot(point - startPoint);
        if(dist > lowerBound && dist < upperBound){
            filteredIntersections.push_back(x);
        }
    }
    this->mesh->vertex_list.push_back(new Vertex(endPoint));
    filteredIntersections.push_back(make_tuple(endPoint,2,currentSize++));
    auto directionSort = [startPoint, direction] (tuple<vec3, int, int> a, tuple<vec3, int, int> b) -> bool
    {
        double x = (get<0>(a) - startPoint).dot(direction);
        double y = (get<0>(b) - startPoint).dot(direction);
        return x <= y;  
    };
    sort(filteredIntersections.begin(),filteredIntersections.end(),directionSort);
    return filteredIntersections;
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

    // Re-meshing for each intersection point
    assert(intersections.size() >= 2);
    for(int i=0;i<intersections.size();i++){
        if(i==0){
            this->mesh->reMesh(intersections[i], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }else if(i==intersections.size()-1){
            this->mesh->reMesh(intersections[i], intersections[i-1], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }else{
            this->mesh->reMesh(intersections[i], intersections[i-1], intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normal.normalized()); 
        }
    }
}

//Tearing
void Mesh::Tear(vector<vec3> startPoints, vector<vec3> endPoints, vector<vec3> normals){
    vector<tuple<vec3, int, int>> intersections;
    vector<vec3> allNormals;

    for(int i=0;i<startPoints.size();i++){
        //The normal should be perpendicular to the line joining the end points of the cut
        assert(normals[i].dot(endPoints[i]-startPoints[i]) == 0);

        //Define the plane for cut and intersect it with mesh
        Plane plane(startPoints[i], normals[i]);
        vector<tuple<vec3, int, int>> intersection = mesh->Intersect(plane);

        //Filter(to obtain points only in between the end points) and sort them by direction for progressive re-meshing
        intersection = filterAndSort(intersection,startPoints[i],endPoints[i],(i==0)?true:false);

        //Testing
        for(auto x:intersection){
            intersections.push_back(x);
            allNormals.push_back(normals[i].normalized());
            vec3 points = get<0>(x);
            cout << points[0] << " " << points[1] << " " << points[2] << " " << "\n";
        }
    }

    //Remeshing starts here
    Vertex* vertexLast;
    Edge* crossEdgeLeft;
    Edge* crossEdgeRight;
    Edge* sideEdgeLeft;
    Edge* sideEdgeRight;

    // Re-meshing for each intersection point
    assert(intersections.size() >= 2);
    for(int i=0;i<intersections.size();i++){
        if(i==0){
            this->mesh->reMesh(intersections[i], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, allNormals[i]); 
        }else if(i==intersections.size()-1){
            this->mesh->reMesh(intersections[i], intersections[i-1], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, allNormals[i]); 
        }else{
            this->mesh->reMesh(intersections[i], intersections[i-1], intersections[i+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, allNormals[i]); 
        }
    }

} 

#endif