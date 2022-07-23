#ifndef MESH_HPP
#define MESH_HPP

#include<bits/stdc++.h>
#include "common.hpp"
#include "draw.hpp"
#include "camera.hpp"
#include "gui.hpp"
#include "path.hpp"
#include "half_edge.hpp"
using namespace std;
using namespace Eigen;

class Mesh {
    private:
        bool isPlaying = false;
        double t = 0;
        HalfEdge *mesh;
        Path* currentPath = NULL;
        void setupMesh(vector<vec3> Vertices, vector<int> Indices);
        void setupPath();
        int currIntersectIdx;
        vec3 upVec;
        vector<tuple<vec3, int, int>> intersectPts;
        vector<vec3> normals;
        void removeDuplicates(vector<tuple<vec3,int,int>> &vertices);
        vector<tuple<vec3, int, int>> filterAndSort(vector<tuple<vec3, int, int>> intersections, vec3 startPoint, vec3 endPoint, bool first);
        Vertex* vertexLast;
        Edge* crossEdgeLeft;
        Edge* crossEdgeRight;
        Edge* sideEdgeLeft;
        Edge* sideEdgeRight;

    public:
        Mesh();
        void update(float dt);
        void processInput(Window &window);
        void renderMesh(); 

};

//Mesh constructor
Mesh::Mesh(){                       
    //Using a sample mesh
    upVec = vec3(0,0,1);

    vector<vec3> vertices;
    vector<int> indices;

    for(int i = 0;i < 50;i++){
        for(int j = 0;j < 50;j++){
            vertices.push_back(vec3(i*0.5,j*0.5,0));
        }
    }

    for(int i = 0;i < 49;i++){
        for(int j = 0;j < 49;j++){
            indices.push_back(i*50+j);
            indices.push_back((i+1)*50 + j);
            indices.push_back((i+1)*50 + j+1);
            indices.push_back(i*50+j);
            indices.push_back((i+1)*50 + j+1);
            indices.push_back(i*50 + j + 1);
        }
    }

    this->mesh = new HalfEdge(vertices, indices);
}

//Mesh Update
void Mesh::update(float dt){
    //Update the mesh
    if(isPlaying){
        if(currIntersectIdx < intersectPts.size()){
            if(currIntersectIdx == 0){
                this->mesh->reMesh(intersectPts[currIntersectIdx], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }else if(currIntersectIdx == intersectPts.size()-1){
                this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }else{
                this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }
            currIntersectIdx++;
        }else{
            isPlaying = false;
        }
    }
}

//Setting up the path for cut/tear
void Mesh::setupPath(){
    //Custom path
    currentPath = new Path();
    vector<vec3> inputPts;
    vector<vec3> inputPts2;
    inputPts.push_back(vec3(3.234, 13.725, 0));
    inputPts.push_back(vec3(7.345, 21.345, 0));
    inputPts.push_back(vec3(10.237, 21.123, 0));
    inputPts.push_back(vec3(14.334, 13.721, 0));
    currentPath->addCurve(inputPts);
    inputPts2.push_back(vec3(14.334, 13.721, 0));
    inputPts2.push_back(vec3(18.234, 6.43, 0));
    inputPts2.push_back(vec3(20.34, 5.77, 0));
    inputPts2.push_back(vec3(23.237, 13.823, 0));
    currentPath->addCurve(inputPts2);

    

    //Setting up the intersection points
    vec3 startPoint = this->currentPath->lastPoint;
    vec3 startTangent = this->currentPath->lastTangent;
    bool isFirst = true;
    while(!this->currentPath->isOver){
        this->currentPath->updatePath();
        vec3 endPoint = this->currentPath->lastPoint;
        vec3 endTangent = this->currentPath->lastTangent;
        vec3 dir = (endPoint - startPoint).normalized();
        vec3 normal = upVec.cross(dir);
        Plane plane(startPoint, normal);
        vector<tuple<vec3, int, int>> intersections = this->mesh->Intersect(plane);
        auto filteredIntersections = filterAndSort(intersections, startPoint, endPoint, isFirst);
        for(auto intPt : filteredIntersections){
            this->intersectPts.push_back(intPt);
            this->normals.push_back(normal);
        }
        startPoint = endPoint;
        startTangent = endTangent;
        isFirst = false;
    }
    removeDuplicates(this->intersectPts);
}

//Mesh Process Input
void Mesh::processInput(Window &window){
    GLFWwindow *win = window.window;
    if(glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS){
        if(!isPlaying){
            isPlaying = true;
            setupPath();
        }
    }
}

void Mesh::removeDuplicates(vector<tuple<vec3,int,int>> &vertices){
    int i = 0;
    while(i < vertices.size()-1){
        if(get<0>(vertices[i]) == get<0>(vertices[i+1])){
            if(get<1>(vertices[i]) == 2){
                vertices.erase(vertices.begin()+i);
            }else{
                vertices.erase(vertices.begin()+i+1);
            }
        }else{
            i++;
        }
    }
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
    auto directionSort = [startPoint, direction] (tuple<vec3, int, int> a, tuple<vec3, int, int> b) -> bool
    {
        double x = (get<0>(a) - startPoint).dot(direction);
        double y = (get<0>(b) - startPoint).dot(direction);
        return x <= y;  
    };
    sort(filteredIntersections.begin(),filteredIntersections.end(),directionSort);
    this->mesh->vertex_list.push_back(new Vertex(endPoint));
    filteredIntersections.push_back(make_tuple(endPoint,2,currentSize++));
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

#endif