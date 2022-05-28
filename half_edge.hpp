#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;
using namespace Eigen;

struct half_edge{
    struct Vertex *startVertex;
    struct half_edge *next;
    struct half_edge *prev;
    struct half_edge *twin;
    struct Face *face;
    half_edge(Vertex *startVertex, Face *face){
        this->startVertex = startVertex;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = face;
    }
};

//Vertex stores:
//1) 3D coordinate
//2) An outgoing half edge
struct Vertex{
    vec3 position;
    struct half_edge *edge;
    double offset;
    Vertex(vec3 pos){
        position = pos;
        edge = NULL;
    }
};

//Face stores;
//1) A half edge that is part of the face
//2) A normal
//3) Indices of its vertices
struct Face{
    struct half_edge *edge;
    vector<int> indices;
    Face(int a, int b, int c){
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
        edge = NULL;
    }
};

struct Plane{
    vec3 origin;
    vec3 normal;
    Plane(vec3 origin, vec3 normal){
        this->origin = origin;
        this->normal = normal.normalized();
    }
};

class HalfEdge {

    public:
        //Vertices, Edges and Faces
        vector<Vertex*> vertex_list;
        vector<half_edge*> edge_list;
        vector<Face*> face_list;
        vector<vector<int>> adjList;
    
        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<int> Indices);
        vector<tuple<vec3, int, int>> Intersect(Plane plane);

    private:
        //Helper functions for checking intersection
        void VertexOffsets(Plane plane);
        vector<int> IntersectingEdges();
        vector<tuple<vec3,int,int>> IntersectingVertices(vector<int> edges);

};

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<int> Indices){
    int n = Vertices.size();

    for(int i=0;i<n;i++){
        vector<int> adj;
        for(int j=0;j<n;j++){
            adj.push_back(-1);
        }
        this->adjList.push_back(adj); 
    }

    //Adding vertices
    for(int i=0;i<Vertices.size();i++){
        this->vertex_list.push_back(new Vertex(Vertices[i]));
    }

    int i=0;
    while(i < Indices.size()){
        int a,b,c;

        //Vertex Indices
        a = Indices[i];
        b = Indices[i+1];
        c = Indices[i+2];

        //Create Face and Half Edges
        struct Face* f = new Face(a,b,c);
        struct half_edge* e1 = new half_edge(vertex_list[a],f);
        struct half_edge* e2 = new half_edge(vertex_list[b],f);
        struct half_edge* e3 = new half_edge(vertex_list[c],f);

        //Log twin edge values for later linking
        adjList[a][b] = i;
        adjList[b][c] = i+1;
        adjList[c][a] = i+2;

        //Linking next and prev entries
        e1->next = e2;
        e2->next = e3;
        e3->next = e1;
        e1->prev = e3;
        e2->prev = e1;
        e3->prev = e2;

        //Linking face to one of the half edges
        f->edge = e1;

        //Adding half edges
        this->edge_list.push_back(e1);
        this->edge_list.push_back(e2);
        this->edge_list.push_back(e3);

        //Next iteration
        i+=3;
    }
    int currentSize = this->edge_list.size();

    //Linking twin edges
    for(int a=0;a<n;a++){
        for(int b=0;b<n;b++){
            if(adjList[a][b] == -1){
                if(adjList[b][a] != -1){
                    auto v = this->edge_list[adjList[b][a]]->next->startVertex;
                    struct half_edge* newEdge = new half_edge(v,NULL);
                    newEdge->twin = this->edge_list[adjList[b][a]];
                    this->edge_list[adjList[b][a]]->twin = newEdge;
                    this->edge_list.push_back(newEdge);
                    adjList[a][b] = currentSize;
                }
            }else{
                if(adjList[b][a] != -1){
                    edge_list[adjList[a][b]]->twin = edge_list[adjList[b][a]];
                }
            }
        }
    }
}

//Intersection with a plane
vector<tuple<vec3,int,int>> HalfEdge::Intersect(Plane plane){
    VertexOffsets(plane);
    auto intersectingEdges = IntersectingEdges();
    return IntersectingVertices(intersectingEdges);
}

//Calculating offsets for each vertex from the plane
void HalfEdge::VertexOffsets(Plane plane){
    for(int i=0;i<vertex_list.size();i++){
        double newOffset = plane.normal.dot(vertex_list[i]->position - plane.origin);
        vertex_list[i]->offset = newOffset;
    }
}

//Obtaining the edges that intersect with the plane
vector<int> HalfEdge::IntersectingEdges(){
    vector<int> intersectingEdges;
    vector<int> isEdgeIntersecting;
    for(int i=0;i<this->edge_list.size();i++){
        isEdgeIntersecting.push_back(0);
    }
    for(int i=0;i<edge_list.size();i++){
        if(isEdgeIntersecting[i] == 0){
            auto v1 = edge_list[i]->startVertex;
            auto v2 = edge_list[i]->twin->startVertex;
            if(v1->offset * v2->offset < 0){
                intersectingEdges.push_back(i);
                isEdgeIntersecting[i] = 1;
                int j = find(edge_list.begin(),edge_list.end(),edge_list[i]->twin) - edge_list.begin();
                isEdgeIntersecting[j] = 1;
            }
        }
    }
    return intersectingEdges;
}

//Obtaining the intersection points of the edges
vector<tuple<vec3,int,int>> HalfEdge::IntersectingVertices(vector<int> edges){
    vector<tuple<vec3,int,int>> intersectingVertices;
    for(int i=0;i<vertex_list.size();i++){
        if(vertex_list[i]->offset==0){
            intersectingVertices.push_back(make_tuple(vertex_list[i]->position,0,i));
        }
    }
    for(auto idx : edges){
        double offset1 = edge_list[idx]->startVertex->offset;
        double offset2 = edge_list[idx]->twin->startVertex->offset;
        double factor = offset1 / (offset1 - offset2);
        vec3 edgeStart = edge_list[idx]->startVertex->position;
        vec3 edgeEnd = edge_list[idx]->twin->startVertex->position;
        vec3 newPoint = edgeStart + factor * (edgeEnd - edgeStart);
        intersectingVertices.push_back(make_tuple(newPoint,1,idx));
    }
    return intersectingVertices;
}

#endif