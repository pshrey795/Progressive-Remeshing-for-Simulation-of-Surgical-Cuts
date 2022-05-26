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

class HalfEdge {

    public:
        //Vertices, Edges and Faces
        vector<Vertex*> vertex_list;
        vector<half_edge*> edge_list;
        vector<Face*> face_list;
    
        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<int> Indices);

};

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<int> Indices){
    int n = Vertices.size();

    //To be used at last to link twin edges
    int twinEdges[n][n] = {-1};

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
        twinEdges[b][a] = i;
        twinEdges[c][b] = i+1;
        twinEdges[a][c] = i+2;

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

    //Linking twin edges
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            if(twinEdges[j][i] != -1){
                //This means that a half edge exists from i to j
                int currentIndex = twinEdges[j][i];
                if(twinEdges[i][j]!=-1){
                    //This means that there exists a twin edge of the current edge
                    int twinIndex = twinEdges[i][j];
                    this->edge_list[currentIndex]->twin = this->edge_list[twinIndex];
                }
            }
        }
    }
}

#endif