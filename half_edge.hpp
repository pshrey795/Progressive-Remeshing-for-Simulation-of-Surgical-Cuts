#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;
using namespace Eigen;

#define EPSILON 0.01

struct Edge{
    struct Vertex *startVertex;
    struct Edge *next;
    struct Edge *prev;
    struct Edge *twin;
    struct Face *face;
    Edge(Vertex *startVertex, Face *face){
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
    struct Edge *edge;
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
    struct Edge *edge;
    int indices[3];
    Face(int a, int b, int c){
        indices[0] = a;
        indices[1] = b;
        indices[2] = c;
        edge = NULL;
    }
    void setFace(int a, int b, int c){
        indices[0] = a;
        indices[1] = b;
        indices[2] = c;
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
        vector<Edge*> edge_list;
        vector<Face*> face_list;
        vector<vector<int>> adjList;
    
        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<int> Indices);
        vector<tuple<vec3, int, int>> Intersect(Plane plane);

        //Re Mesh
        //Intersection Record (point, type, edge of intersection) (Type 0: Vertex, Type 1: Edge, Type 2: Face)
        //Type -1 if last or first point

        //1) Intersection Record of current point
        //2) Intersection Record of last point
        //3) Intersection Record of next point
        //2) Left vertex of the split of the last intersection point
        //3) Right vertex of the split of the last intersection point
        //4) Left edge of the split of the last intersection point
        //5) Right edge of the split of the last intersection point
        //6) Left face of the split of the last intersection point
        //7) Right face of the split of the last intersection point
        //8) Direction of normal of plane for splitting
        void reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Vertex* leftVertex, Vertex* rightVertex, Edge* leftEdge, Edge* rightEdge, Face* leftFace, Face* rightFace, vec3 normal);

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
        struct Edge* e1 = new Edge(vertex_list[a],f);
        struct Edge* e2 = new Edge(vertex_list[b],f);
        struct Edge* e3 = new Edge(vertex_list[c],f);

        //Log twin edge values for later linking
        adjList[a][b] = i;
        adjList[b][c] = i+1;
        adjList[c][a] = i+2;

        //VERY IMPORTANT
        //Need to make sure that the indices are given in anti clockwise order
        //This condition is required for rendering as well as remeshing algorithms

        //Linking next and prev entries
        e1->next = e2;
        e2->next = e3;
        e3->next = e1;
        e1->prev = e3;
        e2->prev = e1;
        e3->prev = e2;

        //Linking face to one of the half edges
        f->edge = e1;
        this->face_list.push_back(f);

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
                    struct Edge* newEdge = new Edge(v,NULL);
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

//Remeshing 
void HalfEdge::reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Vertex* leftVertex, Vertex* rightVertex, Edge* leftEdge, Edge* rightEdge, Face* leftFace, Face* rightFace, vec3 normal){
    int currentType = get<1>(intPt);
    Vertex* newVertexLeft;
    Vertex* newVertexRight;
    Edge* newEdgeLeft;
    Edge* newEdgeRight;
    Face* newFaceLeft;
    Face* newFaceRight;
    int n = this->vertex_list.size();
    
    //Creating new vertices and edges or using an existing vertex depending on the type of intersection point
    if(currentType = 0){
        //Intersection point is already a vertex of the mesh
        newVertexLeft = this->vertex_list[get<2>(intPt)];
        vec3 oldPos = newVertexLeft->position;
        newVertexLeft->position = oldPos - normal * EPSILON;
        newVertexRight = new Vertex(oldPos + normal * EPSILON);
        this->vertex_list.push_back(newVertexRight);
    }else if(currentType == 1){
        //Intersection point is on an edge of the mesh
        vec3 oldPos = get<0>(intPt);
        newVertexLeft = new Vertex(oldPos - normal * EPSILON);
        newVertexRight = new Vertex(oldPos + normal * EPSILON);
        this->vertex_list.push_back(newVertexLeft);
        this->vertex_list.push_back(newVertexRight);
    }else{
        //Intersection point is on a face of the mesh(in case of a tear)
        //Only case in which the vertex doesn't split
        //Such type of vertices are allotted only to the start and end points of a tear
        newVertexLeft = newVertexRight = new Vertex(get<0>(intPt));
        this->vertex_list.push_back(newVertexLeft);
    }

    //Adding the new vertices to the mesh
    int newIndexLeft;
    int newIndexRight; 
    if(currentType == 0){
        //Only one new vertex added
        newIndexLeft = get<2>(intPt);
        newIndexRight = n;
    }else if(currentType == 1){
        //Two new vertices added
        newIndexLeft = n;
        newIndexRight = n + 1;
    }else{
        //One new vertex added
        newIndexLeft = n;
    }

    //Auxiliary variables
    int lastType = get<1>(lastIntPt);
    int nextType = get<1>(nextIntPt);
    bool first = (lastType == -1);
    bool last = (nextType == -1);

    //Complete Restructuring of data strcuture based on cases
    if(first){
        if(currentType == 0){

        }else if(currentType == 1){
            Edge* intersectingEdge = this->edge_list[get<2>(intPt)];
            if(intersectingEdge->face == NULL){
                intersectingEdge = intersectingEdge->twin;
            }
            newFaceLeft = intersectingEdge->face;
            int currentFaceIndex = find(this->face_list.begin(),this->face_list.end(),newFaceLeft) - this->face_list.begin();

            //Depending on the type of the next vertex
            if(nextType == 0){
                //Edge to Edge relations
                //Two new edges on the intersecting edge
                //Four on the path from this intersection point to the next
                //Two of the second type are to be passed onto the next remeshing iteration
                Edge* newEdge1;
                Edge* newEdge2;
                Edge* newEdge3;
                Edge* newEdge4;

                //For new edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newEdgeLeft;
                newEdgeLeft->twin = newEdge3;
                newEdge4->twin = newEdgeRight;
                newEdgeRight->twin = newEdge4;
                newEdge1->prev = intersectingEdge->prev;
                newEdge1->next = newEdge4;
                newEdge2->next = NULL;
                newEdge2->prev = NULL;
                newEdge3->next = NULL;
                newEdge3->prev = NULL;
                newEdge4->next = intersectingEdge->prev;
                newEdge4->prev = newEdge1;
                newEdgeLeft->next = intersectingEdge->next;
                newEdgeLeft->prev = intersectingEdge;
                newEdgeRight->next = NULL;
                newEdgeRight->prev = NULL;

                //For old edges
                intersectingEdge->next->next = newEdgeLeft;
                intersectingEdge->prev->next = newEdge1;
                intersectingEdge->prev->prev = newEdge4;
                intersectingEdge->prev = newEdgeLeft; 

                //Edge to Vertex relations
                newEdge1->startVertex = newVertexLeft;
                newEdge2->startVertex = intersectingEdge->next->startVertex;
                newEdge3->startVertex = newVertexLeft;
                newEdge4->startVertex = newVertexRight;

                int oldIndexLeft, oldIndexRight, oppIndex;
                Edge* currentEdge = newFaceLeft->edge;
                int i = 0;
                while(currentEdge != intersectingEdge){
                    i++;
                    currentEdge = currentEdge->next;
                }
                oldIndexRight = newFaceLeft->indices[i%3];
                oldIndexLeft = newFaceLeft->indices[(i+1)%3];
                oppIndex = newFaceLeft->indices[(i+2)%3];
                newEdgeLeft->startVertex = this->vertex_list[oppIndex];
                newEdgeRight->startVertex = this->vertex_list[oppIndex]; 

                //Vertex to Edge relations
                newVertexLeft->edge = newEdge3;
                newVertexRight->edge = newEdge4;

                //Face to Vertex/Edge relations
                newFaceRight = new Face(oppIndex,oldIndexRight,newIndexRight);
                newFaceRight->edge = newEdge4->next;
                newFaceLeft->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                newFaceLeft->edge = newEdgeLeft;

                //Edge to Face relations
                newEdge1->face = newFaceRight;
                newEdge1->next->face = newFaceRight;
                newEdge1->next->next->face = newFaceRight;
                newEdgeLeft->face = newFaceLeft;

                //Adding the new edges and faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdgeLeft);
                this->edge_list.push_back(newEdgeRight);
                this->face_list.push_back(newFaceRight);
            }else if(nextType == 1){

            }else{

            }
        }else{
        }
    }else if(last){

    }else{

    }






    //Reallocating vertices and edges for next intersection point
    leftVertex = newVertexLeft;
    rightVertex = newVertexRight;
    leftEdge = newEdgeLeft;
    rightEdge = newEdgeRight;
    leftFace = newFaceLeft;
    rightFace = newFaceRight;
}


#endif