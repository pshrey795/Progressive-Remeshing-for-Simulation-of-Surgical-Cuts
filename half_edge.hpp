#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;
using namespace Eigen;

#define EPSILON 0.05
#define DELTA 0.01

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
    Edge(){
        this->startVertex = NULL;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = NULL;
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
    Face(){
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
        vector<Edge*> edge_list;
        vector<Face*> face_list;
        vector<vector<int>> adjList;
    
        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<int> Indices);

        //Obtaining the intersection points of the edges
        //Each element is a tuple with the following elements
        //1) vec3 : intersection point
        //2) int : type of the intersection point(explained later in re-meshing)
        //3) int : index of the relevant data structure which stores the locality info
        // of the intersection point
        vector<tuple<vec3, int, int>> Intersect(Plane plane);

        //Re Mesh
        //Intersection Record (point, type, edge of intersection) (Type 0: Vertex, Type 1: Edge, Type 2: Face)
        //Type -1 if last or first point

        //1) Intersection Record of current point
        //2) Intersection Record of last point
        //3) Intersection Record of next point
        //2) Vertex of the last intersection
        //4) Left cross edge of the split of the last intersection point
        //5) Right cross edge of the split of the last intersection point
        //6) Left side edge of the split of the last intersection point
        //7) Right side edge of the split of the last intersection point
        //8) Direction of normal of plane for splitting
        //Cross Edge: Edge between current intersection point and last intersection point
        //Side Edge: Split edges on the edge of the current intersection point
        void reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Vertex* &lastVertex, Edge* &leftCrossEdge, Edge* &rightCrossEdge, Edge* &leftSideEdge, Edge* &rightSideEdge, vec3 normal);

    private:
        //Helper functions for checking intersection
        double triArea(vec3 a, vec3 b, vec3 c);
        bool isInside(Face* face, vec3 point);
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

        //Linking edges to vertices if not done already 
        if(vertex_list[a]->edge == NULL){
            vertex_list[a]->edge = e1;
        }
        if(vertex_list[b]->edge == NULL){
            vertex_list[b]->edge = e2;
        }
        if(vertex_list[c]->edge == NULL){
            vertex_list[c]->edge = e3;
        }

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
                    adjList[a][b] = currentSize++;
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
            if((v1->offset > 0 && v2->offset < 0) || (v1->offset < 0 && v2->offset > 0)){
                if(abs(v1->offset) > DELTA && abs(v2->offset) > DELTA){
                    intersectingEdges.push_back(i);
                }
                isEdgeIntersecting[i] = 1;
                int j = find(edge_list.begin(),edge_list.end(),edge_list[i]->twin) - edge_list.begin();
                isEdgeIntersecting[j] = 1;
            }
        }
    }
    return intersectingEdges;
}

vector<tuple<vec3,int,int>> HalfEdge::IntersectingVertices(vector<int> edges){
    vector<tuple<vec3,int,int>> intersectingVertices;
    for(int i=0;i<vertex_list.size();i++){
        if(abs(vertex_list[i]->offset) < DELTA){
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

double HalfEdge::triArea(vec3 a, vec3 b, vec3 c){
    vec3 x = b - a;
    vec3 y = c - a;
    return abs(0.5 * (x.cross(y)).norm());
}
bool HalfEdge::isInside(Face* face, vec3 point){
    vec3 a = this->vertex_list[face->indices[0]]->position;
    vec3 b = this->vertex_list[face->indices[1]]->position;
    vec3 c = this->vertex_list[face->indices[2]]->position;
    double A = this->triArea(a,b,c);
    double A1 = this->triArea(point,b,c);
    double A2 = this->triArea(point,a,c);
    double A3 = this->triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < DELTA);
}

//Remeshing 
void HalfEdge::reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Vertex* &lastVertex, Edge* &leftCrossEdge, Edge* &rightCrossEdge, Edge* &leftSideEdge, Edge* &rightSideEdge, vec3 normal){
    //Auxiliary variables
    int currentType = get<1>(intPt);
    int lastType = get<1>(lastIntPt);
    int nextType = get<1>(nextIntPt);
    bool first = (lastType == -1);
    bool last = (nextType == -1);
    int n = this->vertex_list.size();

    //New mesh entities 
    Vertex* newVertex;
    Vertex* newVertexLeft;
    Vertex* newVertexRight;
    Edge* newCrossEdgeLeft;
    Edge* newCrossEdgeRight;
    Edge* newSideEdgeLeft;
    Edge* newSideEdgeRight;
    
    //Creating new vertices and edges or using an existing vertex depending on the type of intersection point
    if(currentType == 0){
        //Intersection point is already a vertex of the mesh
        newVertexLeft = this->vertex_list[get<2>(intPt)];
        vec3 oldPos = newVertexLeft->position;
        newVertexLeft->position = oldPos - normal * EPSILON;
        newVertexRight = new Vertex(oldPos + normal * EPSILON);
        this->vertex_list.push_back(newVertexRight);
    }else if(currentType == 1){
        //Intersection point is on an edge of the mesh
        if(first){
            vec3 oldPos = get<0>(intPt);
            newVertexLeft = new Vertex(oldPos - normal * EPSILON);
            newVertexRight = new Vertex(oldPos + normal * EPSILON);
            this->vertex_list.push_back(newVertexLeft);
        }else{
            newVertexLeft = lastVertex;
            vec3 oldPos = lastVertex->position;
            newVertexLeft->position = oldPos - normal * EPSILON;
            newVertexRight = new Vertex(oldPos + normal * EPSILON);
        }
        this->vertex_list.push_back(newVertexRight);
    }else{
        //Intersection point is on a face of the mesh(in case of a tear)
        //Only case in which the vertex doesn't split
        //Such type of vertices are allotted only to the start and end points of a tear
        newVertexLeft  = newVertexRight = this->vertex_list[get<2>(intPt)];
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
        if(first){
            newIndexLeft = n;
            newIndexRight = n + 1;
        }else{
            newIndexLeft = find(this->vertex_list.begin(),this->vertex_list.end(),lastVertex) - this->vertex_list.begin();
            newIndexRight = n;
        }
    }else{
        //Only one vertex
        newIndexLeft = newIndexRight = get<2>(intPt);
    }

    //Complete Restructuring of data strcuture based on cases
    if(first){
        if(currentType == 0){
            
            //Depending on the type of the next vertex
            if(nextType == 0){
                //Current: Vertex, Next: Vertex
                Vertex* nextVertex = this->vertex_list[get<2>(nextIntPt)];
                Edge* currentEdge = newVertexLeft->edge;

                //Finding the edge between the current and the next 
                //intersection point 
                while(true){
                    if(currentEdge->twin->startVertex == nextVertex){
                        break;
                    }else if(currentEdge->twin->next == NULL){
                        break;
                    }else{
                        currentEdge = currentEdge->twin->next;
                    }
                }

                while(currentEdge->twin->startVertex != nextVertex){
                    currentEdge = currentEdge->prev->twin;
                }

                //Edge to Edge relations
                Edge* newEdge = new Edge();
                newCrossEdgeLeft = currentEdge->twin;
                newCrossEdgeRight = new Edge();
                newCrossEdgeLeft->twin = newEdge;
                newEdge->twin = newCrossEdgeLeft;
                newCrossEdgeRight->twin = currentEdge;
                currentEdge->twin = newCrossEdgeRight;

                //Edge to Vertex relations
                newEdge->startVertex = newVertexLeft;
                newCrossEdgeRight->startVertex = nextVertex;
                currentEdge->startVertex = newVertexRight;
                currentEdge->prev->twin->startVertex = newVertexRight;

                //Vertex to Edge relations
                newVertexLeft->edge = newEdge;
                newVertexRight->edge = currentEdge;

                //Face to Vertex relations
                Face* rightFace = currentEdge->face;
                for(int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }

                this->edge_list.push_back(newEdge);
                this->edge_list.push_back(newCrossEdgeRight);
            }else if(nextType == 1){
                //Current: Vertex, Next: Edge
                newVertex = new Vertex(get<0>(nextIntPt));
                this->vertex_list.push_back(newVertex);
                int newIndex = this->vertex_list.size() - 1;

                Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                if(currentEdge->prev == NULL){
                    currentEdge = currentEdge->twin->prev;
                }else{
                    if(currentEdge->prev->startVertex == newVertexLeft){
                        currentEdge = currentEdge->prev;
                    }else{
                        currentEdge = currentEdge->twin->prev;
                    }
                }
                //Now, currentEdge is the edge starting from the the current 
                //intersection point and adjacent to the cut

                //Edge to Edge relations
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                newSideEdgeLeft = currentEdge->next->twin;
                newSideEdgeRight = new Edge();

                //Twin Edges
                newCrossEdgeLeft->twin = newEdge1;
                newEdge1->twin = newCrossEdgeLeft;
                newCrossEdgeRight->twin = newEdge2;
                newEdge2->twin = newCrossEdgeRight;
                newSideEdgeLeft->twin = currentEdge->next;
                currentEdge->next->twin = newSideEdgeRight;
                newSideEdgeRight->twin = newEdge3;
                newEdge3->twin = newSideEdgeRight;

                //Next/Prev Edges
                newCrossEdgeLeft->next = currentEdge;
                newCrossEdgeLeft->prev = currentEdge->next;
                newEdge2->next = newEdge3;
                newEdge2->prev = currentEdge->prev;
                newEdge3->next = currentEdge->prev;
                newEdge3->prev = newEdge2;
                currentEdge->prev->next = newEdge2;
                currentEdge->prev->prev = newEdge3;
                currentEdge->prev = newCrossEdgeLeft;
                currentEdge->next->next = newCrossEdgeLeft;

                //Edge to vertex relations
                newCrossEdgeLeft->startVertex = newVertex;
                newCrossEdgeRight->startVertex = newVertex;
                newEdge1->startVertex = newVertexLeft;
                newEdge2->startVertex = newVertexRight;
                newEdge3->startVertex = newVertex;
                newSideEdgeLeft->startVertex = newVertex;
                newSideEdgeRight->startVertex = newEdge3->next->startVertex;
                newEdge3->next->twin->startVertex = newVertexRight;

                //Vertex to Edge relations
                newVertex->edge = newCrossEdgeLeft;
                newVertexLeft->edge = currentEdge;
                newVertexRight->edge = newEdge2;

                //Obtaining old vertex indices
                int oldIndexLeft, oldIndexRight, centreIndex;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    centreIndex = oldFace->indices[0];
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                }else if(oldFace->edge->next == currentEdge){
                    centreIndex = oldFace->indices[1];
                    oldIndexLeft = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[0];
                }else{
                    centreIndex = oldFace->indices[2];
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                }

                //Face to Vertex/Edge relations
                oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                oldFace->edge = currentEdge;
                Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                newFace->edge = newEdge2;

                //Edge to Face relations
                newCrossEdgeLeft->face = oldFace;
                newEdge2->face = newFace;
                newEdge3->face = newFace;
                newEdge3->next->face = newFace;

                //Adding the edges/faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->edge_list.push_back(newSideEdgeRight);
                this->face_list.push_back(newFace);
            }else{

            }

        }else if(currentType == 1){
            Edge* intersectingEdge = this->edge_list[get<2>(intPt)];
            if(intersectingEdge->face == NULL){
                intersectingEdge = intersectingEdge->twin;
            }
            Face* intersectingFace = intersectingEdge->face;
            int currentFaceIndex = find(this->face_list.begin(),this->face_list.end(),intersectingFace) - this->face_list.begin();

            int oldIndexLeft, oldIndexRight, oppIndex;
            Edge* currentEdge = intersectingFace->edge;
            int i = 0;
            while(currentEdge != intersectingEdge){
                i++;
                currentEdge = currentEdge->next;
            }
            oldIndexRight = intersectingFace->indices[i%3];
            oldIndexLeft = intersectingFace->indices[(i+1)%3];
            oppIndex = intersectingFace->indices[(i+2)%3];

            //Depending on the type of the next vertex
            if(nextType == 0){
                //Current: Edge, Next: Vertex

                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();

                newCrossEdgeLeft->startVertex = this->vertex_list[oppIndex];
                newCrossEdgeRight->startVertex = this->vertex_list[oppIndex]; 

                //Edge to Edge relations
                //Two new edges on the intersecting edge
                //Four on the path from this intersection point to the next
                //Two of the second type are to be passed onto the next remeshing iteration
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();

                //For new edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newCrossEdgeLeft;
                newCrossEdgeLeft->twin = newEdge3;
                newEdge4->twin = newCrossEdgeRight;
                newCrossEdgeRight->twin = newEdge4;

                newEdge1->prev = intersectingEdge->prev;
                newEdge1->next = newEdge4;
                newEdge4->next = intersectingEdge->prev;
                newEdge4->prev = newEdge1;
                newCrossEdgeLeft->next = intersectingEdge;
                newCrossEdgeLeft->prev = intersectingEdge->next;

                //For old edges
                intersectingEdge->next->next = newCrossEdgeLeft;
                intersectingEdge->prev->next = newEdge1;
                intersectingEdge->prev->prev = newEdge4;
                intersectingEdge->prev = newCrossEdgeLeft; 

                //Edge to Vertex relations
                newEdge1->startVertex = this->vertex_list[oldIndexRight];
                newEdge2->startVertex = newVertexRight;
                newEdge3->startVertex = newVertexLeft;
                newEdge4->startVertex = newVertexRight;

                //Vertex to Edge relations
                newVertexLeft->edge = newEdge3;
                newVertexRight->edge = newEdge4;

                //Face to Vertex/Edge relations
                Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                newFace->edge = newEdge4->next;
                intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                intersectingFace->edge = newCrossEdgeLeft;

                //Edge to Face relations
                newEdge1->face = newFace;
                newEdge1->next->face = newFace;
                newEdge1->next->next->face = newFace;
                newCrossEdgeLeft->face = intersectingFace;

                //Adding the new edges and faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->face_list.push_back(newFace);
            }else if(nextType == 1){
                //Current:Edge, Next:Edge
                newVertex = new Vertex(get<0>(nextIntPt));
                this->vertex_list.push_back(newVertex);
                int newIndex = this->vertex_list.size()-1;

                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7 = new Edge();
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                newSideEdgeRight = new Edge();
                newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                if(newSideEdgeLeft->face == intersectingFace){
                    newSideEdgeLeft = newSideEdgeLeft->twin;
                }
                bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                if(left){
                    //Edge to edge relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge4;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newSideEdgeRight->twin = newEdge7;
                    newEdge7->twin = newSideEdgeRight;

                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;
                    newEdge2->next = newEdge5;
                    newEdge2->prev = intersectingEdge->prev;
                    newEdge3->next = newEdge7;
                    newEdge3->prev = newEdge6;
                    newEdge5->next = intersectingEdge->prev;
                    newEdge5->prev = newEdge2;
                    newEdge6->next = newEdge3;
                    newEdge6->prev = newEdge7;
                    newEdge7->next = newEdge6;
                    newEdge7->prev = newEdge3;
                    intersectingEdge->prev->next = newEdge2;
                    intersectingEdge->prev->prev = newEdge5;
                    intersectingEdge->prev = newCrossEdgeLeft;
                    intersectingEdge->next->next = newCrossEdgeLeft;

                    //Edge to vertex relations
                    newEdge1->startVertex = newVertexRight;
                    newEdge2->startVertex = this->vertex_list[oldIndexRight];
                    newEdge3->startVertex = newVertexRight;
                    newEdge4->startVertex = newVertexLeft;
                    newEdge5->startVertex = newVertexRight;
                    newEdge6->startVertex = this->vertex_list[oppIndex];
                    newEdge7->startVertex = newVertex;
                    newSideEdgeRight->startVertex = this->vertex_list[oppIndex];
                    newSideEdgeLeft->startVertex = newVertex;
                    newCrossEdgeLeft->startVertex = newVertex;
                    newCrossEdgeRight->startVertex = newVertex;

                    //Vertex to edge relations
                    newVertex->edge = newCrossEdgeLeft;
                    newVertexLeft->edge = intersectingEdge;
                    newVertexRight->edge = newEdge5;

                    //Face to Vertex/Edge relations
                    Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                    newFaceTop->edge = newEdge5;
                    Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                    newFaceBot->edge = newEdge3;
                    intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                    intersectingFace->edge = intersectingEdge->next;

                    //Edge to Face relations
                    newEdge2->face = newFaceTop;
                    newEdge3->face = newFaceBot;
                    newEdge5->face = newFaceTop;
                    newEdge6->face = newFaceBot;
                    newEdge7->face = newFaceBot;
                    newCrossEdgeLeft->face = intersectingFace;
                    newEdge5->next->face = newFaceTop;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);   
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFaceTop);
                    this->face_list.push_back(newFaceBot);
                }else{
                    //Edge to edge relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge4;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newSideEdgeRight->twin = newEdge7;
                    newEdge7->twin = newSideEdgeRight;

                    newCrossEdgeLeft->next = newEdge6;
                    newCrossEdgeLeft->prev = intersectingEdge->prev;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = newEdge7;
                    newEdge3->next = newEdge7;
                    newEdge3->prev = newEdge2;
                    newEdge5->next = intersectingEdge;
                    newEdge5->prev = intersectingEdge->next;
                    newEdge6->next = intersectingEdge->prev;
                    newEdge6->prev = newCrossEdgeLeft;
                    newEdge7->next = newEdge2;
                    newEdge7->prev = newEdge3;
                    intersectingEdge->prev->next = newCrossEdgeLeft;
                    intersectingEdge->prev->prev = newEdge6;
                    intersectingEdge->prev = newEdge5;
                    intersectingEdge->next->next = newEdge5;

                    //Edge to vertex relations
                    newEdge1->startVertex = newVertexRight;
                    newEdge2->startVertex = this->vertex_list[oldIndexRight];
                    newEdge3->startVertex = newVertexRight;
                    newEdge4->startVertex = newVertexLeft;
                    newEdge5->startVertex = this->vertex_list[oppIndex];
                    newEdge6->startVertex = newVertexLeft;
                    newEdge7->startVertex = newVertex;
                    newSideEdgeRight->startVertex = this->vertex_list[oldIndexRight];
                    newSideEdgeLeft->startVertex = newVertex;
                    newCrossEdgeLeft->startVertex = newVertex;
                    newCrossEdgeRight->startVertex = newVertex;

                    //Vertex to edge relations
                    newVertex->edge = newCrossEdgeLeft;
                    newVertexLeft->edge = intersectingEdge;
                    newVertexRight->edge = newEdge3;

                    //Face to Vertex/Edge relations
                    Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                    newFaceLeft->edge = newEdge6;
                    Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                    newFaceRight->edge = newEdge3;
                    intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                    intersectingFace->edge = intersectingEdge->next;

                    //Edge to Face relations
                    newEdge2->face = newFaceRight;
                    newEdge3->face = newFaceRight;
                    newEdge5->face = intersectingFace;
                    newEdge6->face = newFaceLeft;
                    newEdge7->face = newFaceRight;
                    newCrossEdgeLeft->face = newFaceLeft;
                    newEdge6->next->face = newFaceLeft;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);   
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFaceLeft);
                    this->face_list.push_back(newFaceRight);
                }
            }else{

            }
        }else{
            //Determining the face containing the first intersection point
            if(nextType == 0){
                //Current:Face, Next: Vertex
                Vertex* nextVertex = this->vertex_list[get<2>(nextIntPt)];
                Edge* currentEdge = nextVertex->edge;
                while(currentEdge->twin->next != NULL){
                    currentEdge = currentEdge->twin->next;
                }

                //Finding the face which contains the current intersection point
                while(true){
                    Face* currentFace = currentEdge->face;
                    if(isInside(currentFace, get<0>(intPt))){
                        break;
                    }else{
                        currentEdge = currentEdge->prev->twin;
                    }
                }

                //Edge to edge relations
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();

                //Twin Edges
                newEdge1->twin = newCrossEdgeRight;
                newCrossEdgeRight->twin = newEdge1;
                newEdge2->twin = newCrossEdgeLeft;
                newCrossEdgeLeft->twin = newEdge2;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;

                //Next/Prev Edges
                newCrossEdgeLeft->next = newEdge6;
                newCrossEdgeLeft->prev = currentEdge->prev;
                newEdge1->next = currentEdge;
                newEdge1->prev = newEdge3;
                newEdge3->next = newEdge1;
                newEdge3->prev = currentEdge;
                newEdge4->next = currentEdge->next;
                newEdge4->prev = newEdge5;
                newEdge5->next = newEdge4;
                newEdge5->prev = currentEdge->next;
                newEdge6->next = currentEdge->prev;
                newEdge6->prev = newCrossEdgeLeft;
                currentEdge->prev->next = newCrossEdgeLeft;
                currentEdge->prev->prev = newEdge6;
                currentEdge->next->next = newEdge5;
                currentEdge->next->prev = newEdge4;
                currentEdge->prev = newEdge1;
                currentEdge->next = newEdge3;

                //Edge to vertex relations
                newEdge1->startVertex = newVertexLeft;
                newEdge2->startVertex = newVertexLeft;
                newEdge3->startVertex = vertexTwo;
                newEdge4->startVertex = newVertexLeft;
                newEdge5->startVertex = vertexThree;
                newEdge6->startVertex = newVertexLeft;
                newCrossEdgeLeft->startVertex = vertexOne;
                newCrossEdgeRight->startVertex = vertexOne;

                //Vertex to edge relations
                newVertexLeft->edge = newEdge4;

                //Face to Vertex/Edge relations

            }else if(nextType == 1){
                //Current:Face, Next: Edge
            }else{
                //When the cut never leaves the face(Special Case)
            }
        }
    }else if(last){
        //The last intersection point
        if(currentType == 2){

        }else{
            Edge* currentCrossEdge = rightCrossEdge;
            while(currentCrossEdge->twin->next != NULL){
                currentCrossEdge->startVertex = newVertexRight;
                Face* rightFace = currentCrossEdge->twin->face;
                for(int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                    }
                }
                currentCrossEdge = currentCrossEdge->twin->next;
            }
            currentCrossEdge->startVertex = newVertexRight;
            newVertexRight->edge = rightCrossEdge;
        }
    }else{
        //A middle intersection point
        if(lastType == 0){
            if(currentType == 0){
                if(nextType == 0){
                    //Last: Vertex, Current: Vertex, Next: Vertex
                    Vertex* nextVertex = this->vertex_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newVertexLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startVertex == nextVertex){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startVertex != nextVertex){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startVertex = newVertexRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startVertex = newVertexRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Vertex relations
                    newEdge->startVertex = newVertexLeft;
                    newCrossEdgeRight->startVertex = nextVertex;
                    currentEdge->startVertex = newVertexRight;
                    currentEdge->prev->twin->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertexLeft->edge = newEdge;
                    newVertexRight->edge = currentEdge;

                    //Face to Vertex relations
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last:Vertex, Current: Vertex, Next: Edge
                    newVertex = new Vertex(get<0>(nextIntPt));
                    this->vertex_list.push_back(newVertex);
                    int newIndex = this->vertex_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startVertex == newVertexLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startVertex = newVertexRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startVertex = newVertexRight;

                    //Obtaining old vertex indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeRight;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to vertex relations
                    newCrossEdgeLeft->startVertex = newVertex;
                    newCrossEdgeRight->startVertex = newVertex;
                    newEdge1->startVertex = newVertexLeft;
                    newEdge2->startVertex = newVertexRight;
                    newEdge3->startVertex = newVertex;
                    newSideEdgeLeft->startVertex = newVertex;
                    newSideEdgeRight->startVertex = newEdge3->next->startVertex;
                    newEdge3->next->twin->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertex->edge = newCrossEdgeLeft;
                    newVertexLeft->edge = currentEdge;
                    newVertexRight->edge = newEdge2;

                    //Face to Vertex/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{

                }
            }else{
                Edge* intersectingEdge = leftSideEdge;
                Face* intersectingFace = intersectingEdge->face;
                int currentFaceIndex = find(this->face_list.begin(),this->face_list.end(),intersectingFace) - this->face_list.begin();

                int oldIndexLeft, oldIndexRight, oppIndex;
                Edge* currentEdge = intersectingFace->edge;
                int i = 0;
                while(currentEdge != intersectingEdge){
                    i++;
                    currentEdge = currentEdge->next;
                }
                oldIndexRight = intersectingFace->indices[i%3];
                oldIndexLeft = intersectingFace->indices[(i+1)%3];
                oppIndex = intersectingFace->indices[(i+2)%3];

                if(nextType == 0){
                    //Last:Vertex, Current: Edge, Next: Vertex

                    rightCrossEdge->startVertex = newVertexRight;
                    rightSideEdge->twin->startVertex = newVertexRight;

                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();

                    newCrossEdgeLeft->startVertex = this->vertex_list[oppIndex];
                    newCrossEdgeRight->startVertex = this->vertex_list[oppIndex]; 

                    //Edge to Edge relations
                    //Two new edges on the intersecting edge
                    //Four on the path from this intersection point to the next
                    //Two of the second type are to be passed onto the next remeshing iteration
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();

                    //For new edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge4;

                    newEdge1->prev = intersectingEdge->prev;
                    newEdge1->next = newEdge4;
                    newEdge4->next = intersectingEdge->prev;
                    newEdge4->prev = newEdge1;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;

                    //For old edges
                    intersectingEdge->next->next = newCrossEdgeLeft;
                    intersectingEdge->prev->next = newEdge1;
                    intersectingEdge->prev->prev = newEdge4;
                    intersectingEdge->prev = newCrossEdgeLeft; 

                    //Edge to Vertex relations
                    newEdge1->startVertex = this->vertex_list[oldIndexRight];
                    newEdge2->startVertex = newVertexRight;
                    newEdge3->startVertex = newVertexLeft;
                    newEdge4->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertexLeft->edge = newEdge3;
                    newVertexRight->edge = newEdge4;

                    //Face to Vertex/Edge relations
                    Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                    newFace->edge = newEdge4->next;
                    intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                    intersectingFace->edge = newCrossEdgeLeft;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newEdge1->face = newFace;
                    newEdge1->next->face = newFace;
                    newEdge1->next->next->face = newFace;
                    newCrossEdgeLeft->face = intersectingFace;

                    //Adding the new edges and faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFace);

                }else if(nextType == 1){
                    //Last:Vertex, Current: Edge, Next: Edge
                    rightCrossEdge->startVertex = newVertexRight;
                    rightSideEdge->twin->startVertex = newVertexRight;

                    newVertex = new Vertex(get<0>(nextIntPt));
                    this->vertex_list.push_back(newVertex);
                    int newIndex = this->vertex_list.size()-1;

                    Edge* newEdge1 = rightSideEdge->twin;
                    Edge* newEdge2 = rightSideEdge;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeRight = new Edge();
                    newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                    if(newSideEdgeLeft->face == intersectingFace){
                        newSideEdgeLeft = newSideEdgeLeft->twin;
                    }
                    bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                    if(left){
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = intersectingEdge;
                        newCrossEdgeLeft->prev = intersectingEdge->next;
                        newEdge2->next = newEdge5;
                        newEdge2->prev = intersectingEdge->prev;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge6;
                        newEdge5->next = intersectingEdge->prev;
                        newEdge5->prev = newEdge2;
                        newEdge6->next = newEdge3;
                        newEdge6->prev = newEdge7;
                        newEdge7->next = newEdge6;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newEdge2;
                        intersectingEdge->prev->prev = newEdge5;
                        intersectingEdge->prev = newCrossEdgeLeft;
                        intersectingEdge->next->next = newCrossEdgeLeft;

                        //Edge to vertex relations
                        newEdge1->startVertex = newVertexRight;
                        newEdge2->startVertex = this->vertex_list[oldIndexRight];
                        newEdge3->startVertex = newVertexRight;
                        newEdge4->startVertex = newVertexLeft;
                        newEdge5->startVertex = newVertexRight;
                        newEdge6->startVertex = this->vertex_list[oppIndex];
                        newEdge7->startVertex = newVertex;
                        newSideEdgeRight->startVertex = this->vertex_list[oppIndex];
                        newSideEdgeLeft->startVertex = newVertex;
                        newCrossEdgeLeft->startVertex = newVertex;
                        newCrossEdgeRight->startVertex = newVertex;

                        //Vertex to edge relations
                        newVertex->edge = newCrossEdgeLeft;
                        newVertexLeft->edge = intersectingEdge;
                        newVertexRight->edge = newEdge5;

                        //Face to Vertex/Edge relations
                        Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                        newFaceTop->edge = newEdge5;
                        Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                        newFaceBot->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceTop;
                        newEdge3->face = newFaceBot;
                        newEdge5->face = newFaceTop;
                        newEdge6->face = newFaceBot;
                        newEdge7->face = newFaceBot;
                        newCrossEdgeLeft->face = intersectingFace;
                        newEdge5->next->face = newFaceTop;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceTop);
                        this->face_list.push_back(newFaceBot);
                    }else{
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = newEdge6;
                        newCrossEdgeLeft->prev = intersectingEdge->prev;
                        newEdge2->next = newEdge3;
                        newEdge2->prev = newEdge7;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge2;
                        newEdge5->next = intersectingEdge;
                        newEdge5->prev = intersectingEdge->next;
                        newEdge6->next = intersectingEdge->prev;
                        newEdge6->prev = newCrossEdgeLeft;
                        newEdge7->next = newEdge2;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newCrossEdgeLeft;
                        intersectingEdge->prev->prev = newEdge6;
                        intersectingEdge->prev = newEdge5;
                        intersectingEdge->next->next = newEdge5;

                        //Edge to vertex relations
                        newEdge1->startVertex = newVertexRight;
                        newEdge2->startVertex = this->vertex_list[oldIndexRight];
                        newEdge3->startVertex = newVertexRight;
                        newEdge4->startVertex = newVertexLeft;
                        newEdge5->startVertex = this->vertex_list[oppIndex];
                        newEdge6->startVertex = newVertexLeft;
                        newEdge7->startVertex = newVertex;
                        newSideEdgeRight->startVertex = this->vertex_list[oldIndexRight];
                        newSideEdgeLeft->startVertex = newVertex;
                        newCrossEdgeLeft->startVertex = newVertex;
                        newCrossEdgeRight->startVertex = newVertex;

                        //Vertex to edge relations
                        newVertex->edge = newCrossEdgeLeft;
                        newVertexLeft->edge = intersectingEdge;
                        newVertexRight->edge = newEdge3;

                        //Face to Vertex/Edge relations
                        Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                        newFaceLeft->edge = newEdge6;
                        Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                        newFaceRight->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceRight;
                        newEdge3->face = newFaceRight;
                        newEdge5->face = intersectingFace;
                        newEdge6->face = newFaceLeft;
                        newEdge7->face = newFaceRight;
                        newCrossEdgeLeft->face = newFaceLeft;
                        newEdge6->next->face = newFaceLeft;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceLeft);
                        this->face_list.push_back(newFaceRight);
                    }
                }else{

                }
            }
        }else if(lastType == 1){
            if(currentType == 0){
                if(nextType == 0){
                    //Last:Edge, Current: Vertex, Next: Vertex

                    Vertex* nextVertex = this->vertex_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newVertexLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startVertex == nextVertex){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startVertex != nextVertex){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startVertex = newVertexRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startVertex = newVertexRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Vertex relations
                    newEdge->startVertex = newVertexLeft;
                    newCrossEdgeRight->startVertex = nextVertex;
                    currentEdge->startVertex = newVertexRight;
                    currentEdge->prev->twin->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertexLeft->edge = newEdge;
                    newVertexRight->edge = currentEdge;

                    //Face to Vertex relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last:Edge, Current: Vertex, Next: Edge
                    newVertex = new Vertex(get<0>(nextIntPt));
                    this->vertex_list.push_back(newVertex);
                    int newIndex = this->vertex_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startVertex == newVertexLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startVertex = newVertexRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startVertex = newVertexRight;

                    //Obtaining old vertex indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeRight;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to vertex relations
                    newCrossEdgeLeft->startVertex = newVertex;
                    newCrossEdgeRight->startVertex = newVertex;
                    newEdge1->startVertex = newVertexLeft;
                    newEdge2->startVertex = newVertexRight;
                    newEdge3->startVertex = newVertex;
                    newSideEdgeLeft->startVertex = newVertex;
                    newSideEdgeRight->startVertex = newEdge3->next->startVertex;
                    newEdge3->next->twin->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertex->edge = newCrossEdgeLeft;
                    newVertexLeft->edge = currentEdge;
                    newVertexRight->edge = newEdge2;

                    //Face to Vertex/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{

                }
            }else{
                Edge* intersectingEdge = leftSideEdge;
                Face* intersectingFace = intersectingEdge->face;
                int currentFaceIndex = find(this->face_list.begin(),this->face_list.end(),intersectingFace) - this->face_list.begin();

                int oldIndexLeft, oldIndexRight, oppIndex;
                Edge* currentEdge = intersectingFace->edge;
                int i = 0;
                while(currentEdge != intersectingEdge){
                    i++;
                    currentEdge = currentEdge->next;
                }
                oldIndexRight = intersectingFace->indices[i%3];
                oldIndexLeft = intersectingFace->indices[(i+1)%3];
                oppIndex = intersectingFace->indices[(i+2)%3];

                if(nextType == 0){
                    //Last:Edge, Current: Edge, Next: Vertex

                    rightCrossEdge->startVertex = newVertexRight;
                    rightSideEdge->twin->startVertex = newVertexRight;

                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();

                    newCrossEdgeLeft->startVertex = this->vertex_list[oppIndex];
                    newCrossEdgeRight->startVertex = this->vertex_list[oppIndex]; 

                    //Edge to Edge relations
                    //Two new edges on the intersecting edge
                    //Four on the path from this intersection point to the next
                    //Two of the second type are to be passed onto the next remeshing iteration
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();

                    //For new edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge4;

                    newEdge1->prev = intersectingEdge->prev;
                    newEdge1->next = newEdge4;
                    newEdge4->next = intersectingEdge->prev;
                    newEdge4->prev = newEdge1;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;

                    //For old edges
                    intersectingEdge->next->next = newCrossEdgeLeft;
                    intersectingEdge->prev->next = newEdge1;
                    intersectingEdge->prev->prev = newEdge4;
                    intersectingEdge->prev = newCrossEdgeLeft; 

                    //Edge to Vertex relations
                    newEdge1->startVertex = this->vertex_list[oldIndexRight];
                    newEdge2->startVertex = newVertexRight;
                    newEdge3->startVertex = newVertexLeft;
                    newEdge4->startVertex = newVertexRight;

                    //Vertex to Edge relations
                    newVertexLeft->edge = newEdge3;
                    newVertexRight->edge = newEdge4;

                    //Face to Vertex/Edge relations
                    Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                    newFace->edge = newEdge4->next;
                    intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                    intersectingFace->edge = newCrossEdgeLeft;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newEdge1->face = newFace;
                    newEdge1->next->face = newFace;
                    newEdge1->next->next->face = newFace;
                    newCrossEdgeLeft->face = intersectingFace;

                    //Adding the new edges and faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFace);

                }else if(nextType == 1){
                    //Last:Edge, Current: Edge, Next: Edge

                    rightCrossEdge->startVertex = newVertexRight;
                    rightSideEdge->twin->startVertex = newVertexRight;

                    newVertex = new Vertex(get<0>(nextIntPt));
                    this->vertex_list.push_back(newVertex);
                    int newIndex = this->vertex_list.size()-1;

                    Edge* newEdge1 = rightSideEdge->twin;
                    Edge* newEdge2 = rightSideEdge;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeRight = new Edge();
                    newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                    if(newSideEdgeLeft->face == intersectingFace){
                        newSideEdgeLeft = newSideEdgeLeft->twin;
                    }
                    bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                    if(left){
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = intersectingEdge;
                        newCrossEdgeLeft->prev = intersectingEdge->next;
                        newEdge2->next = newEdge5;
                        newEdge2->prev = intersectingEdge->prev;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge6;
                        newEdge5->next = intersectingEdge->prev;
                        newEdge5->prev = newEdge2;
                        newEdge6->next = newEdge3;
                        newEdge6->prev = newEdge7;
                        newEdge7->next = newEdge6;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newEdge2;
                        intersectingEdge->prev->prev = newEdge5;
                        intersectingEdge->prev = newCrossEdgeLeft;
                        intersectingEdge->next->next = newCrossEdgeLeft;

                        //Edge to vertex relations
                        newEdge1->startVertex = newVertexRight;
                        newEdge2->startVertex = this->vertex_list[oldIndexRight];
                        newEdge3->startVertex = newVertexRight;
                        newEdge4->startVertex = newVertexLeft;
                        newEdge5->startVertex = newVertexRight;
                        newEdge6->startVertex = this->vertex_list[oppIndex];
                        newEdge7->startVertex = newVertex;
                        newSideEdgeRight->startVertex = this->vertex_list[oppIndex];
                        newSideEdgeLeft->startVertex = newVertex;
                        newCrossEdgeLeft->startVertex = newVertex;
                        newCrossEdgeRight->startVertex = newVertex;

                        //Vertex to edge relations
                        newVertex->edge = newCrossEdgeLeft;
                        newVertexLeft->edge = intersectingEdge;
                        newVertexRight->edge = newEdge5;

                        //Face to Vertex/Edge relations
                        Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                        newFaceTop->edge = newEdge5;
                        Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                        newFaceBot->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceTop;
                        newEdge3->face = newFaceBot;
                        newEdge5->face = newFaceTop;
                        newEdge6->face = newFaceBot;
                        newEdge7->face = newFaceBot;
                        newCrossEdgeLeft->face = intersectingFace;
                        newEdge5->next->face = newFaceTop;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceTop);
                        this->face_list.push_back(newFaceBot);
                    }else{
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = newEdge6;
                        newCrossEdgeLeft->prev = intersectingEdge->prev;
                        newEdge2->next = newEdge3;
                        newEdge2->prev = newEdge7;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge2;
                        newEdge5->next = intersectingEdge;
                        newEdge5->prev = intersectingEdge->next;
                        newEdge6->next = intersectingEdge->prev;
                        newEdge6->prev = newCrossEdgeLeft;
                        newEdge7->next = newEdge2;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newCrossEdgeLeft;
                        intersectingEdge->prev->prev = newEdge6;
                        intersectingEdge->prev = newEdge5;
                        intersectingEdge->next->next = newEdge5;

                        //Edge to vertex relations
                        newEdge1->startVertex = newVertexRight;
                        newEdge2->startVertex = this->vertex_list[oldIndexRight];
                        newEdge3->startVertex = newVertexRight;
                        newEdge4->startVertex = newVertexLeft;
                        newEdge5->startVertex = this->vertex_list[oppIndex];
                        newEdge6->startVertex = newVertexLeft;
                        newEdge7->startVertex = newVertex;
                        newSideEdgeRight->startVertex = this->vertex_list[oldIndexRight];
                        newSideEdgeLeft->startVertex = newVertex;
                        newCrossEdgeLeft->startVertex = newVertex;
                        newCrossEdgeRight->startVertex = newVertex;

                        //Vertex to edge relations
                        newVertex->edge = newCrossEdgeLeft;
                        newVertexLeft->edge = intersectingEdge;
                        newVertexRight->edge = newEdge3;

                        //Face to Vertex/Edge relations
                        Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                        newFaceLeft->edge = newEdge6;
                        Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                        newFaceRight->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceRight;
                        newEdge3->face = newFaceRight;
                        newEdge5->face = intersectingFace;
                        newEdge6->face = newFaceLeft;
                        newEdge7->face = newFaceRight;
                        newCrossEdgeLeft->face = newFaceLeft;
                        newEdge6->next->face = newFaceLeft;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceLeft);
                        this->face_list.push_back(newFaceRight);
                    }
                }else{
                    
                }
            }
        }else{

        }
    }

    //Reallocating vertices and edges for next intersection point
    lastVertex = newVertex;
    leftCrossEdge = newCrossEdgeLeft;
    rightCrossEdge = newCrossEdgeRight;
    leftSideEdge = newSideEdgeLeft;
    rightSideEdge = newSideEdgeRight;
}


#endif