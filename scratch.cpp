#include "common.hpp"
#include "MeshPlaneIntersect.hpp"
#include<bits/stdc++.h>
using namespace std;
using namespace Eigen;

typedef MeshPlaneIntersect<double, int> Intersector;

int main(int argc, char** argv){
    vector<Intersector::Vec3D> vertices = {
        {-1, 1, 0},
        {-1, -1, -1},
        {1, -1, -1},
        {2, 1, 0},
        {0, 1, 0},
        // {1, -1, 0}
    };
    vector<Intersector::Face> faces = {
        {1, 2, 4},
        {2, 3, 4},
        {1, 4, 0}
    };
    Intersector::Vec3D origin = {0.0, 0.0, 0.0};
    Intersector::Vec3D normal = {1.0, 1.0, 0.0};
    Intersector::Plane plane(origin, normal);
    Intersector::Mesh mesh(vertices, faces);
    auto result = mesh.Intersect(plane);
    for(int i=0;i<result.size();i++){
        Intersector::Vec3D points = result[i].first;
        cout << points[0] << " " << points[1] << " " << points[2] << " " << result[i].second << "\n";
    }
    return 0;
}