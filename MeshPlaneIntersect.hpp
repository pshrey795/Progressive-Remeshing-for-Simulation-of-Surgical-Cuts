#ifndef INTERSECT_HPP
#define INTERSECT_HPP

#include <bits/stdc++.h>
using namespace std;


template <class FloatType, class IndexType>
class MeshPlaneIntersect {

public:
	typedef std::array<FloatType, 3> Vec3D;
	typedef std::array<IndexType, 3> Face;

	struct Plane {
		Vec3D origin;
		Vec3D normal;
		Plane(Vec3D origin, Vec3D v) {
			this->origin = origin;
			FloatType norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			this->normal = {v[0] / norm, v[1] / norm, v[2] / norm};
		}
	};

	class Mesh {
	public:
		Mesh(const std::vector<Vec3D>& vertices, const std::vector<Face>& faces) :
			vertices(vertices), faces(faces) {}

		std::vector<pair<Vec3D,int>> Intersect(const Plane& plane) const {
			return _Execute(*this, plane);
		}

	private:
		const std::vector<Vec3D>& vertices;
		const std::vector<Face>& faces;

		typedef std::pair<int, int> Edge;

		static std::vector<pair<Vec3D,int>> _Execute(const Mesh& mesh, const Plane& plane) {
			const auto vertexOffsets(VertexOffsets(mesh.vertices, plane));
			auto edges(IntersectingEdges(mesh.faces, vertexOffsets));
			return IntersectingPoints(mesh, edges, vertexOffsets);
		}

		static vector<pair<Vec3D,int>> removeDuplicates(vector<pair<Vec3D,int>> points){
			vector<pair<Vec3D,int>> result;
			for(int i=0;i<points.size();i++){
				bool flag = false;
				for(int j=0;j<result.size();j++){
					if((points[i].first)[0] == (result[j].first)[0] && (points[i].first)[1] == (result[j].first)[1] && (points[i].first)[2] == (result[j].first)[2]){
						flag = true;
						break;
					}
				}
				if(!flag){
					result.push_back(points[i]);
				}
			}
			return result;
		}

		static std::vector<pair<Vec3D,int>> IntersectingPoints(const Mesh& mesh,
			const std::vector<Edge>& edges,
			const std::vector<FloatType>& vertexOffsets) {
			vector<pair<Vec3D,int>> points;
			for(int i=0;i<mesh.vertices.size();i++){
				if(vertexOffsets[i] == 0){
					points.push_back(make_pair(mesh.vertices[i],0));
				}
			}
			for (const auto& edge : edges) {
				const auto& offset1(vertexOffsets[edge.first]);
				const auto& offset2(vertexOffsets[edge.second]);
				const auto factor = offset1 / (offset1 - offset2);
				const auto& edgeStart(mesh.vertices.at(edge.first));
				const auto& edgeEnd(mesh.vertices.at(edge.second));
				Vec3D newPoint;
				for (int i(0); i < 3; ++i) {
					newPoint[i] = edgeStart[i] + (edgeEnd[i] - edgeStart[i]) * factor;
				}
				points.push_back(make_pair(newPoint,1));
			}
			return removeDuplicates(points);
		}

		//Distance(algebraic) from vertex to plane 
		static const std::vector<FloatType> VertexOffsets(const std::vector<Vec3D>& vertices,
			const Plane& plane) {
			std::vector<FloatType> offsets;
			offsets.reserve(vertices.size());
			std::transform(vertices.begin(), vertices.end(), std::back_inserter(offsets),
				[&plane](const auto& vertex) {
					return VertexOffset(vertex, plane);
				});
			return offsets;
		}

		//Returns algebraic distance from vertex to plane
		static FloatType VertexOffset(const Vec3D& vertex, const Plane& plane) {
			FloatType offset(0);
			for (int i(0); i < 3; ++i) {
				offset += plane.normal[i] * (vertex[i] - plane.origin[i]);
			}
			return offset;
		}

		static std::vector<Edge> IntersectingEdges(const std::vector<Face>& faces,
			const std::vector<FloatType>& vertexOffsets) {
			vector<Edge> crossingEdges;
			for (const auto& face : faces) {
				const bool edge1crosses = vertexOffsets[face[0]] * vertexOffsets[face[1]] < 0;
				const bool edge2crosses = vertexOffsets[face[1]] * vertexOffsets[face[2]] < 0;
				const bool edge3crosses = vertexOffsets[face[2]] * vertexOffsets[face[0]] < 0;
				if (edge1crosses) {
					crossingEdges.push_back({face[0], face[1]});
				}
				if (edge2crosses) {
					crossingEdges.push_back({face[1], face[2]});
				}
				if (edge1crosses) {
					crossingEdges.push_back({face[2], face[0]});
				}
			}
			return crossingEdges;
		}
	};

private:
	// constructor is private, use the mesh class Interect and Clip methods
	MeshPlaneIntersect() {};
};

#endif