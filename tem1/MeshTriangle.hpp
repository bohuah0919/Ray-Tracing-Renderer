#pragma once
#include"Triangle.hpp"
#include "OBJ_Loader.hpp"
#include "BoundingBox.hpp"

class MeshTriangle : public Object {
public:
	MeshTriangle(std::string filename, bool loadNormal, bool loadTextureCoor, bool Lumin) {
		objl::Loader loader;
		loader.LoadFile(filename);
		assert(loader.LoadedMeshes.size() == 1);
		auto mesh = loader.LoadedMeshes[0];
		bool isLumin = Lumin;
		Eigen::Vector3f Vmax = -Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());
		Eigen::Vector3f Vmin = Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());

		for (int i = 0; i < mesh.Vertices.size(); i += 3) {
			Eigen::Vector3f e1 = Eigen::Vector3f(mesh.Vertices[i + 1].Position.X - mesh.Vertices[i].Position.X,
				mesh.Vertices[i + 1].Position.Y - mesh.Vertices[i].Position.Y,
				mesh.Vertices[i + 1].Position.Z - mesh.Vertices[i].Position.Z);
			Eigen::Vector3f e2 = Eigen::Vector3f(mesh.Vertices[i + 2].Position.X - mesh.Vertices[i].Position.X,
				mesh.Vertices[i + 2].Position.Y - mesh.Vertices[i].Position.Y,
				mesh.Vertices[i + 2].Position.Z - mesh.Vertices[i].Position.Z);

			Eigen::Vector3f normal = e1.normalized().cross(e2.normalized());
			Triangle* t = new Triangle(e1,e2, Lumin);
			for (int j = 0; j < 3; j++) {

				t->setVertex(j, Eigen::Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0f));
				if (loadNormal)
					t->setNormal(j, Eigen::Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				else
					t->setNormal(j, normal);

				if (loadTextureCoor)
					t->setTextureCoor(j, Eigen::Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));

				Vmax(0) = std::max(Vmax.x(), mesh.Vertices[i + j].Position.X);
				Vmax(1) = std::max(Vmax.y(), mesh.Vertices[i + j].Position.Y);
				Vmax(2) = std::max(Vmax.z(), mesh.Vertices[i + j].Position.Z);

				Vmin(0) = std::min(Vmin.x(), mesh.Vertices[i + j].Position.X);
				Vmin(1) = std::min(Vmin.y(), mesh.Vertices[i + j].Position.Y);
				Vmin(2) = std::min(Vmin.z(), mesh.Vertices[i + j].Position.Z);

			}
			triangleList.push_back(t);
		}
		aabb = BoundingBox(Vmax, Vmin);

		std::vector<Object*> objList;
		for (auto tri : triangleList) {
			objList.push_back(tri);
		}
		MeshBVH = new BVH(objList);
	}
	std::vector<Triangle*> triangleList;
	bool isLight() { return isLumin; }
	BoundingBox getBoundingBox() { return aabb;}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (MeshBVH) {
			inter = MeshBVH->getIntersection(ori, dir);
		}
		return inter;
	}
private:
	bool isLumin;
	Eigen::Vector3f Vmax, Vmin;
	BoundingBox aabb;
	BVH* MeshBVH;
};