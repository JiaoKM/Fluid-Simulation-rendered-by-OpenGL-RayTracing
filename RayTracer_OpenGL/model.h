#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <vector>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

struct Triangle
{
	glm::vec3 v0, v1, v2;
};

class model {
public:
	model(const char* filename) : model_path(filename) {
		Assimp::Importer importer;
		scene = importer.ReadFile(model_path, 
			aiProcess_Triangulate | 
			aiProcess_JoinIdenticalVertices | 
			aiProcess_OptimizeMeshes);

		if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
			std::cerr << "Error: " << importer.GetErrorString() << std::endl;
		}

		for (unsigned int mesh_i = 0; mesh_i < scene->mNumMeshes; mesh_i++) {
			aiMesh* mesh = scene->mMeshes[mesh_i];
			for (unsigned int face_i = 0; face_i < mesh->mNumFaces; face_i++) {
				aiFace& face = mesh->mFaces[face_i];
				if (face.mNumIndices == 3) {
					unsigned int i_0 = face.mIndices[0];
					unsigned int i_1 = face.mIndices[1];
					unsigned int i_2 = face.mIndices[2];

					aiVector3D v0 = mesh->mVertices[i_0];
					aiVector3D v1 = mesh->mVertices[i_1];
					aiVector3D v2 = mesh->mVertices[i_2];

					triangles.push_back({ glm::vec3(v0.x, v0.y, v0.z), glm::vec3(v1.x, v1.y, v1.z), glm::vec3(v2.x, v2.y, v2.z)});
				}
			}
		}
	}

	int num_triangles() {
		return triangles.size();
	}

	glm::vec3 triangle_vertex(int i, int j) {
		Triangle tmp = triangles[i];

		if (j == 0) return tmp.v0;
		else if (j == 1) return tmp.v1;
		else if (j == 2) return tmp.v2;
	}

private:
	const aiScene* scene;
	std::vector<Triangle> triangles;
	const char* model_path;
};

#endif
