#ifndef BVH_H
#define BVH_H

#define INFINITY 10e8
#define EPSILON 10e-8

#include <vector>
#include <algorithm>
#include <glm/glm.hpp>

#include "shader_raytrace.h"

struct BVH_node
{
	int num_objects;
	glm::vec3 min_point;
	glm::vec3 max_point;
	int left_child;
	int right_child;
	int object_index;
};

struct BVH_GLSL_node
{
	float num_objects;
	glm::vec3 min_point;
	glm::vec3 max_point;
	float left_child;
	float right_child;
	float object_index;
	float padding1;
	float padding2;
};

struct AABB
{
	glm::vec3 min_point;
	glm::vec3 max_point;
};

extern std::vector<BVH_node> bvh_nodes;

AABB triangle_AABB(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
	glm::vec3 min_p = glm::min(v0, v1);
	min_p = glm::min(min_p, v2);
	glm::vec3 max_p = glm::max(v0, v1);
	max_p = glm::max(max_p, v2);

	if (max_p.x - min_p.x < 2 * EPSILON) {
		max_p.x += EPSILON;
		min_p.x -= EPSILON;
	}
	if (max_p.y - min_p.y < 2 * EPSILON) {
		max_p.y += EPSILON;
		min_p.y -= EPSILON;
	}
	if (max_p.z - min_p.z < 2 * EPSILON) {
		max_p.z += EPSILON;
		min_p.z -= EPSILON;
	}

	return {min_p, max_p};
}

AABB sphere_AABB(glm::vec3 center, float radius) {
	glm::vec3 min_p = glm::vec3(center.x - radius, center.y - radius, center.z - radius);
	glm::vec3 max_p = glm::vec3(center.x + radius, center.y + radius, center.z + radius);

	return { min_p, max_p };
}

AABB object_AABB(Object obj) {
	if (fabs(obj.type - 0.0f) < EPSILON) {
		return sphere_AABB(obj.position, obj.radius);
	}
	else {
		return triangle_AABB(obj.position, obj.p1, obj.p2);
	}
}

glm::vec3 get_centroid(const Object& obj) {
	if (fabs(obj.type - 0.0f) < EPSILON) {
		return obj.position;
	}
	else {
		return (obj.position + obj.p1 + obj.p2) / glm::vec3(3.0, 3.0, 3.0);
	}
}

bool compare_x(const Object& obj1, const Object& obj2) {
	glm::vec3 center1 = get_centroid(obj1);
	glm::vec3 center2 = get_centroid(obj2);
	return center1.x < center2.x;
}

bool compare_y(const Object& obj1, const Object& obj2) {
	glm::vec3 center1 = get_centroid(obj1);
	glm::vec3 center2 = get_centroid(obj2);
	return center1.y < center2.y;
}

bool compare_z(const Object& obj1, const Object& obj2) {
	glm::vec3 center1 = get_centroid(obj1);
	glm::vec3 center2 = get_centroid(obj2);
	return center1.z < center2.z;
}

int build_BVH(std::vector<Object>& object_list, int start, int end, int num) {
	if (end < start) return 0;

	bvh_nodes.push_back(BVH_node());
	int id = bvh_nodes.size() - 1;
	bvh_nodes[id].left_child = bvh_nodes[id].right_child = bvh_nodes[id].num_objects = bvh_nodes[id].object_index = 0;
	bvh_nodes[id].min_point = glm::vec3(INFINITY, INFINITY, INFINITY);
	bvh_nodes[id].max_point = glm::vec3(-INFINITY, -INFINITY, -INFINITY);

	// find the aabb
	for (int i = start; i < end; i++) {
		AABB aabb = object_AABB(object_list[i]);
		bvh_nodes[id].min_point = glm::min(bvh_nodes[id].min_point, aabb.min_point);
		bvh_nodes[id].max_point = glm::max(bvh_nodes[id].max_point, aabb.max_point);
	}

	// if the number of objects in the node is small enough, return
	if (end - start <= num) {
		bvh_nodes[id].num_objects = end - start;
		bvh_nodes[id].object_index = start;
		return id;
	}

	float len_x = bvh_nodes[id].max_point.x - bvh_nodes[id].min_point.x;
	float len_y = bvh_nodes[id].max_point.y - bvh_nodes[id].min_point.y;
	float len_z = bvh_nodes[id].max_point.z - bvh_nodes[id].min_point.z;

	// sort, but it will cause wrong rendering, need to be corrected
	/*if (len_x > len_y && len_x > len_z) {
		std::stable_sort(object_list.begin() + start, object_list.begin() + end, compare_x);
	}
	else if (len_y > len_z) {
		std::stable_sort(object_list.begin() + start, object_list.begin() + end, compare_y);
	}
	else {
		std::stable_sort(object_list.begin() + start, object_list.begin() + end, compare_z);
	}*/

	// incursive building
	int mid = start + (end - start) / 2;
	int left = build_BVH(object_list, start, mid, num);
	int right = build_BVH(object_list, mid, end, num);

	bvh_nodes[id].left_child = left;
	bvh_nodes[id].right_child = right;

	return id;
}

#endif