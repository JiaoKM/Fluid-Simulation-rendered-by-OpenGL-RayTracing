#ifndef CAMERA_H
#define CAMERA_H

#include <glm/gtc/matrix_transform.hpp>

class camera {
public:
	glm::vec3 position;
	glm::vec3 lookat;
	glm::vec3 vup = glm::vec3(0, 1, 0);
	int image_height;
	int image_width;
	int max_depth;
	int sample_num;
	float aspect_ratio;
	float vfov = 90;
	float focus_dist = 10.0f;

	glm::vec3 pixel_00_loc;
	glm::vec3 pixel_delta_u;
	glm::vec3 pixel_delta_v;

	camera(glm::vec3 pos, glm::vec3 _lookat, int width, float ratio, int depth, float focus, float fov, int sample) {
		position = pos;
		lookat = _lookat;

		image_width = width;
		aspect_ratio = ratio;
		image_height = int(image_width / aspect_ratio);
		image_height = (image_height < 1) ? 1 : image_height;

		vfov = fov;
		focus_dist = focus;
		float theta = glm::radians(vfov);
		float h = glm::tan(theta / 2);
		float viewport_height = 2 * h * focus_dist;
		float viewport_width = viewport_height * (float(image_width) / image_height);

		glm::vec3 w = glm::normalize(position - lookat);
		glm::vec3 u = glm::normalize(glm::cross(vup, w));
		glm::vec3 v = glm::cross(w, u);

		pixel_delta_u = viewport_width * u / float(image_width);
		pixel_delta_v = viewport_height * v / float(image_height);
		pixel_00_loc = position - (focus_dist * w) - viewport_width * u / 2.0f - viewport_height * v / 2.0f;

		max_depth = depth;
		sample_num = sample;
	}

};

#endif // !CAMERA_H
