#define _CRT_SECURE_NO_WARNINGS

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader_raytrace.h"
#include "camera.h"
#include "model.h"
#include "bvh.h"

std::vector<BVH_node> bvh_nodes;
std::vector<BVH_GLSL_node> bvh_glsl_nodes;

// Save rendered image to PNG
void saveImage(const char* filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // Flip vertically because OpenGL's origin is bottom-left
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width * 3; ++x) {
            std::swap(pixels[y * width * 3 + x], pixels[(height - y - 1) * width * 3 + x]);
        }
    }

    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

GLuint loadTexture(std::string texture_path)
{
    GLuint textureID;
    glGenTextures(1, &textureID);

    int width, height, nrChannels;
    unsigned char* data = stbi_load(texture_path.c_str(), &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format;
        if (nrChannels == 1)
            format = GL_RED;
        else if (nrChannels == 3)
            format = GL_RGB;
        else if (nrChannels == 4)
            format = GL_RGBA;

        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << texture_path << std::endl;
        stbi_image_free(data);
    }

    return textureID;

}

GLuint loadCubemap(std::string skybox_path, std::vector<std::string> faces)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char* data = stbi_load((skybox_path + faces[i]).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
            );
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap texture failed to load at path: " << skybox_path + faces[i] << std::endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}

std::vector<std::string> faces
{
    "right.jpg",
    "left.jpg",
    "top.jpg",
    "bottom.jpg",
    "front.jpg",
    "back.jpg"
};

int main() {

    auto t1 = std::chrono::high_resolution_clock::now();
    int num_frame = 150;
    std::stringstream ss1;
    ss1 << std::setw(6) << std::setfill('0') << num_frame * 83;
    std::string file_num;
    ss1 >> file_num;
    std::stringstream ss2;
    ss2 << std::setw(4) << std::setfill('0') << num_frame;
    std::string output_num;
    ss2 >> output_num;
    std::cout << file_num << std::endl;
    std::string obj_path = "fluid_obj\\" + file_num + "_simplify.obj";
    std::string output_path = "frame\\" + output_num + ".png";
    
    // camera setting
    // ------------------------------
    // camera cam(glm::vec3(0, 0, 0), glm::vec3(0, 0, -1), 1280, 16.0f / 9.0f, 10, 1.0f, 80.0f, 200);
    camera cam(glm::vec3(5, 3, 9), glm::vec3(5, 2, 0), 640, 16.0f / 9.0f, 6, 10.0f, 80.0f, 36);

    // glfw: initialize and configure
    // ------------------------------
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window (off) creation
    // --------------------
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(cam.image_width, cam.image_height, "Offscreen", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    GLint maxSize = 0;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxSize);
    std::cout << "Max Texture Buffer Size: " << maxSize << std::endl;

    std::string skybox_path = "skybox\\ash\\";
    GLuint skybox_texture = loadCubemap(skybox_path, faces);

    std::string texture_path = "earthmap.jpg";
    GLuint earth_texture = loadTexture(texture_path);
    float rotation_angle = glm::radians(360.f / 150.f * num_frame);

    std::vector<Object> objectList;
    
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(-2, 0, -2), glm::vec3(-2, 0, 12), glm::vec3(12, 0, -2), 0.0f, {0.0f, 0.0f, glm::vec3(0.73, 0.73, 0.73), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(12, 0, 12), glm::vec3(12, 0, -2), glm::vec3(-2, 0, 12), 0.0f, {0.0f, 0.0f, glm::vec3(0.73, 0.73, 0.73), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(0, 0, 0), glm::vec3(12, 0, 0), glm::vec3(0, 5, 0), 0.0f, {1.0f, 0.0f, glm::vec3(0.96, 1, 0.86), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(12, 5, 0), glm::vec3(0, 5, 0), glm::vec3(12, 0, 0), 0.0f, {1.0f, 0.0f, glm::vec3(0.96, 1, 0.86), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(-2, 0, 10), glm::vec3(10, 0, 10), glm::vec3(-2, 5, 10), 0.0f, {1.0f, 0.0f, glm::vec3(0.96, 1, 0.86), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 1.0f, 0.0f, glm::vec3(10, 5, 10), glm::vec3(-2, 5, 10), glm::vec3(10, 0, 10), 0.0f, {1.0f, 0.0f, glm::vec3(0.96, 1, 0.86), 0.0f, 0.0f, 0.0f} });

    objectList.push_back({ 0.0f, 0.0f, glm::vec3(8, 1, 4), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 0.8f, {4.0f, 0.0f, glm::vec3(0.73, 0.73, 0.73), 0.0f, 0.0f, 0.0f} });
    objectList.push_back({ 0.0f, 0.0f, glm::vec3(-1, 5, 1), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 0.9f, {3.0f, 0.0f, glm::vec3(0.9, 0.95, 0.73), 0.0f, 0.0f, 0.0f} });
    

    glm::vec3 pos_offset = glm::vec3(0.0, 0.0, 2.5);

    model letter_L("mesh_object_0.obj");
    for (int i = 0; i < letter_L.num_triangles(); i++) {
        objectList.push_back({ 1.0f, 0.0f, letter_L.triangle_vertex(i, 0) + pos_offset, letter_L.triangle_vertex(i, 1) + pos_offset, letter_L.triangle_vertex(i, 2) + pos_offset, 0.0f, {1.0f, 0.0f, glm::vec3(0.3, 0.86, 0.43), 0.4f, 0.0f, 0.0f} });
    }

    model letter_T("mesh_object_1.obj");
    for (int i = 0; i < letter_T.num_triangles(); i++) {
        objectList.push_back({ 1.0f, 0.0f, letter_T.triangle_vertex(i, 0) + pos_offset, letter_T.triangle_vertex(i, 1) + pos_offset, letter_T.triangle_vertex(i, 2) + pos_offset, 0.0f, {1.0f, 0.0f, glm::vec3(0.93, 0.21, 0.32), 0.95f, 0.0f, 0.0f} });
    }

    model letter_H("mesh_object_2.obj");
    for (int i = 0; i < letter_H.num_triangles(); i++) {
        objectList.push_back({ 1.0f, 0.0f, letter_H.triangle_vertex(i, 0) + pos_offset, letter_H.triangle_vertex(i, 1) + pos_offset, letter_H.triangle_vertex(i, 2) + pos_offset, 0.0f, {1.0f, 0.0f, glm::vec3(0.24, 0.43, 0.89), 0.0f, 0.0f, 0.0f} });
    }

    model fluid(obj_path.c_str());
    std::cout << fluid.num_triangles() << std::endl;
    for (int i = 0; i < fluid.num_triangles(); i++) {
        objectList.push_back({ 1.0f, 0.0f, fluid.triangle_vertex(i, 0) + pos_offset, fluid.triangle_vertex(i, 1) + pos_offset, fluid.triangle_vertex(i, 2) + pos_offset, 0.0f, {2.0f, 0.0f, glm::vec3(0.63, 0.85, 0.98), 0.0f, 1.33f, 0.0f} });
    }

    GLuint objectTBO;
    glGenBuffers(1, &objectTBO);
    glBindBuffer(GL_TEXTURE_BUFFER, objectTBO);
    glBufferData(GL_TEXTURE_BUFFER, sizeof(Object) * objectList.size(), objectList.data(), GL_STATIC_DRAW);

    GLuint objectTexture;
    glGenTextures(1, &objectTexture);
    glBindTexture(GL_TEXTURE_BUFFER, objectTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, objectTBO);

    bvh_nodes.clear();
    bvh_glsl_nodes.clear();
    build_BVH(objectList, 0, objectList.size(), 16);
    std::cout << "Number of BVH nodes: " << bvh_nodes.size() << std::endl;

    for (int i = 0; i < bvh_nodes.size(); i++) {
        bvh_glsl_nodes.push_back({ float(bvh_nodes[i].num_objects), bvh_nodes[i].min_point, bvh_nodes[i].max_point, float(bvh_nodes[i].left_child), float(bvh_nodes[i].right_child), float(bvh_nodes[i].object_index)});
    }

    int num_obj = 0;
    for (int i = 0; i < bvh_nodes.size(); i++) {
        num_obj += bvh_nodes[i].num_objects;
    }
    if (num_obj != objectList.size()) {
        std::cout << "The BVH tree is wrong!" << std::endl;
    }

    GLuint BVH_TBO;
    glGenBuffers(1, &BVH_TBO);
    glBindBuffer(GL_TEXTURE_BUFFER, BVH_TBO);
    glBufferData(GL_TEXTURE_BUFFER, sizeof(BVH_GLSL_node) * bvh_glsl_nodes.size(), bvh_glsl_nodes.data(), GL_STATIC_DRAW);

    GLuint BVHTexture;
    glGenTextures(1, &BVHTexture);
    glBindTexture(GL_TEXTURE_BUFFER, BVHTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, BVH_TBO);
    
    Shader raytrace_shader("vertex.glsl", "fragment.glsl");

    raytrace_shader.use();

    raytrace_shader.setInt("skybox", 0);

    raytrace_shader.setInt("earthmap", 4);
    
    raytrace_shader.setInt("objectData", 2);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_BUFFER, objectTexture);

    raytrace_shader.setInt("BVHData", 3);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_BUFFER, BVHTexture);

    raytrace_shader.setInt("max_depth", cam.max_depth);
    raytrace_shader.setInt("sample_num", cam.sample_num);
    raytrace_shader.setVec3("cam_pos", cam.position);
    raytrace_shader.setVec3("viewport_bottom_left", cam.pixel_00_loc);
    raytrace_shader.setVec3("pixel_delta_x", cam.pixel_delta_u);
    raytrace_shader.setVec3("pixel_delta_y", cam.pixel_delta_v);
    raytrace_shader.setInt("obj_num", objectList.size());
    raytrace_shader.setFloat("rotation_angle", rotation_angle);

    auto t2 = std::chrono::high_resolution_clock::now();
    auto elapsed = t2 - t1;
    // raytrace_shader.setFloat("time", elapsed.count() / 512459.0f);
    raytrace_shader.setFloat("time", 664060800 / 512459.0f);
    // std::cout << elapsed.count() << std::endl;


    // Fullscreen quad vertices
    float quadVertices[] = {
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         1.0f,  1.0f, 0.0f,
         1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f,
    };

    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Create framebuffer for offscreen rendering
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cam.image_width, cam.image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is not complete!" << std::endl;
        return -1;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // Render to texture
    glViewport(0, 0, cam.image_width, cam.image_height);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    //raytrace_shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    // Save the result as a PNG
    saveImage(output_path.c_str(), cam.image_width, cam.image_height);

    // Clean up
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteTextures(1, &texture);

    glfwTerminate();

    auto t_end = std::chrono::high_resolution_clock::now();
    elapsed = t_end - t1;
    std::cout << elapsed.count() / 10e9 << "s";

	return 0;
}