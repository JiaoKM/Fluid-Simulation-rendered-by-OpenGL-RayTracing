#define _CRT_SECURE_NO_WARNINGS

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <vector>
#include <chrono>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader_raytrace.h"
#include "camera.h"

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

int main() {
    auto t1 = std::chrono::high_resolution_clock::now();
    
    // camera setting
    // ------------------------------
    camera cam(glm::vec3(0, 0, 0), glm::vec3(0, 0, -1), 400, 16.0f / 9.0f, 50, 1.0f, 90.0f, 100);

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
    
    int sbWidth, sbHeight, sbChannels;
    float* skyboxData = stbi_loadf("skybox\\studio_small_03_2k.hdr", &sbWidth, &sbHeight, &sbChannels, 0);

    GLuint skyboxTexture;
    glGenTextures(1, &skyboxTexture);


    Object obj_list[5] = { {0, glm::vec3(0.0, -100.5, -1.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), 100.0f, {0, glm::vec3(0.8, 0.8, 0.0), 0.0f, 0.0f}},
    {0, glm::vec3(0.0, 0.0, -1.2), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), 0.5f, {0, glm::vec3(0.1, 0.2, 0.5), 0.0f, 0.0f}},
    {0, glm::vec3(-1.0, 0.0, -1.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), 0.5f, {2, glm::vec3(1.0, 1.0, 1.0), 0.0f, 1.5f}},
    {0, glm::vec3(-1.0, 0.0, -1.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), 0.4f, {2, glm::vec3(1.0, 1.0, 1.0), 0.0f, 1.0f / 1.5f}},
    {0, glm::vec3(1.0, 0.0, -1.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), 0.5f, {1, glm::vec3(0.8, 0.6, 0.2), 1.0f, 0.0f}} };
   
    Shader raytrace_shader("vertex.glsl", "fragment.glsl");

    raytrace_shader.use();
    raytrace_shader.setInt("max_depth", cam.max_depth);
    raytrace_shader.setInt("sample_num", cam.sample_num);
    raytrace_shader.setVec3("cam_pos", cam.position);
    raytrace_shader.setVec3("viewport_bottom_left", cam.pixel_00_loc);
    raytrace_shader.setVec3("pixel_delta_x", cam.pixel_delta_u);
    raytrace_shader.setVec3("pixel_delta_y", cam.pixel_delta_v);
    raytrace_shader.setObjList(obj_list, 5);

    raytrace_shader.setInt("obj_num", 5);

    auto t2 = std::chrono::high_resolution_clock::now();
    auto elapsed = t2 - t1;
    raytrace_shader.setFloat("time", elapsed.count() / 512459.0f);

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
    //raytrace_shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    // Save the result as a PNG
    saveImage("output.png", cam.image_width, cam.image_height);

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