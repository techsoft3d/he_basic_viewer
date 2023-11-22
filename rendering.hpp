#pragma once

#include <unordered_map>
#include <vector>

#include <glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "linmath.h"

#include <A3DSDKIncludes.h>

// Represents a drawable object in the GPU.
// Created from an `A3DMeshData` instance during `he_mesh_data_to_opengl()`.
struct SceneObject {
    mat4x4  mat_transform_model;  // Transformation matrix for the object.
    GLuint  gl_vao;               // OpenGL Vertex Array Object ID.
    GLsizei gl_indices_count;     // Number of indices in the buffer.
};

// Holds information about all objects and resources for drawing.
// Uses an associative array to link each representation item to its drawable object.
struct TraverseData {
    std::vector<SceneObject> objects; // All GPU objects in the scene.

    // Associates each representation item with its corresponding `SceneObject`.
    std::unordered_map<A3DRiRepresentationItem*, std::pair<GLuint, GLsizei>> ri_to_gl;

    std::vector<GLuint> gl_vaos;      // IDs of GPU Vertex Array Objects for cleanup.
    std::vector<GLuint> gl_vbos;      // IDs of GPU Vertex Buffer Objects for cleanup.
};

////////////////////////////////////////////////////////////////////////////////
/// Window/Graphics API Functions
////////////////////////////////////////////////////////////////////////////////
void                               rendering_loop(GLFWwindow*, GLuint, const SceneObject*, size_t);
GLuint                             rendering_prepare();
GLFWwindow*                        rendering_prepare_window();
void                               rendering_error_callback(int, const char*);
void                               rendering_key_callback(GLFWwindow*, int, int, int, int);
void                               rendering_cleanup(GLuint, TraverseData*);
void                               rendering_cleanup_window(GLFWwindow*);
std::tuple<GLuint, GLuint, GLuint> rendering_to_gpu(const std::vector<GLuint>& index_buffer, const std::vector<GLdouble>& vertex_buffer);
