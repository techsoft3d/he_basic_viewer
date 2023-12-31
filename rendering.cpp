#include "rendering.hpp"

#include <cstdio>

// For cleanup purpose
static std::vector<GLuint> gl_vaos;
static std::vector<GLuint> gl_vbos;

////////////////////////////////////////////////////////////////////////////////
/// Drawing loop.
/// The functions indefinitely calls the graphics API drawing calls to display
/// all the objects generated by `he_mesh_data_to_opengl()`.
/// A simple rotation computation is called to animate the display.
void rendering_loop(GLFWwindow* window, GLuint program, const SceneObject* object_start, size_t n_objects)
{
    GLint p_location = glGetUniformLocation(program, "P");
    GLint v_location = glGetUniformLocation(program, "V");
    GLint m_location = glGetUniformLocation(program, "M");

    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        mat4x4 m, v, p;
 
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
        mat4x4_ortho(p, -150.f, 150.f, -150.f, 150.f, -1000, 1000.0);

        mat4x4_identity(v);
        float radius = 150.f;
        float angle  = glfwGetTime();
        vec3 eye     = {cosf(angle) * radius, cosf(angle / 100.0) * radius, sinf(angle) * radius};
        vec3 center  = {0.0, 0.0, 0.0};
        vec3 up      = {0.0, 1.0, 0.0};
        mat4x4_look_at(v, eye, center, up);

        glUseProgram(program);

        for(const SceneObject* object = object_start ; object <= object_start + n_objects ; ++object) {
            glUniformMatrix4fv(p_location, 1, GL_FALSE, (const GLfloat*) p);
            glUniformMatrix4fv(v_location, 1, GL_FALSE, (const GLfloat*) v);
            glUniformMatrix4fv(m_location, 1, GL_FALSE, (const GLfloat*) object->mat_transform_model);
    
            glBindVertexArray(object->gl_vao);
            glDrawElements(GL_TRIANGLES, object->gl_indices_count, GL_UNSIGNED_INT, 0);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Initialized the window using GLFW
GLFWwindow* rendering_prepare_window()
{
    glfwSetErrorCallback(rendering_error_callback);
 
    if (!glfwInit())
        exit(EXIT_FAILURE);
 
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
 
    GLFWwindow* window = glfwCreateWindow(800, 600, "MeshViewer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
 
    glfwSetKeyCallback(window, rendering_key_callback);
 
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSwapInterval(1);

    return window;
}

void rendering_error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}
 
void rendering_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

////////////////////////////////////////////////////////////////////////////////
/// Initializes OpenGL Shader program
GLuint rendering_prepare()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    const char* vertex_shader_text =
   "#version 420 core\n"
   "layout (location = 0) in vec3 vPos;\n"
   "layout (location = 1) in vec3 vNorm;\n"
   "out vec3 FragNormal;\n"
   "uniform mat4 M;\n"
   "uniform mat4 V;\n"
   "uniform mat4 P;\n"
   "void main()\n"
   "{\n"
       "gl_Position = P * V * M * vec4(vPos, 1.0f);\n"
       "FragNormal = normalize(mat3(transpose(inverse(M))) * vNorm);\n"
   "}\n";


    const char* fragment_shader_text =
   "#version 420 core\n"
   "out vec4 FragColor;\n"
   "\n"
   "in vec3 FragNormal;\n"
     "\n"
   "void main()\n"
   "{\n"
       "vec3 normal = normalize(FragNormal);\n"
       "vec3 lightDirection = normalize(vec3(1.0, 1.0, 1.0));\n"
       "float diff = max(dot(normal, lightDirection), 0.0);\n"
       "vec4 diffuseColor = vec4(0.7, 0.7, 0.7, 1.0);\n"
       "FragColor = diff * diffuseColor;\n"
   "}\n";

    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);
 
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);
 
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

////////////////////////////////////////////////////////////////////////////////
/// Takes a vector of vertices and a vector of indices and sends them to the GPU
/// returns OpenGL IDs for reference.
/// Returns value is a pair of the following:
/// - Vertex Array Object ID
/// - Vertex Buffer Object ID
/// - Index Buffer Object ID
GLuint rendering_to_gpu(const std::vector<GLuint>& index_buffer, const std::vector<GLdouble>& vertex_buffer, const std::vector<GLdouble>& normal_buffer)
{
    const GLuint gl_shader_coord_location =  0;
    const GLuint gl_shader_normal_location = 1;

    GLuint gl_vao = 0;
    glGenVertexArrays(1, &gl_vao);
    glBindVertexArray(gl_vao);

    GLuint gl_bo_vertex = 0;
    glGenBuffers(1, &gl_bo_vertex);
    glBindBuffer(GL_ARRAY_BUFFER, gl_bo_vertex);
    glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(GLdouble), vertex_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(gl_shader_coord_location);
    glVertexAttribPointer(gl_shader_coord_location, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*) 0);

    GLuint gl_bo_normal = 0;
    glGenBuffers(1, &gl_bo_normal);
    glBindBuffer(GL_ARRAY_BUFFER, gl_bo_normal);
    glBufferData(GL_ARRAY_BUFFER, normal_buffer.size() * sizeof(GLdouble), normal_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(gl_shader_normal_location);
    glVertexAttribPointer(gl_shader_normal_location, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*) 0);

    GLuint gl_bo_index = 0;
    glGenBuffers(1, &gl_bo_index);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gl_bo_index);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(GLuint), index_buffer.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    gl_vaos.push_back(gl_vao);
    gl_vbos.insert(gl_vbos.end(), {gl_bo_vertex, gl_bo_normal, gl_bo_index});

    return gl_vao;
}


 
////////////////////////////////////////////////////////////////////////////////
/// Cleans up all GLFW resources
void rendering_cleanup(GLuint program)
{
    glDeleteProgram(program);
    glDeleteVertexArrays(gl_vaos.size(), gl_vaos.data());
    glDeleteBuffers(gl_vbos.size(), gl_vbos.data());
    gl_vbos.clear();
    gl_vaos.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Cleans up all OpenGL resources
void rendering_cleanup_window(GLFWwindow* window)
{
    glfwDestroyWindow(window);
    glfwTerminate();
}

