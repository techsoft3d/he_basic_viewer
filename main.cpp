#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "linmath.h"

#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>

/* #define INPUT_FILE "/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct" */
/* #define INPUT_FILE "/pmi/PMI_Sample/asm/CV5_Assy_Sample_.CATProduct" */
#define INPUT_FILE "/prc/_micro engine.prc"

#define GL_SHADER_COORD_LOCATION  0
#define GL_SHADER_NORMAL_LOCATION 1

typedef struct {
    mat4x4  mat_transform_model;
    GLuint  gl_vao;
    GLsizei gl_indices_count;
} SceneObject;

typedef struct {
    std::vector<SceneObject> objects;

    std::unordered_map<A3DRiRepresentationItem*, std::pair<GLuint, GLsizei>> ri_to_gl;
    std::vector<GLuint> gl_vaos;
    std::vector<GLuint> gl_vbos;
} TraverseData;

A3DStatus code = A3D_SUCCESS;


A3DAsmModelFile* load_model_file(const char* path)
{
    A3DAsmModelFile*    model_file                           = 0;
    A3DRWParamsLoadData load_params;
    A3D_INITIALIZE_DATA(A3DRWParamsLoadData, load_params);
    load_params.m_sGeneral.m_bReadSolids                     = A3D_TRUE;
    load_params.m_sAssembly.m_bUseRootDirectory              = A3D_TRUE;
    load_params.m_sAssembly.m_bRootDirRecursive              = A3D_TRUE;
    load_params.m_sGeneral.m_eReadGeomTessMode               = kA3DReadGeomAndTess;
    load_params.m_sTessellation.m_eTessellationLevelOfDetail = kA3DTessLODHigh;
    load_params.m_sGeneral.m_bReadSurfaces                   = A3D_TRUE;
    load_params.m_sMultiEntries.m_bLoadDefault               = A3D_TRUE;

    code = A3DAsmModelFileLoadFromFile(path, &load_params, &model_file);
    if (code  != A3D_SUCCESS) {
        printf("Error %d while loading '%s'\n", code, path);
    }
    return model_file;
}

void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}
 
void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

GLFWwindow* glfw_prepare()
{
 
    glfwSetErrorCallback(glfw_error_callback);
 
    if (!glfwInit())
        exit(EXIT_FAILURE);
 
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
/* #ifdef __APPLE__ */
/* 	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); */
/* #endif */
 
    GLFWwindow* window = glfwCreateWindow(800, 600, "MeshViewer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
 
    glfwSetKeyCallback(window, glfw_key_callback);
 
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSwapInterval(1);

    return window;
}

GLuint gl_prepare_program()
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
 
A3DAsmPartDefinition* get_product_occurrence_part_definition(A3DAsmProductOccurrence* hnd_po)
{
    if(hnd_po == 0) {
        return 0;
    }

    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    A3DMiscTransformation* hnd_part = data_po.m_pPart;
    if (hnd_part == 0) {
        A3DAsmProductOccurrence* hnd_po_reference = data_po.m_pPrototype ? data_po.m_pPrototype : data_po.m_pExternalData;
        hnd_part = get_product_occurrence_part_definition(hnd_po_reference);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
    return hnd_part;
}

A3DMiscTransformation* get_product_occurrence_transformation(A3DAsmProductOccurrence* hnd_po)
{
    if(hnd_po == 0) {
        return 0;
    }

    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    A3DMiscTransformation* hnd_po_transformation = data_po.m_pLocation;
    if (hnd_po_transformation == 0) {
        A3DAsmProductOccurrence* hnd_po_reference = data_po.m_pPrototype ? data_po.m_pPrototype : data_po.m_pExternalData;
        hnd_po_transformation = get_product_occurrence_transformation(hnd_po_reference);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
    return hnd_po_transformation;
}

void mat4x4_from_transformation(const A3DMiscTransformation* hnd_transformation, mat4x4 mat_result)
{
    if (hnd_transformation == 0) {
        mat4x4_identity(mat_result);
    } else {
        A3DEEntityType entity_type = kA3DTypeUnknown;
        A3DEntityGetType(hnd_transformation, &entity_type);
        assert(entity_type == kA3DTypeMiscCartesianTransformation);

        A3DMiscCartesianTransformationData data;
        A3D_INITIALIZE_DATA(A3DMiscCartesianTransformationData, data);
        code = A3DMiscCartesianTransformationGet(hnd_transformation, &data);
        assert(code == A3D_SUCCESS);


        vec3 x_vector = {(float)data.m_sXVector.m_dX, (float)data.m_sXVector.m_dY, (float)data.m_sXVector.m_dZ};
        vec3 y_vector = {(float)data.m_sYVector.m_dX, (float)data.m_sYVector.m_dY, (float)data.m_sYVector.m_dZ};
        vec3 z_vector;
        vec3_mul_cross(z_vector, x_vector, y_vector);

        double mirror = ( data.m_ucBehaviour & kA3DTransformationMirror ) ? -1. : 1.;

        mat_result[0][0] = data.m_sXVector.m_dX * data.m_sScale.m_dX;
        mat_result[0][1] = data.m_sXVector.m_dY * data.m_sScale.m_dX;
        mat_result[0][2] = data.m_sXVector.m_dZ * data.m_sScale.m_dX;
        mat_result[0][3] = 0.;

        mat_result[1][0] = data.m_sYVector.m_dX * data.m_sScale.m_dY;
        mat_result[1][1] = data.m_sYVector.m_dY * data.m_sScale.m_dY;
        mat_result[1][2] = data.m_sYVector.m_dZ * data.m_sScale.m_dY;
        mat_result[1][3] = 0.;

        mat_result[2][0] = mirror * z_vector[0] * data.m_sScale.m_dZ;
        mat_result[2][1] = mirror * z_vector[1] * data.m_sScale.m_dZ;
        mat_result[2][2] = mirror * z_vector[2] * data.m_sScale.m_dZ;
        mat_result[2][3] = 0.;

        mat_result[3][0] = data.m_sOrigin.m_dX;
        mat_result[3][1] = data.m_sOrigin.m_dY;
        mat_result[3][2] = data.m_sOrigin.m_dZ;
        mat_result[3][3] = 1.;

        A3DMiscCartesianTransformationGet(0, &data);
    }
}


std::pair<GLuint, GLsizei> gl_make_buffers(A3DMeshData* const data_mesh, TraverseData* const data_traverse)
{
    std::vector<GLuint> index_buffer;
    std::vector<GLdouble> vertex_buffer;

    // Count the total number of indices
    // The buffer objects will have at max n_indices indices
    size_t n_indices = 0;
    for(A3DUns32 face_i = 0 ; face_i < data_mesh->m_uiFaceSize ; ++face_i) {
        n_indices += 3 * data_mesh->m_puiTriangleCountPerFace[face_i];
    }

    // This map will serve as cache for the OpenGL indices
    // vertex_index|normal_index -> gl_index
    std::unordered_map<uint64_t, GLuint> index_cache;

    for (A3DUns32 face_i = 0; face_i < data_mesh->m_uiFaceSize; ++face_i) {
        A3DUns32 n_triangles = data_mesh->m_puiTriangleCountPerFace[face_i];
        if (n_triangles == 0) {
            continue;
        }
        const A3DUns32* ptr_coord_index = data_mesh->m_ppuiPointIndicesPerFace[face_i];
        const A3DUns32* ptr_normal_index = data_mesh->m_ppuiNormalIndicesPerFace[face_i];

        for (size_t vertex_i = 0 ; vertex_i < n_triangles * 3 ; ++vertex_i) {
            A3DUns32 coord_index = *ptr_coord_index++;
            A3DUns32 normal_index = *ptr_normal_index++;

            uint64_t cache_key = ((uint64_t)coord_index << 32) | normal_index;
            auto insertion = index_cache.insert({cache_key, vertex_buffer.size() / 6});

            GLuint vertex_index = insertion.first->second;
            if(insertion.second) {
                auto x = data_mesh->m_pdCoords[coord_index];
                auto y = data_mesh->m_pdCoords[coord_index+1];
                auto z = data_mesh->m_pdCoords[coord_index+2];

                // Create the coordinates
                vertex_buffer.insert(vertex_buffer.end(), {
                    data_mesh->m_pdCoords[coord_index],
                    data_mesh->m_pdCoords[coord_index + 1],
                    data_mesh->m_pdCoords[coord_index + 2],
                    data_mesh->m_pdNormals[normal_index],
                    data_mesh->m_pdNormals[normal_index + 1],
                    data_mesh->m_pdNormals[normal_index + 2]
                });
            }
            index_buffer.push_back(vertex_index);
        }
    }

    GLuint gl_vao = 0;
    glGenVertexArrays(1, &gl_vao);
    glBindVertexArray(gl_vao);

    GLuint gl_bo_vertex = 0;
    glGenBuffers(1, &gl_bo_vertex);
    glBindBuffer(GL_ARRAY_BUFFER, gl_bo_vertex);
    glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(GLdouble), vertex_buffer.data(), GL_STATIC_DRAW);

    GLuint gl_bo_index = 0;
    glGenBuffers(1, &gl_bo_index);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gl_bo_index);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(GLuint), index_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(GL_SHADER_COORD_LOCATION);
    glVertexAttribPointer(GL_SHADER_COORD_LOCATION, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(GLdouble), (void*) 0);
    glEnableVertexAttribArray(GL_SHADER_NORMAL_LOCATION);
    glVertexAttribPointer(GL_SHADER_NORMAL_LOCATION, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(GLdouble), (void*) (3 * sizeof(GLdouble)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    data_traverse->gl_vaos.push_back(gl_vao);
    data_traverse->gl_vbos.push_back(gl_bo_vertex);
    data_traverse->gl_vbos.push_back(gl_bo_index);

    return {gl_vao, (GLsizei)index_buffer.size()};
}

void traverse_representation_item(A3DRiRepresentationItem* const hnd_ri, A3DMiscCascadedAttributes* const hnd_attrs_part, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{
    A3DEEntityType ri_type = kA3DTypeUnknown;
    A3DEntityGetType(hnd_ri, &ri_type);

    if (ri_type == kA3DTypeRiSet) {
        A3DRiSetData data_ri_set;
        A3D_INITIALIZE_DATA(A3DRiSetData, data_ri_set);
        code = A3DRiSetGet(hnd_ri, &data_ri_set);
        assert(code == A3D_SUCCESS);

        for(A3DUns32 ri_i = 0 ; ri_i < data_ri_set.m_uiRepItemsSize ; ++ri_i) {
            traverse_representation_item(data_ri_set.m_ppRepItems[ri_i], hnd_attrs_part, mat_transform_world, data_traverse);
        }

        A3DRiSetGet(0, &data_ri_set);
    } else {
        A3DMeshData data_mesh;
        A3D_INITIALIZE_DATA(A3DMeshData, data_mesh);
        code = A3DRiComputeMesh(hnd_ri, hnd_attrs_part, &data_mesh);
        if (code != A3D_SUCCESS)
        {
            A3DRWParamsTessellationData data_params_tess;
            A3D_INITIALIZE_DATA(A3DRWParamsTessellationData, data_params_tess);
            data_params_tess.m_eTessellationLevelOfDetail = kA3DTessLODMedium;
            A3DRiRepresentationItemComputeTessellation(hnd_ri, &data_params_tess);
            code = A3DRiComputeMesh(hnd_ri, hnd_attrs_part, &data_mesh);
        }
        assert(code == A3D_SUCCESS);

        auto gl_iterator = data_traverse->ri_to_gl.find(hnd_ri);
        if(gl_iterator == data_traverse->ri_to_gl.end()) {
            auto pair = gl_make_buffers(&data_mesh, data_traverse);
            gl_iterator = data_traverse->ri_to_gl.insert({hnd_ri, pair}).first;
        }

        SceneObject object;
        mat4x4_dup(object.mat_transform_model, mat_transform_world);
        object.gl_vao = gl_iterator->second.first;
        object.gl_indices_count = gl_iterator->second.second;
        data_traverse->objects.push_back(object);

    }
}

void traverse_part_definition(A3DAsmPartDefinition* const hnd_part, A3DMiscCascadedAttributes* const hnd_attrs_po, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{
    if(hnd_part == 0) {
        return;
    }

    A3DMiscCascadedAttributes* hnd_attrs_part = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_part);
    A3DMiscCascadedAttributesPush(hnd_attrs_part, hnd_part, hnd_attrs_po);

    A3DAsmPartDefinitionData data_part;
    A3D_INITIALIZE_DATA(A3DAsmPartDefinitionData, data_part);
    code = A3DAsmPartDefinitionGet(hnd_part, &data_part);
    assert(code == A3D_SUCCESS);

    for (A3DUns32 ri_i = 0 ; ri_i < data_part.m_uiRepItemsSize ; ++ri_i) {
        traverse_representation_item(data_part.m_ppRepItems[ri_i], hnd_attrs_part, mat_transform_world, data_traverse);
    }

    A3DAsmPartDefinitionGet(0, &data_part);
}

void traverse_product_occurrence(A3DAsmProductOccurrence* const hnd_po, A3DMiscCascadedAttributes* const hnd_attrs_parent, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{

    A3DMiscCascadedAttributes* hnd_attrs_po = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_po);
    A3DMiscCascadedAttributesPush(hnd_attrs_po, hnd_po, hnd_attrs_parent);
    A3DMiscCascadedAttributesEntityReferencePush(hnd_attrs_po, hnd_po, 0);


    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    // Accumulate the position
    A3DMiscTransformation* hnd_po_transformation = get_product_occurrence_transformation(hnd_po);
    mat4x4 mat_transform_po_local, mat_transform_po_world;
    mat4x4_from_transformation(hnd_po_transformation, mat_transform_po_local);
    mat4x4_mul(mat_transform_po_world, mat_transform_world, mat_transform_po_local);

    A3DAsmPartDefinition* hnd_part = get_product_occurrence_part_definition(hnd_po);
    traverse_part_definition(hnd_part, hnd_attrs_po, mat_transform_po_world, data_traverse);

    for (A3DUns32 po_i = 0 ; po_i < data_po.m_uiPOccurrencesSize ; ++po_i) {
        traverse_product_occurrence(data_po.m_ppPOccurrences[po_i], hnd_attrs_po, mat_transform_po_world, data_traverse);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
}

void traverse_model_file(A3DAsmModelFile* const hnd_modelfile, TraverseData* const data_traverse)
{
    A3DMiscCascadedAttributes* hnd_attrs_modelfile = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_modelfile);

    A3DAsmModelFileData data_modelfile;
    A3D_INITIALIZE_DATA(A3DAsmModelFileData, data_modelfile);
    code = A3DAsmModelFileGet(hnd_modelfile, &data_modelfile);
    assert(code == A3D_SUCCESS);

    mat4x4 mat_transform;
    mat4x4_identity(mat_transform);

    for (A3DUns32 po_i = 0 ; po_i < data_modelfile.m_uiPOccurrencesSize ; ++po_i) {
        traverse_product_occurrence(data_modelfile.m_ppPOccurrences[po_i], hnd_attrs_modelfile, mat_transform, data_traverse);
    }
    
    A3DAsmModelFileGet(0, &data_modelfile);
    A3DMiscCascadedAttributesDelete(hnd_attrs_modelfile);
}

 
void glfw_loop(GLFWwindow* window, GLuint program, const SceneObject* object_start, size_t n_objects)
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

int main(int argc, char* argv[])
{
    GLFWwindow* window  = glfw_prepare();
    GLuint      program = gl_prepare_program();

    A3DSDKLoadLibraryA(HE_BINARY_DIRECTORY);
    A3DLicPutUnifiedLicense(HOOPS_LICENSE);
    A3DDllInitialize(A3D_DLL_MAJORVERSION, A3D_DLL_MINORVERSION);

    A3DAsmModelFile* model_file = load_model_file(HE_DATA_DIRECTORY INPUT_FILE);

    TraverseData     data_traverse;
    traverse_model_file(model_file, &data_traverse);

    A3DAsmModelFileDelete(model_file);
    A3DDllTerminate();
    A3DSDKUnloadLibrary();
 
    printf("Starting Loop\n");
    glfw_loop(window, program, data_traverse.objects.data(), data_traverse.objects.size());
 
    // Clean up all resources
    glDeleteProgram(program);
    glDeleteVertexArrays(data_traverse.gl_vaos.size(), data_traverse.gl_vaos.data());
    glDeleteBuffers(data_traverse.gl_vaos.size(), data_traverse.gl_vaos.data());
    
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}
