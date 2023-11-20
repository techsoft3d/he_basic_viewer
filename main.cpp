////////////////////////////////////////////////////////////////////////////////
/// Basic Mesh Viewer                                                        ///
/// #################                                                        ///
///                                                                          ///
/// This sample provides a single source CAD viewer using HOOPS Exchange.    ///
/// The codes makes use of the traversal API and `A3DMeshData` to send mesh  ///
/// data to OpenGL Graphics API                                              ///
///                                                                          ///
/// For more information see:                                                ///
///                                                                          ///
/// * docs.techsoft3d.com/exchange/latest/tutorials/mesh-viewer-sample.html  ///
/// * docs.techsoft3d.com/exchange/latest/tutorials/basic_viewing.html       ///
////////////////////////////////////////////////////////////////////////////////
/// TODO list week 11-20-2023:
/// - Prune down he_mesh_data_to_opengl code using Usability Phase 1 Feedback
/// - Prune down traversal code using Usability Phase 2
/// - Use separate file for non-HE code
/// - Subdir the deps
/// Expected results: about 250 lines of code removal. Less algorithm, less HE
/// calls.
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// INPUT_FILE
/// Default CAD input file, relative to Exchange sample data folder.
/// To see how the value is used, check the `main()` function.
#define INPUT_FILE "/prc/_micro engine.prc"

////////////////////////////////////////////////////////////////////////////////
/// Data structures

// Represents a drawable object in the GPU.
// Created from an `A3DMeshData` instance during `he_mesh_data_to_opengl()`.
typedef struct {
    mat4x4  mat_transform_model;  // Transformation matrix for the object.
    GLuint  gl_vao;               // OpenGL Vertex Array Object ID.
    GLsizei gl_indices_count;     // Number of indices in the buffer.
} SceneObject;

// Holds information about all objects and resources for drawing.
// Uses an associative array to link each representation item to its drawable object.
typedef struct {
    std::vector<SceneObject> objects; // All GPU objects in the scene.

    // Associates each representation item with its corresponding `SceneObject`.
    std::unordered_map<A3DRiRepresentationItem*, std::pair<GLuint, GLsizei>> ri_to_gl;

    std::vector<GLuint> gl_vaos;      // IDs of GPU Vertex Array Objects for cleanup.
    std::vector<GLuint> gl_vbos;      // IDs of GPU Vertex Buffer Objects for cleanup.
} TraverseData;

////////////////////////////////////////////////////////////////////////////////
/// Exchange API Functions
/// TODO Ideally the Usability Phase 2 reduces this all to 1 function and prunes
/// the entire sample about 200 lines of code! 
////////////////////////////////////////////////////////////////////////////////
// Traversal functions, one per entity type.
void he_traverse_model_file(A3DAsmModelFile* const hnd_modelfile, TraverseData* const data_traverse);
void he_traverse_product_occurrence( A3DAsmProductOccurrence* const hnd_po, A3DMiscCascadedAttributes* const hnd_attrs_parent, const mat4x4 mat_transform_world, TraverseData* const data_traverse);
void he_traverse_part_definition(A3DAsmPartDefinition* const hnd_part, A3DMiscCascadedAttributes* const hnd_attrs_po, const mat4x4 mat_transform_world, TraverseData* const data_traverse);
void he_traverse_representation_item(A3DRiRepresentationItem* const hnd_ri, A3DMiscCascadedAttributes* const hnd_attrs_part, const mat4x4 mat_transform_world, TraverseData* const data_traverse);

// Recursively retrieve the part definition and transformation underlying a Product Occurrence
A3DAsmPartDefinition*  he_get_product_occurrence_part_definition(A3DAsmProductOccurrence* hnd_po);
A3DMiscTransformation* he_get_product_occurrence_transformation(A3DAsmProductOccurrence* hnd_po);

// Converts an A3DMiscTransformation to a linmath 4x4 transformation matrix
// ready to be used in the graphics API
void he_transformation_to_mat4x4(const A3DMiscTransformation* hnd_transformation, mat4x4 mat_result);

////////////////////////////////////////////////////////////////////////////////
/// Exchange <-> Graphics
////////////////////////////////////////////////////////////////////////////////
// Send the data in `mesh_data` to the GPU, reading to be drawn.
// GPU data is storedd in `data_traverse` and is used later on by `glfw_loop()`
std::pair<GLuint, GLsizei> he_mesh_data_to_opengl(A3DMeshData* const mesh_data, TraverseData* const data_traverse);

////////////////////////////////////////////////////////////////////////////////
/// Window/Graphics API Functions
////////////////////////////////////////////////////////////////////////////////
void glfw_loop(GLFWwindow* window, GLuint program, const SceneObject* object_start, size_t n_objects);
GLuint gl_prepare_program();
GLFWwindow* glfw_prepare();
void glfw_error_callback(int error, const char* description);
void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

////////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    ///////////////////////////////////////////
    // Initialize window and graphics resources
    GLFWwindow* window  = glfw_prepare();
    GLuint      program = gl_prepare_program();

    /////////////////////////////////////////////////////
    // INITIALIZE HOOPS EXCHANGE AND LOAD THE MODEL FILE.
    A3DSDKHOOPSExchangeLoader he_loader(HE_BINARY_DIRECTORY);
    assert(he_loader.m_eSDKStatus == A3D_SUCCESS);
    
    A3DImport he_import(HE_DATA_DIRECTORY INPUT_FILE);
    A3DStatus status = he_loader.Import(he_import);
    assert(status == A3D_SUCCESS);
    A3DAsmModelFile* model_file = he_loader.m_psModelFile;

    ////////////////////////////////////////////////////////
    // CALL THE TRAVERSE FUNCTIONS ON THE LOADED MODEL FILE.
    TraverseData     data_traverse;
    he_traverse_model_file(model_file, &data_traverse);

    /////////////////////////////////////////////////////////
    // Everything is loaded to GPU, Exchange can be released.
    A3DAsmModelFileDelete(model_file);
    A3DDllTerminate();
    A3DSDKUnloadLibrary();
 
    ///////////////////////////
    // Draw the scene on-screen
    printf("Starting Loop\n");
    glfw_loop(window, program, data_traverse.objects.data(), data_traverse.objects.size());
 
    /////////////////////////////////////////////
    // Clean up all window and graphics resources
    glDeleteProgram(program);
    glDeleteVertexArrays(data_traverse.gl_vaos.size(), data_traverse.gl_vaos.data());
    glDeleteBuffers(data_traverse.gl_vaos.size(), data_traverse.gl_vaos.data());
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////////////////////      FUNCTION DEFINITIONS      //////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Model File Traversal
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// ENTRY-POINT OF THE TREE TRAVERSAL.
/// Traverse the given model file instance and recursively visits the underlying
/// product occurrence entities.
/// The cascaded attributes and transform information are initialized and send
/// to the next traversal functions.
void he_traverse_model_file(A3DAsmModelFile* const hnd_modelfile, TraverseData* const data_traverse)
{
    A3DMiscCascadedAttributes* hnd_attrs_modelfile = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_modelfile);

    A3DAsmModelFileData data_modelfile;
    A3D_INITIALIZE_DATA(A3DAsmModelFileData, data_modelfile);
    A3DStatus code = A3DAsmModelFileGet(hnd_modelfile, &data_modelfile);
    assert(code == A3D_SUCCESS);

    mat4x4 mat_transform;
    mat4x4_identity(mat_transform);

    for (A3DUns32 po_i = 0 ; po_i < data_modelfile.m_uiPOccurrencesSize ; ++po_i) {
        he_traverse_product_occurrence(data_modelfile.m_ppPOccurrences[po_i], hnd_attrs_modelfile, mat_transform, data_traverse);
    }
    
    A3DAsmModelFileGet(0, &data_modelfile);
    A3DMiscCascadedAttributesDelete(hnd_attrs_modelfile);
}

////////////////////////////////////////////////////////////////////////////////
/// INTERMEDIARY STEP OF THE TREE TRAVERSAL.
/// Traverse a product occurrence and visits the underlying entities, which are
/// either other product occurrences or part definition entities.
/// Cascaded attributes and transform data are computed and accumulated for the
/// next traverse calls.
void he_traverse_product_occurrence(A3DAsmProductOccurrence* const hnd_po, A3DMiscCascadedAttributes* const hnd_attrs_parent, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{
    A3DMiscCascadedAttributes* hnd_attrs_po = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_po);
    A3DMiscCascadedAttributesPush(hnd_attrs_po, hnd_po, hnd_attrs_parent);
    A3DMiscCascadedAttributesEntityReferencePush(hnd_attrs_po, hnd_po, 0);

    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    A3DStatus code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    A3DMiscTransformation* hnd_po_transformation = he_get_product_occurrence_transformation(hnd_po);
    mat4x4 mat_transform_po_local, mat_transform_po_world;
    he_transformation_to_mat4x4(hnd_po_transformation, mat_transform_po_local);
    mat4x4_mul(mat_transform_po_world, mat_transform_world, mat_transform_po_local);

    A3DAsmPartDefinition* hnd_part = he_get_product_occurrence_part_definition(hnd_po);
    he_traverse_part_definition(hnd_part, hnd_attrs_po, mat_transform_po_world, data_traverse);

    for (A3DUns32 po_i = 0 ; po_i < data_po.m_uiPOccurrencesSize ; ++po_i) {
        he_traverse_product_occurrence(data_po.m_ppPOccurrences[po_i], hnd_attrs_po, mat_transform_po_world, data_traverse);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
}

////////////////////////////////////////////////////////////////////////////////
/// INTERMEDIARY STEP OF THE TREE TRAVERSAL.
/// Traverse a part definition and visits the underlying representation item
/// Cascaded attributes and transform data are computed and accumulated for the
/// next traverse calls.
void he_traverse_part_definition(A3DAsmPartDefinition* const hnd_part, A3DMiscCascadedAttributes* const hnd_attrs_po, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{
    if(hnd_part == 0) {
        return;
    }

    A3DMiscCascadedAttributes* hnd_attrs_part = 0;
    A3DMiscCascadedAttributesCreate(&hnd_attrs_part);
    A3DMiscCascadedAttributesPush(hnd_attrs_part, hnd_part, hnd_attrs_po);

    A3DAsmPartDefinitionData data_part;
    A3D_INITIALIZE_DATA(A3DAsmPartDefinitionData, data_part);
    A3DStatus code = A3DAsmPartDefinitionGet(hnd_part, &data_part);
    assert(code == A3D_SUCCESS);

    for (A3DUns32 ri_i = 0 ; ri_i < data_part.m_uiRepItemsSize ; ++ri_i) {
        he_traverse_representation_item(data_part.m_ppRepItems[ri_i], hnd_attrs_part, mat_transform_world, data_traverse);
    }

    A3DAsmPartDefinitionGet(0, &data_part);
}


////////////////////////////////////////////////////////////////////////////////
/// FINAL STEP OF THE TREE TRAVERSAL.
/// Traverse a representation item entity.
/// The main operation of this function is to call `A3DRiComputeMesh()` to
/// generate an `A3DMeshData` instance before sending the latter to the GPU
/// using `he_mesh_data_to_opengl()`.
/// The transform and cascaded attributes information accumulated along the tree
/// traversal are used in this step.
///
/// If the input representation item is an aggregate of several, the entity is
/// first unfolded and `he_traverse_representation_item()` is recursively called
/// for each of them.
void he_traverse_representation_item(A3DRiRepresentationItem* const hnd_ri, A3DMiscCascadedAttributes* const hnd_attrs_part, const mat4x4 mat_transform_world, TraverseData* const data_traverse)
{
    A3DEEntityType ri_type = kA3DTypeUnknown;
    A3DEntityGetType(hnd_ri, &ri_type);

    if (ri_type == kA3DTypeRiSet) {
        A3DRiSetData data_ri_set;
        A3D_INITIALIZE_DATA(A3DRiSetData, data_ri_set);
        A3DStatus code = A3DRiSetGet(hnd_ri, &data_ri_set);
        assert(code == A3D_SUCCESS);

        for(A3DUns32 ri_i = 0 ; ri_i < data_ri_set.m_uiRepItemsSize ; ++ri_i) {
            he_traverse_representation_item(data_ri_set.m_ppRepItems[ri_i], hnd_attrs_part, mat_transform_world, data_traverse);
        }

        A3DRiSetGet(0, &data_ri_set);
    } else {
        A3DMeshData mesh_data;
        A3D_INITIALIZE_DATA(A3DMeshData, mesh_data);
        A3DStatus code = A3DRiComputeMesh(hnd_ri, hnd_attrs_part, &mesh_data);
        if (code != A3D_SUCCESS)
        {
            A3DRWParamsTessellationData data_params_tess;
            A3D_INITIALIZE_DATA(A3DRWParamsTessellationData, data_params_tess);
            data_params_tess.m_eTessellationLevelOfDetail = kA3DTessLODMedium;
            A3DRiRepresentationItemComputeTessellation(hnd_ri, &data_params_tess);
            A3DStatus code = A3DRiComputeMesh(hnd_ri, hnd_attrs_part, &mesh_data);
        }
        assert(code == A3D_SUCCESS);

        auto gl_iterator = data_traverse->ri_to_gl.find(hnd_ri);
        if(gl_iterator == data_traverse->ri_to_gl.end()) {
            auto pair = he_mesh_data_to_opengl(&mesh_data, data_traverse);
            gl_iterator = data_traverse->ri_to_gl.insert({hnd_ri, pair}).first;
        }

        // Release mesh data memory
        A3DRiComputeMesh(0, 0, &mesh_data);

        SceneObject object;
        mat4x4_dup(object.mat_transform_model, mat_transform_world);
        object.gl_vao = gl_iterator->second.first;
        object.gl_indices_count = gl_iterator->second.second;
        data_traverse->objects.push_back(object);

    }
}


////////////////////////////////////////////////////////////////////////////////
/// The underlying part definition a product occurence entity may not be directly
/// accessed. Instead, it can be part of a prototype reference, or an external data.
/// This function provides a one-call function that returns the correct part
/// definition of a product occurrence, if available.
/// The function is used in `he_traverse_product_occurrence()`
A3DAsmPartDefinition* he_get_product_occurrence_part_definition(A3DAsmProductOccurrence* hnd_po)
{
    if(hnd_po == 0) {
        return 0;
    }

    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    A3DStatus code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    A3DMiscTransformation* hnd_part = data_po.m_pPart;
    if (hnd_part == 0) {
        A3DAsmProductOccurrence* hnd_po_reference = data_po.m_pPrototype ? data_po.m_pPrototype : data_po.m_pExternalData;
        hnd_part = he_get_product_occurrence_part_definition(hnd_po_reference);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
    return hnd_part;
}

////////////////////////////////////////////////////////////////////////////////
/// The underlying transformation a product occurence entity may not be directly
/// accessed. Instead, it can be part of a prototype reference, or an external data.
/// This function provides a one-call function that returns the correct
/// transformation of a product occurrence, if available.
/// The function is used in `he_traverse_product_occurrence()`
A3DMiscTransformation* he_get_product_occurrence_transformation(A3DAsmProductOccurrence* hnd_po)
{
    if(hnd_po == 0) {
        return 0;
    }

    A3DAsmProductOccurrenceData data_po;
    A3D_INITIALIZE_DATA(A3DAsmProductOccurrenceData, data_po);
    A3DStatus code = A3DAsmProductOccurrenceGet(hnd_po, &data_po);
    assert(code == A3D_SUCCESS);

    A3DMiscTransformation* hnd_po_transformation = data_po.m_pLocation;
    if (hnd_po_transformation == 0) {
        A3DAsmProductOccurrence* hnd_po_reference = data_po.m_pPrototype ? data_po.m_pPrototype : data_po.m_pExternalData;
        hnd_po_transformation = he_get_product_occurrence_transformation(hnd_po_reference);
    }

    A3DAsmProductOccurrenceGet(0, &data_po);
    return hnd_po_transformation;
}

////////////////////////////////////////////////////////////////////////////////
/// This utility function computes a column-major 4x4 transformation matrix out
/// of an `A3DMiscTransformation` entity. 
/// The result is ready to be send to our graphics API.
void he_transformation_to_mat4x4(const A3DMiscTransformation* hnd_transformation, mat4x4 mat_result)
{
    if (hnd_transformation == 0) {
        mat4x4_identity(mat_result);
    } else {
        A3DEEntityType entity_type = kA3DTypeUnknown;
        A3DEntityGetType(hnd_transformation, &entity_type);
        assert(entity_type == kA3DTypeMiscCartesianTransformation);

        A3DMiscCartesianTransformationData data;
        A3D_INITIALIZE_DATA(A3DMiscCartesianTransformationData, data);
        A3DStatus code = A3DMiscCartesianTransformationGet(hnd_transformation, &data);
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

////////////////////////////////////////////////////////////////////////////////
/// Exchange <-> Graphics
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Pivot function that sends a mesh represented by `mesh_data` into the GPU.
/// The Graphics API uses OpenGL buffer.
/// This function first prepares the data for the buffer memory and stores the
/// buffer identifier into `data_traverse`.
/// The identifiers are used later on for drawing by `glfw_loop()`.
/// TODO: Split this function to extract A3DMeshData content into std containers which are sent to he_mesh_data_to_opengl.
/// + Maybe rename this function.
std::pair<GLuint, GLsizei> he_mesh_data_to_opengl(A3DMeshData* const mesh_data, TraverseData* const data_traverse)
{
    // TODO About the 40 first lines (until gl.. calls) of this function are
    // removed thanks to Usability Phase 1 feedback.

    GLuint gl_shader_coord_location =  0;
    GLuint gl_shader_normal_location = 1;
    std::vector<GLuint> index_buffer;
    std::vector<GLdouble> vertex_buffer;

    // Count the total number of indices
    // The buffer objects will have at max n_indices indices
    size_t n_indices = 0;
    for(A3DUns32 face_i = 0 ; face_i < mesh_data->m_uiFaceSize ; ++face_i) {
        n_indices += 3 * mesh_data->m_puiTriangleCountPerFace[face_i];
    }

    // This map will serve as cache for the OpenGL indices
    // vertex_index|normal_index -> gl_index
    std::unordered_map<uint64_t, GLuint> index_cache;

    for (A3DUns32 face_i = 0; face_i < mesh_data->m_uiFaceSize; ++face_i) {
        A3DUns32 n_triangles = mesh_data->m_puiTriangleCountPerFace[face_i];
        if (n_triangles == 0) {
            continue;
        }
        const A3DUns32* ptr_coord_index = mesh_data->m_ppuiPointIndicesPerFace[face_i];
        const A3DUns32* ptr_normal_index = mesh_data->m_ppuiNormalIndicesPerFace[face_i];

        for (size_t vertex_i = 0 ; vertex_i < n_triangles * 3 ; ++vertex_i) {
            A3DUns32 coord_index = *ptr_coord_index++;
            A3DUns32 normal_index = *ptr_normal_index++;

            uint64_t cache_key = ((uint64_t)coord_index << 32) | normal_index;
            auto insertion = index_cache.insert({cache_key, vertex_buffer.size() / 6});

            GLuint vertex_index = insertion.first->second;
            if(insertion.second) {
                auto x = mesh_data->m_pdCoords[coord_index];
                auto y = mesh_data->m_pdCoords[coord_index+1];
                auto z = mesh_data->m_pdCoords[coord_index+2];

                // Create the coordinates
                vertex_buffer.insert(vertex_buffer.end(), {
                    mesh_data->m_pdCoords[coord_index],
                    mesh_data->m_pdCoords[coord_index + 1],
                    mesh_data->m_pdCoords[coord_index + 2],
                    mesh_data->m_pdNormals[normal_index],
                    mesh_data->m_pdNormals[normal_index + 1],
                    mesh_data->m_pdNormals[normal_index + 2]
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

    glEnableVertexAttribArray(gl_shader_coord_location);
    glVertexAttribPointer(gl_shader_coord_location, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(GLdouble), (void*) 0);
    glEnableVertexAttribArray(gl_shader_normal_location);
    glVertexAttribPointer(gl_shader_normal_location, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(GLdouble), (void*) (3 * sizeof(GLdouble)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    data_traverse->gl_vaos.push_back(gl_vao);
    data_traverse->gl_vbos.push_back(gl_bo_vertex);
    data_traverse->gl_vbos.push_back(gl_bo_index);

    return {gl_vao, (GLsizei)index_buffer.size()};
}

////////////////////////////////////////////////////////////////////////////////
/// Window/Graphics API Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Drawing loop.
/// The functions indefinitely calls the graphics API drawing calls to display
/// all the objects generated by `he_mesh_data_to_opengl()`.
/// A simple rotation computation is called to animate the display.
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

////////////////////////////////////////////////////////////////////////////////
/// Initialized the window using GLFW
GLFWwindow* glfw_prepare()
{
    glfwSetErrorCallback(glfw_error_callback);
 
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
 
    glfwSetKeyCallback(window, glfw_key_callback);
 
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSwapInterval(1);

    return window;
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

////////////////////////////////////////////////////////////////////////////////
/// Initializes OpenGL Shader program
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
 


