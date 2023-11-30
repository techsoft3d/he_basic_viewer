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
#include <cassert>

#include "rendering.hpp"

#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>

////////////////////////////////////////////////////////////////////////////////
/// INPUT_FILE
/// Default CAD input file, relative to Exchange sample data folder.
/// To see how the value is used, check the `main()` function.
#define INPUT_FILE "/prc/_micro engine.prc"

////////////////////////////////////////////////////////////////////////////////
// Recursive tree traversal function used to retrieve drawable entities
void he_traverse_tree(A3DTree* const hnd_tree, A3DTreeNode* const hnd_node, TraverseData* const data_traverse);

////////////////////////////////////////////////////////////////////////////////
// Utility function to convert A3DMiscTransformation object to 4x4 transform matrix
void he_transformation_to_mat4x4(const A3DMiscTransformation* hnd_transformation, mat4x4 mat_result);

////////////////////////////////////////////////////////////////////////////////
// Send the data in `mesh_data` to the rendering API, ready to be drawn.
std::pair<GLuint, GLsizei> he_mesh_data_to_rendering(A3DMeshData* const mesh_data);

////////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    ///////////////////////////////////////////
    // Initialize window and graphics resources
    GLFWwindow* window  = rendering_prepare_window();
    GLuint      program = rendering_prepare();

    /////////////////////////////////////////////////////
    // INITIALIZE HOOPS EXCHANGE AND LOAD THE MODEL FILE.
    A3DSDKHOOPSExchangeLoader he_loader(HE_BINARY_DIRECTORY);
    assert(he_loader.m_eSDKStatus == A3D_SUCCESS);
    
    A3DImport he_import(HE_DATA_DIRECTORY INPUT_FILE);
    A3DStatus status = he_loader.Import(he_import);
    assert(status == A3D_SUCCESS);
    A3DAsmModelFile* model_file = he_loader.m_psModelFile;

    ////////////////////////////////////////////////////////
    // TRAVERSE THE MODEL TREE
    TraverseData     data_traverse;
    A3DTree*         hnd_tree = 0;

    status = A3DTreeCompute(model_file, &hnd_tree, 0);
    assert(status == A3D_SUCCESS);

    A3DTreeNode* hnd_root_node = 0;
    status = A3DTreeGetRootNode(hnd_tree, &hnd_root_node);
    assert(status == A3D_SUCCESS);

    he_traverse_tree(hnd_tree, hnd_root_node, &data_traverse);

    A3DTreeCompute(0, &hnd_tree, 0);

    /////////////////////////////////////////////////////////
    // Everything is loaded to GPU, Exchange can be released.
    A3DAsmModelFileDelete(model_file);
    A3DDllTerminate();
    A3DSDKUnloadLibrary();
 
    ///////////////////////////
    // Draw the scene on-screen
    rendering_loop(window, program, data_traverse.objects.data(), data_traverse.objects.size());
 
    /////////////////////////////////////////////
    // Clean up all window and graphics resources
    rendering_cleanup(program);
    rendering_cleanup_window(window);

    return EXIT_SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive traversal function, initially called in main() on the root node.
// The function:
// - Extracts any geometry as A3DMeshData and send it to the GPU using he_mesh_data_to_rendering()
// - Recursively calls itself on the child nodes
void he_traverse_tree(A3DTree* const hnd_tree, A3DTreeNode* const hnd_node, TraverseData* const data_traverse)
{
    // Extract the geometry as an A3DMeshData instance and send it to the GPU
    // if the operation is successful.
    A3DMeshData mesh_data;
    A3D_INITIALIZE_DATA(A3DMeshData, mesh_data);
    A3DStatus code = A3DTreeNodeGetGeometry(hnd_tree, hnd_node, A3D_TRUE, &mesh_data, 0);

    if(code == A3D_SUCCESS) {

        // Search for existing geometry, insert new one if new
        A3DEntity* hnd_ri = 0;
        A3DTreeNodeGetEntity(hnd_node, &hnd_ri);
        auto gl_iterator = data_traverse->ri_to_gl.find(hnd_ri);
        if(gl_iterator == data_traverse->ri_to_gl.end()) {
            auto pair = he_mesh_data_to_rendering(&mesh_data);
            gl_iterator = data_traverse->ri_to_gl.insert({hnd_ri, pair}).first;
        }

        // Release the mesh data memory
        A3DTreeNodeGetGeometry(0, 0, A3D_TRUE, &mesh_data, 0);

        // Get the net transform of the node
        A3DMiscTransformation* hnd_net_transform = 0;
        A3DTreeNodeGetNetTransformation(hnd_node, &hnd_net_transform);

        // Store the drawable object with the following information:
        // - Net transform as a 4x4 matrix (.mat_transform_model)
        // - Graphics API-side identifier (.gl_vao)
        // - Number of vertex index used upon drawing (.gl_indices_count)
        SceneObject object;
        he_transformation_to_mat4x4(hnd_net_transform, object.mat_transform_model);
        object.gl_vao = gl_iterator->second.first;
        object.gl_indices_count = gl_iterator->second.second;
        data_traverse->objects.push_back(object);
    }

    // Recursively traverse the child nodes
    A3DUns32 n_children        = 0;
    A3DTreeNode** hnd_children = 0;

    code = A3DTreeNodeGetChildren(hnd_tree, hnd_node, &n_children, &hnd_children);
    assert(code == A3D_SUCCESS);
    for (size_t c = 0 ; c < n_children ; ++c) {
        he_traverse_tree(hnd_tree, hnd_children[c], data_traverse);
    }
    A3DTreeNodeGetChildren(0, 0, &n_children, &hnd_children);
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
/// Pivot function that sends a mesh represented by `mesh_data` into the GPU.
/// The Graphics API uses OpenGL buffer.
/// This function first prepares the data for the buffer memory and stores the
/// buffer identifier into `data_traverse`.
/// The identifiers are used later on for drawing by `rendering_loop()`.
std::pair<GLuint, GLsizei> he_mesh_data_to_rendering(A3DMeshData* const mesh_data)
{
    std::vector<GLdouble> vertex_buffer(mesh_data->m_pdCoords, mesh_data->m_pdCoords + mesh_data->m_uiCoordSize);
    std::vector<GLdouble> normal_buffer(mesh_data->m_pdNormals, mesh_data->m_pdNormals + mesh_data->m_uiNormalSize);

    // Count the total number of indices, which is equal to 3 times the sum of triangle count per face
    // The buffer objects will have at max n_indices indices
    size_t n_indices = 0;
    for(A3DUns32 face_i = 0 ; face_i < mesh_data->m_uiFaceSize ; ++face_i) {
        n_indices += 3 * mesh_data->m_puiTriangleCountPerFace[face_i];
    }
    std::vector<GLuint> index_buffer(mesh_data->m_puiVertexIndicesPerFace, mesh_data->m_puiVertexIndicesPerFace + n_indices);

    GLuint renderable_id = rendering_to_gpu(index_buffer, vertex_buffer, normal_buffer);
    return {renderable_id, (GLsizei)index_buffer.size()};
}


