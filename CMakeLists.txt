cmake_minimum_required(VERSION 3.18)
project(MeshViewer)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
add_subdirectory(glfw-3.3.8)

set(CMAKE_CXX_STANDARD 11)

set(EXCHANGE_PACKAGE_PATH "" CACHE PATH "Path to Exchange")
if(NOT EXCHANGE_PACKAGE_PATH)
    message(FATAL_ERROR "EXCHANGE_PACKAGE_PATH must be set to a valid folder path.")
endif()
set(HE_DATA_DIRECTORY "${EXCHANGE_PACKAGE_PATH}/samples/data")
if(WIN32)
elseif(APPLE)
else()
    set(HE_BINARY_DIRECTORY ${EXCHANGE_PACKAGE_PATH}/bin/linux32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(HE_BINARY_DIRECTORY ${EXCHANGE_PACKAGE_PATH}/bin/linux64)
    endif()
endif()

add_executable(MeshViewer main.cpp glad.c)
target_include_directories(MeshViewer PRIVATE 
    "${EXCHANGE_PACKAGE_PATH}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_definitions(MeshViewer PRIVATE 
    HE_BINARY_DIRECTORY="${HE_BINARY_DIRECTORY}"
    HE_DATA_DIRECTORY="${HE_DATA_DIRECTORY}"
)
target_link_libraries(MeshViewer PRIVATE glfw)
