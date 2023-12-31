cmake_minimum_required(VERSION 3.20)

project(MeshExtraction LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17) 

# set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../libeigen/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
add_subdirectory(${CMAKE_SOURCE_DIR}/deps/polyscope)
add_subdirectory(${CMAKE_SOURCE_DIR}/deps/geometry-central)

file(GLOB_RECURSE CXX_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# file(GLOB_RECURSE CXX_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

function(ManageList fileList)
    foreach(file IN LISTS fileList)
        get_filename_component(filePath ${file} PATH)
        string(REPLACE "/" "\\" finalPath ${filePath})
        source_group(${finalPath} FILES ${file})
    endforeach()
endfunction()

ManageList("${CXX_SRCS}")
ManageList("${HEADERS}")

add_executable(MeshExtraction ${CXX_SRCS} ${HEADERS})
target_link_libraries(MeshExtraction polyscope)
target_link_libraries(MeshExtraction geometry-central)

set_directory_properties(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT MeshExtraction)