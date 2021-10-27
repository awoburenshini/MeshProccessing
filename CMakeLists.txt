cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(MeshProcessing)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ext/eigen")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ext/RapidJSON/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ext/libTinyTIFF/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB INCS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

# link_directories(MeshProcessing "${CMAKE_CURRENT_SOURCE_DIR}/ext/libTinyTIFF/lib")

find_package(TinyTIFF
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ext/libTinyTIFF/lib/cmake"
    REQUIRED)

add_executable(
    MeshProcessing
 
${SOURCES} 
${INCS} 
${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_link_libraries(MeshProcessing TinyTIFF)
