cmake_minimum_required(VERSION 3.13)
project(Feature)

set(CMAKE_CXX_STANDARD 14)



#Opencv
find_package(OpenCV 3.3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

#DBoW
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/FeatureExtraction.cpp
        src/FeatureMatching.cpp
        src/Image.cpp
        src/tracks.cpp
        src/init.cpp
        src/Camera.cpp
        src/common.cpp
        )


add_executable(FeatureGraph main.cpp)
target_link_libraries(FeatureGraph ${PROJECT_NAME} ${OpenCV_LIBS} ${DBoW3_LIBS})
