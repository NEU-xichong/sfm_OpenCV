cmake_minimum_required(VERSION 3.5.1)

project(Feature)

set(CMAKE_CXX_STANDARD 14)

#aux_source_directory(.. DIR_SRCS)
#add_subdirectory(../exif_third_party)
#add_subdirectory(../exif)

set(SRC_BASE ${PROJECT_SOURCE_DIR})
include_directories(
        ${SRC_BASE}/../include
        ${SRC_BASE}/../exif_third_party
)

#Opencv
find_package(OpenCV 3.3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

#DBoW
#set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
#set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )


# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME} SHARED
        FeatureExtraction.cpp
        FeatureMatching.cpp
        Image.cpp
        tracks.cpp
        init.cpp
        Camera.cpp
        common.cpp
        PnP3D2D.cpp
        BundelOptimizer.cpp
        map.cpp
        sfm.cpp
        easy_IO_exif.cpp
        )


#add_executable(FeatureGraph
#        ../main.cpp
              #${DIR_SRCS}
#)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${DBoW3_LIBS})
