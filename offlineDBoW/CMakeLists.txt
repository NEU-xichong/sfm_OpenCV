cmake_minimum_required(VERSION 2.8 )
project(DBoW)

#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#Opencv
find_package(OpenCV 3.3.0 REQUIRED)
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
link_directories   (${Boost_LIBRARY_DIR})

#DBoW
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )

add_executable(offlineDBoW offlineDBoW.cpp)
target_link_libraries(offlineDBoW ${OpenCV_LIBS} ${DBoW3_LIBS} ${Boost_LIBRARIES})

