cmake_minimum_required(VERSION 2.6)
project(FPFH)

find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED system filesystem)
find_package(Qt4 REQUIRED QtGui QtXml QtOpenGL)
find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# executable is inside the source directory...
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}) 
set(CMAKE_BUILD_TYPE Debug)

include_directories(${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} lib/)

add_subdirectory(lib/gtest)
  
# build your code
add_executable(create-histograms
  project/Features.cpp
  project/Histogram.cpp
  project/FPFH.cpp
  project/Importer.cpp
  project/Exporter.cpp
  project/KdTree.cpp
  project/Point.cpp
  project/PointCloud.cpp
  project/SPFH.cpp
  create-histograms.cpp)
  
add_executable(classify-histograms
  project/Point.cpp
  project/PointCloud.cpp
  project/Importer.cpp
  project/Exporter.cpp
  classify-histograms.cpp)
  
add_executable(post-processing
  project/Importer.cpp
  project/Exporter.cpp
  project/KdTree.cpp
  project/Point.cpp
  project/PointCloud.cpp
  project/RegionMerging.cpp
  post-processing.cpp)
  
add_executable(evaluate-files
  evaluate-files.cpp)

add_executable(runtests
  # tested classes
  project/KdTree.cpp
  project/Point.cpp
  project/PointCloud.cpp
  project/Features.cpp
  project/SPFH.cpp
  project/FPFH.cpp
  project/Histogram.cpp
  project/RegionMerging.cpp
  # test cases
  test/kdtree-test.cpp
  test/normal-test.cpp
  test/histogram-test.cpp
  test/svm-light-test.cpp
  test/region-merging-test.cpp)
  
target_link_libraries(create-histograms ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(post-processing ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(evaluate-files ${Boost_LIBRARIES})
target_link_libraries(runtests ${Boost_LIBRARIES} gtest_main)