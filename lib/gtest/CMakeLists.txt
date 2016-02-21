# build gtest library for linking with sources

find_package(Threads REQUIRED)

add_library(gtest
  gtest-all.cc)
  
add_library(gtest_main
  gtest-all.cc
  gtest_main.cc)
  
target_link_libraries(gtest pthread)
target_link_libraries(gtest_main pthread)