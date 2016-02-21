#!/bin/sh
# build project files and run test...
mkdir build
cd build 
cmake ..
make
cd ..
./runtests

