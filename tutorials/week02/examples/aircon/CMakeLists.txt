# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (aircon)

# This is added to c++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
# Notice we now have two source files, one has our main, the other implements our circle class
add_executable(aircon_ex main.cpp aircon.cpp)
