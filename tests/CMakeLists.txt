# extra cmake project for fcsim
# to build using host compiler
cmake_minimum_required(VERSION 3.7.0)
project(fcsim C CXX)

add_subdirectory(simavr)
add_subdirectory(fcsim)
