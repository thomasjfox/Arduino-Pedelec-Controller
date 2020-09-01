include_directories(${CMAKE_SOURCE_DIR}/simavr/cores)
include_directories(${CMAKE_SOURCE_DIR}/simavr/sim)

add_executable(fcsim fcsim.cpp)
target_link_libraries(fcsim simavr)
