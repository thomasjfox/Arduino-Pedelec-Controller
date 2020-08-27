set(cores_SOURCES
    sim_mega2560.c
    sim_mega328.c
    sim_megax8.c
    )

set(cores_HEADERS
    sim_core_config.h
    sim_core_declare.h
    sim_core_decl.h
    sim_mega2560.h
    sim_megax8.h
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/avr
                    ${CMAKE_CURRENT_SOURCE_DIR}/../sim)

# from simavr Makefiles: enable AVR_CORE define during core compilation
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAVR_CORE=1")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAVR_CORE=1")

add_library(simavr_cores SHARED ${cores_SOURCES} ${cores_HEADERS})
