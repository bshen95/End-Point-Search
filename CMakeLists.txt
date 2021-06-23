project(EPS)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations")
set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${DEV_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${RELEASE_CXX_FLAGS}")


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../bin)

#linux
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include -fopenmp")
#

#mac
#Sometime my machine is way stupid !  have no idea why it cant find omp
find_library(omp_lib omp "/usr/local/opt")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xpreprocessor -fopenmp")
include_directories("/usr/local/Cellar/libomp/10.0.0/include")



include_directories(generator)
include_directories(helpers)
include_directories(index)
include_directories(rstar)
include_directories(search)
include_directories(structs)
include_directories(testcases)
include_directories(ordering)

set(SRC
        helpers/cfg.cpp
        helpers/cfg.h
        helpers/cpool.h
        helpers/geometry.cpp
        helpers/geometry.h
        helpers/rtree.h
        helpers/scenario.cpp
        helpers/scenario.h
        helpers/timer.cpp
        helpers/timer.h
        helpers/timestamp_flag.h
        helpers/constants.h
        helpers/id_queue.h


        search/expansion.cpp
        search/expansion.h
        search/searchinstance.cpp
        search/searchinstance.h
        search/visibleSearchInstance.cpp
        search/visibleSearchInstance.h
        search/epsSearchInstance.h
        search/epsSearchInstance.cpp
        search/dijkstra.h
        search/eps.h


        structs/vec_io.h
        structs/consts.h
        structs/mesh.cpp
        structs/mesh.h
        structs/point.h
        structs/polygon.h
        structs/searchnode.h
        structs/successor.h
        structs/vertex.h
        structs/graph.h
        structs/graph.cpp

        structs/cpd.cpp
        structs/cpd.h
        structs/binary_search.h
        structs/range.h

        )
add_executable(build_visibility_graph ${SRC} build_visibility_graph.cpp)
add_executable(construct_cpd ${SRC} construct_cpd.cpp)
add_executable(testEPS ${SRC} testEPS.cpp)


if (omp_lib)
    target_link_libraries(build_visibility_graph ${omp_lib})
    target_link_libraries(construct_cpd ${omp_lib})
endif()
