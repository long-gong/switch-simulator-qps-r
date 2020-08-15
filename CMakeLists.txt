cmake_minimum_required(VERSION 3.0)

project(SSchedSimulator)

set(CMAKE_CXX_STANDARD 11)



option(WITH_TESTS "Build with tests enabled" ON)
option(BUILD_SEPARATE_MODELS "Build each module as a separate library" OFF)
option(BUILD_SHARED_LIBS "Build as shared libraries" OFF)
option(ENABLE_HDRHISTOGRAM "Build with HdrHistogram library enabled" ON)


# libraries
set(saber_libraries "")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/lib)

list(APPEND saber_libraries emon)

if (ENABLE_HDRHISTOGRAM)
    list(APPEND saber_libraries hdr_histogram_static)
    add_definitions(-DENABLE_HDRHISTOGRAM)
endif ()

# includes
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

# added
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/utils")

if (CMAKE_BUILD_TYPE EQUAL "Debug")
    message("debug mode")
    set(CMAKE_CXX_FLAGS "-g -O0")
else ()
    set(CMAKE_CXX_FLAGS "-O3 -Wall")
endif ()


add_subdirectory(src)

if (WITH_TESTS)
# Enable test
enable_testing()
add_subdirectory(test)
add_subdirectory(example)
endif()


# add executable
add_executable(ssched_simulator
        ssched_simulator.cc)
#target_compile_definitions(ssched_simulator PUBLIC -DDEBUG)  # -D removed
target_link_libraries(ssched_simulator
        scheduler
        switch
        simulator
        ${sbaer_libraries} pthread boost_graph)

