cmake_minimum_required(VERSION 3.15)

project(codeInCpp)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})



set(CMAKE_BUILD_TYPE Debug)

link_directories($(Eigen_INCLUDE_DIRS))

file(GLOB headers include/*.hpp)
file(GLOB sources src/*.cpp)

include_directories(include)
include_directories(/usr/include/python3.8)

# build robot sim with ekf
add_executable(plotRobot_ekf src/plotRobot_ekf.cpp src/world.cpp src/robot.cpp include/world.hpp include/robot.hpp include/parameters.hpp)
#target_include_directories(executableName ./include)

target_link_libraries(plotRobot_ekf
    ${Eigen_LIBRARIES}
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
    )

# #build robot sim with particle filter
add_executable(plotRobot_particle_filter src/plotRobot_particle_filter.cpp  include/parameters.hpp
                   src/world.cpp     src/robot.cpp     src/particle_filter.cpp     src/utils.cpp
               include/world.hpp include/robot.hpp include/particle_filter.hpp include/utils.hpp)
#target_include_directories(executableName ./include)

target_link_libraries(plotRobot_particle_filter
    ${Eigen_LIBRARIES}
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
    )


    