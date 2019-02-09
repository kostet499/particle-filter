cmake_minimum_required(VERSION 3.12)
project(Particle_Filter)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIRS})

add_executable(Particle_Filter main.cpp ParticleFilter.cpp ParticleFilter.h)
target_link_libraries(Particle_Filter ${Boost_LIBRARIES})
target_link_libraries(Particle_Filter jsoncpp)
