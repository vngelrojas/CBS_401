cmake_minimum_required(VERSION 3.21)
project(ITACBS_remake)

set(CMAKE_OSX_ARCHITECTURES "arm64")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(../parlaylib/include)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
find_package(yaml-cpp REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
file(GLOB THIRD_PARTY "third_party/*.cpp" "third_party/*.h")




add_executable(CBS include/CBS/main_CBS.cpp include/CBS/CBS.cpp include/CBS/CBSNode.cpp include/common.cpp)
target_link_libraries(CBS
        ${Boost_LIBRARIES}
        yaml-cpp
        )

add_executable(CBS_parallel include/CBS_parallel/main_CBS_parallel.cpp include/CBS_parallel/CBS_parallel.cpp include/CBS_parallel/CBSNode_parallel.cpp include/common.cpp)
target_link_libraries(CBS_parallel
        ${Boost_LIBRARIES}
        yaml-cpp
        )

add_executable(ECBS include/ECBS/main_ECBS.cpp include/ECBS/ECBS.cpp include/ECBS/ECBSNode.cpp include/common.cpp)
target_link_libraries(ECBS
        ${Boost_LIBRARIES}
        yaml-cpp
        )




