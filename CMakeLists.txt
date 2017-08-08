project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall -DHAVE_CSTDDEF -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(
    "src/Eigen-3.3"
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning
    "src/behavior_planner.cpp"
    "src/highway_map.cpp"
    "src/lane.cpp"
    "src/main.cpp"
    "src/planner.cpp"
    "src/obstacles.cpp"
    "src/path_planner.cpp"
    "src/car.cpp"
    "src/waypoints.cpp"
)

target_link_libraries(path_planning ipopt z ssl uv uWS)
