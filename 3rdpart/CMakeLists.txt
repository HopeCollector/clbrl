set(BUILD_TYPE_CASH ${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE Release)

add_subdirectory(yaml-cpp)

set(CMAKE_BUILD_TYPE ${BUILD_TYPE_CASH})