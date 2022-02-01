cmake_minimum_required(VERSION 3.10)

project(caliba)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3rdpart)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(${PROJECT_NAME}_preprocessor src/preprocessor.cpp)
target_include_directories(${PROJECT_NAME}_preprocessor PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_preprocessor PRIVATE
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_calibrator src/calibrator.cpp)
target_include_directories(${PROJECT_NAME}_calibrator PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_calibrator PRIVATE
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ceres
)

add_library(${PROJECT_NAME}_utils src/utils.cpp)
add_dependencies(${PROJECT_NAME}_utils yaml-cpp)
target_compile_definitions(${PROJECT_NAME}_utils PUBLIC
  DIR_HOME="${CMAKE_CURRENT_LIST_DIR}"
)
target_include_directories(${PROJECT_NAME}_utils PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC
  yaml-cpp
)

add_executable(${PROJECT_NAME}_calib src/calib.cpp)
add_dependencies(${PROJECT_NAME}_calib
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_utils
)
target_link_libraries(${PROJECT_NAME}_calib PUBLIC
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_utils
)

add_executable(${PROJECT_NAME}_cmb src/combine.cpp)
add_dependencies(${PROJECT_NAME}_cmb
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_utils
)
target_link_libraries(${PROJECT_NAME}_cmb PUBLIC
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_utils
)

add_executable(${PROJECT_NAME}_apb src/apply_bias.cpp)
add_dependencies(${PROJECT_NAME}_apb
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_utils
)
target_link_libraries(${PROJECT_NAME}_apb PUBLIC
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_utils
)

add_executable(${PROJECT_NAME}_split src/split.cpp)
target_include_directories(${PROJECT_NAME}_split PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_split PUBLIC
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_preprocessor
)