cmake_minimum_required(VERSION 3.5)
project(bezier)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-fPIC)

option(BUILD_PYTHON_MODULE "Build python module with pybind11" OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log
        log_setup
        )

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
  include
  test
  ${EIGEN3_INCLUDE_DIRS}
)


set(dependencies
  ament_cmake
)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
  src/bezier.cpp
  src/polycurve.cpp
  src/SplineVelocity.cpp
  src/BezierTrajectory2D.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

if(BUILD_PYTHON_MODULE)
  # Python module
  find_package(pybind11 REQUIRED)
  
  pybind11_add_module(python_bezier src/python.cpp)
  target_compile_features(python_bezier PUBLIC cxx_std_17)
  target_link_libraries(python_bezier PUBLIC ${PROJECT_NAME})
  
  set_target_properties(python_bezier PROPERTIES OUTPUT_NAME bezier)
  set_target_properties(python_bezier PROPERTIES ARCHIVE_OUTPUT_NAME python_bezier)
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/Bezier
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
