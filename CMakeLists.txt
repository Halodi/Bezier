cmake_minimum_required(VERSION 3.5)
project(bezier)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


add_compile_options(-fPIC)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
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
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
) 

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
