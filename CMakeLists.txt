cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic (gz-sim 8) via ROS vendor packages.
# The *_vendor packages prepend their install prefix to CMAKE_PREFIX_PATH so
# that the versioned gz-* config packages below can be found.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common5 REQUIRED)

find_package(Boost REQUIRED)

add_library(PedestrianSFMPlugin SHARED src/PedestrianSFMPlugin.cpp)
target_include_directories(PedestrianSFMPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# lightsfm is a standalone header-only library installed under /usr/local.
target_include_directories(PedestrianSFMPlugin SYSTEM PRIVATE
  /usr/local/include
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(PedestrianSFMPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-common5::gz-common5
)

install(TARGETS PedestrianSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY media/models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
