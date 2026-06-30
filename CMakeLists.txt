cmake_minimum_required(VERSION 3.0.2)
project(acc_aeb_controller)

## --- Build configuration ------------------------------------------------------
# C++17 is required ([[nodiscard]], inline constexpr, structured init).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to an optimized build for the Jetson AGX Orin target. Release adds
# -O3 -DNDEBUG which is what the real-time / reciprocal-math work assumes.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

## --- Dependencies -------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  pro_can
  npust_bus_msgs
)
find_package(Threads REQUIRED)   # std::thread watchdog

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs pro_can npust_bus_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## --- Targets ------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/acc_aeb_node.cpp
  src/params.cpp
  src/kinematics.cpp
  src/state_machine.cpp
)

# Ensure generated message headers (pro_can, npust_bus_msgs) exist before build.
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

## --- Tests -------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_unit_test
    test/unit_test.cpp
    src/kinematics.cpp
    src/state_machine.cpp
  )
  target_link_libraries(${PROJECT_NAME}_unit_test
    ${catkin_LIBRARIES}
    gtest_main
  )
endif()

## --- Install ------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
