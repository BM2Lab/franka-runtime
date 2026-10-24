cmake_minimum_required(VERSION 3.16)
project(droid_readonly_state LANGUAGES CXX)
find_package(Franka 0.21.2 EXACT CONFIG REQUIRED)
find_package(Threads REQUIRED)
add_executable(droid_readonly_state droid_readonly_state.cpp)
target_compile_features(droid_readonly_state PRIVATE cxx_std_17)
target_link_libraries(droid_readonly_state PRIVATE Franka::Franka Threads::Threads)
# Frozen libfranka has transitive model-library dependencies. Pin explicit lookup
# paths in this helper, never rely on a sourced ROS/LD_LIBRARY_PATH environment.
set(DROID_DEPENDENCY_RPATH "" CACHE STRING "Explicit transitive SDK library directories")
if(DROID_DEPENDENCY_RPATH)
  set_property(TARGET droid_readonly_state APPEND PROPERTY BUILD_RPATH "${DROID_DEPENDENCY_RPATH}")
  target_link_options(droid_readonly_state PRIVATE "-Wl,--disable-new-dtags")
endif()
