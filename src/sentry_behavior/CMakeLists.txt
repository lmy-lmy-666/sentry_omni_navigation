cmake_minimum_required(VERSION 3.16)
project(sentry_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rm_interfaces REQUIRED)

###########
## Build ##
###########

ament_auto_add_executable(sentry_behavior_node
  src/sentry_behavior_node.cpp
  src/goal_publisher.cpp
  src/behavior_machine.cpp
  src/strategies.cpp
  src/state_viz_server.cpp
)
ament_target_dependencies(sentry_behavior_node rm_interfaces)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  params
  launch
)
