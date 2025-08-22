cmake_minimum_required(VERSION 3.10)
project(rclgo_clock_demo_pkg)

find_package(ament_cmake REQUIRED)

# Ensure Go is available
find_program(GO_EXECUTABLE go)
if(NOT GO_EXECUTABLE)
  message(FATAL_ERROR "Go toolchain (go) not found in PATH")
endif()

# Build directory where the Go binary will be placed
set(OUT_BIN "${CMAKE_CURRENT_BINARY_DIR}/clock_demo")

# Environment for Go build:
# - GO111MODULE=on to use modules
# - CGO_ENABLED=1 because rclgo uses cgo
# - Inherit the sourced ROS 2 environment so DDS libs are found at runtime
add_custom_command(
  OUTPUT ${OUT_BIN}
  COMMAND ${CMAKE_COMMAND} -E env
          GO111MODULE=on
          CGO_ENABLED=1
          ${GO_EXECUTABLE} build -o ${OUT_BIN} .
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building Go binary param_demo (module-resolved rclgo)"
  VERBATIM
)

add_custom_target(param_demo_bin ALL DEPENDS ${OUT_BIN})

# Install so ros2 run can find it: lib/<package>/param_demo
install(PROGRAMS ${OUT_BIN} DESTINATION lib/${PROJECT_NAME})

# Optionally install a sample params file
install(FILES params.yaml DESTINATION share/${PROJECT_NAME})

ament_package()
