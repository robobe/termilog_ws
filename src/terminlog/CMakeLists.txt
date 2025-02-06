cmake_minimum_required(VERSION 3.8)
project(terminlog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(NODES
  terminlog/viewer.py
  terminlog/test_node.py
)

set(APP_FILES
  terminlog/view_tui.py
  terminlog/tui_style.tcss
  terminlog/input_modal.py
)

install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    ${APP_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/debian/control/postinst"
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(Packing)


ament_package()
