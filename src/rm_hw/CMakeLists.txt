cmake_minimum_required(VERSION 3.8)
project(rm_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rm_common REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)
ament_target_dependencies( ${PROJECT_NAME}
        rm_common
        hardware_interface
        xmlrpcpp
)
include_directories(
        ${PROJECT_SOURCE_DIR}/install/rm_common/include
)
include_directories(
        ${PROJECT_SOURCE_DIR}/install/rm_hw/include
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
install(
        DIRECTORY include
        DESTINATION include
)
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

ament_auto_package()
