cmake_minimum_required(VERSION 3.5)
project(ur10e_with_lift)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()
