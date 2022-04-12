cmake_minimum_required(VERSION 3.8)
project(senseglove_hardware_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(senseglove_hardware REQUIRED)

find_library(SGCORE
    NAMES libSGCoreCpp.so
    PATHS ../../SenseGlove_API/Core/SGCoreCpp/lib/linux/Release
    REQUIRED)

find_library(SGCONNECT
    NAMES libSGConnect.so
    PATHS ../../SenseGlove_API/Core/SGConnect/lib/linux
    REQUIRED)

include_directories(include
    SYSTEM
    ../../SenseGlove_API/Core/SGConnect/incl
    ../../SenseGlove_API/Core/SGCoreCpp/incl)

add_library(${PROJECT_NAME}
        SHARED
        src/hardware_builder.cpp
        )
# target_include_directories(${PROJECT_NAME} 
#         PRIVATE
#         include
#       )

target_link_libraries(${PROJECT_NAME} ${SGCONNECT} ${SGCORE} yaml-cpp )

ament_target_dependencies(
  senseglove_hardware_builder
  senseglove_hardware
  rclcpp
)

install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
        )

install(
  TARGETS  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
        )
        
install(DIRECTORY include/
  DESTINATION include/)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_package()
