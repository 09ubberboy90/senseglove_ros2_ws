cmake_minimum_required(VERSION 3.8)
project(senseglove_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(senseglove_hardware REQUIRED)
find_package(senseglove_description REQUIRED)
find_package(senseglove_hardware_builder REQUIRED)
find_package(senseglove_shared_resources REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(backward_ros REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_library(SGCORE
    NAMES libSGCoreCpp.so
    PATHS ../../SenseGlove_API/Core/SGCoreCpp/lib/linux/Release
    REQUIRED)

find_library(SGCONNECT
    NAMES libSGConnect.so
    PATHS ../../SenseGlove_API/Core/SGConnect/lib/linux
    REQUIRED)

set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

include_directories(include
        SYSTEM
        ../../SenseGlove_API/Core/SGConnect/incl
        ../../SenseGlove_API/Core/SGCoreCpp/incl)

add_library(${PROJECT_NAME} SHARED
        src/${PROJECT_NAME}.cpp
        )
target_include_directories(${PROJECT_NAME} 
        PRIVATE
        include
      )

ament_target_dependencies(
        senseglove_hardware_interface
        controller_manager
        joint_trajectory_controller
        joint_state_broadcaster
        realtime_tools
        rclcpp
        senseglove_description
        senseglove_hardware
        senseglove_hardware_builder
        senseglove_shared_resources
        std_msgs
        hardware_interface
      )
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

add_executable(${PROJECT_NAME}_node 
        src/${PROJECT_NAME}_node.cpp
        src/${PROJECT_NAME}.cpp
        include/senseglove_hardware_interface/senseglove_hardware_interface.h
        )
target_include_directories(${PROJECT_NAME}_node PUBLIC ${controller_manager_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${controller_manager_LIBRARIES})
ament_target_dependencies(
        senseglove_hardware_interface_node
        controller_manager
        joint_trajectory_controller
        joint_state_broadcaster
        realtime_tools
        rclcpp
        senseglove_description
        senseglove_hardware
        senseglove_hardware_builder
        senseglove_shared_resources
        std_msgs
        hardware_interface
      )
        
target_link_libraries(${PROJECT_NAME} ${SGCONNECT} ${SGCORE} yaml-cpp)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )
ament_export_include_directories(
  include
)

install(TARGETS ${PROJECT_NAME}
DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/)

ament_export_libraries(
  ${PROJECT_NAME}
)  

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

ament_package()
