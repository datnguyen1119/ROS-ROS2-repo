# # # cmake_minimum_required(VERSION 3.16)
# # # project(rehab_arm_robot_hardware_interface LANGUAGES CXX)

# # # if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
# # #   add_compile_options(-Wall -Wextra)
# # # endif()

# # # # set the same behavior for windows as it is on linux
# # # set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# # # # find dependencies
# # # set(THIS_PACKAGE_INCLUDE_DEPENDS
# # #   hardware_interface
# # #   pluginlib
# # #   rclcpp
# # #   rclcpp_lifecycle
# # # )

# # # # find dependencies
# # # find_package(backward_ros REQUIRED)
# # # find_package(ament_cmake REQUIRED)
# # # foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
# # #   find_package(${Dependency} REQUIRED)
# # # endforeach()

# # # ## COMPILE
# # # add_library(
# # #   rehab_arm_robot_hardware_interface
# # #   SHARED
# # #   #hardware/diffbot_system.cpp 
# # #   odrive_hardware_interface.cpp
# # # )
# # # target_compile_features(rehab_arm_robot_hardware_interface PUBLIC cxx_std_17)
# # # target_include_directories(rehab_arm_robot_hardware_interface PUBLIC
# # # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# # # $<INSTALL_INTERFACE:include/rehab_arm_robot_hardware_interface>
# # # )
# # # ament_target_dependencies(
# # #   rehab_arm_robot_hardware_interface PUBLIC
# # #   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# # # )

# # # # Export hardware plugins
# # # pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

# # # # INSTALL
# # # install(
# # #   DIRECTORY rehab_arm_robot_hardware_interface/include/
# # #   DESTINATION include/rehab_arm_robot_hardware_interface
# # # )
# # # install(
# # #   DIRECTORY description/launch description/ros2_control description/urdf
# # #   DESTINATION share/rehab_arm_robot_hardware_interface
# # # )
# # # install(
# # #   DIRECTORY bringup/launch bringup/config
# # #   DESTINATION share/rehab_arm_robot_hardware_interface
# # # )
# # # install(TARGETS rehab_arm_robot_hardware_interface
# # #   EXPORT export_rehab_arm_robot_hardware_interface
# # #   ARCHIVE DESTINATION lib
# # #   LIBRARY DESTINATION lib
# # #   RUNTIME DESTINATION bin
# # # )

# # # # if(BUILD_TESTING)
# # # #   find_package(ament_cmake_pytest REQUIRED)

# # # #   ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
# # # #   ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
# # # #   ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
# # # # endif()

# # # ## EXPORTS
# # # ament_export_targets(export_rehab_arm_robot_hardware_interface HAS_LIBRARY_TARGET)
# # # ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# # # ament_package()
# # cmake_minimum_required(VERSION 3.10)
# # project(rehab_arm_robot_hardware_interface)

# # # Dependencies
# # find_package(ament_cmake REQUIRED)
# # find_package(hardware_interface REQUIRED)
# # find_package(pluginlib REQUIRED)
# # find_package(rclcpp REQUIRED)
# # find_package(can_odrive_interface REQUIRED)  # nếu bạn dùng thư viện này

# # # Thêm thư mục include
# # include_directories(
# #   include
# # )

# # add_library(${PROJECT_NAME}
# #   odrive_hardware_interface.cpp
# # )

# # ament_target_dependencies(${PROJECT_NAME}
# #   hardware_interface
# #   pluginlib
# #   rclcpp
# #   can_odrive_interface
# # )

# # target_include_directories(${PROJECT_NAME}
# #   PUBLIC
# #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# # )

# # pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

# # install(
# #   TARGETS ${PROJECT_NAME}
# #   EXPORT export_${PROJECT_NAME}
# #   ARCHIVE DESTINATION lib
# #   LIBRARY DESTINATION lib
# #   RUNTIME DESTINATION bin
# # )

# # install(
# #   DIRECTORY include/
# #   DESTINATION include/
# # )

# # install(
# #   FILES odrive_hardware_interface.xml
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # ament_package()

# #-----------------------------------------------------------------------------------

# cmake_minimum_required(VERSION 3.10)
# project(rehab_arm_robot_hardware_interface LANGUAGES CXX)

# # Tùy chọn biên dịch
# if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
#   add_compile_options(-Wall -Wextra)
# endif()

# # Hỗ trợ Windows
# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# # Tìm các phụ thuộc
# find_package(ament_cmake REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(can_odrive_interface REQUIRED)

# # Thêm thư viện
# add_library(odrive_system_interface SHARED
#   src/odrive_hardware_interface.cpp
# )

# # Chỉ định chuẩn C++
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# # Thêm thư mục include
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# # Liên kết phụ thuộc
# ament_target_dependencies(${PROJECT_NAME}
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   can_odrive_interface
# )

# # Xuất plugin
# pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

# # Cài đặt
# install(
#   TARGETS odrive_hardware_interface
#   EXPORT export_odrive_system_hardware
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# # install(
# #   DIRECTORY bringup/
# #   DESTINATION share/${PROJECT_NAME}
# # )

# install(
#   FILES odrive_hardware_interface.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# # Xuất target và phụ thuộc
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   can_odrive_interface
# )

# # Kiểm tra (tùy chọn)
# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)
#   # Thêm các bài kiểm tra nếu cần
#   # ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
# endif()

# ament_package()

#------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(rehab_arm_robot_hardware_interface LANGUAGES CXX)

# Tùy chọn biên dịch
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Hỗ trợ Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Tìm các phụ thuộc
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_odrive_interface REQUIRED)

# Thêm thư viện
add_library(odrive_hardware_interface SHARED
  src/odrive_hardware_interface.cpp
)

# Xuất plugin
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

# Chỉ định chuẩn C++
target_compile_features(odrive_hardware_interface PUBLIC cxx_std_17)

# Thêm thư mục include
target_include_directories(odrive_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Liên kết phụ thuộc
ament_target_dependencies(odrive_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_odrive_interface
)


# Cài đặt
install(
  TARGETS odrive_hardware_interface
  EXPORT export_odrive_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES odrive_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Xuất target và phụ thuộc
ament_export_targets(export_odrive_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_odrive_interface
)

# Kiểm tra (tùy chọn)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(controller_manager REQUIRED)
  # find_package(launch_testing_ros REQUIRED)
  # find_package(liburdfdom-tools REQUIRED)
  # find_package(ros2_controllers_test_nodes REQUIRED)
  find_package(xacro REQUIRED)
endif()

ament_package()