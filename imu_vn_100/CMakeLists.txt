cmake_minimum_required(VERSION 3.5)
<<<<<<< HEAD
project(imu_vn_100)

set(CMAKE_CXX_FLAGS "-std=gnu++11")

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   diagnostic_updater
# )
find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)
# Uncomment below package once ported to ROS 2
#find_package)diagnostic_updater)

# catkin_package(
#   INCLUDE_DIRS include vncpplib/include
#   LIBRARIES imu_vn_100
#   CATKIN_DEPENDS diagnostic_updater roscpp sensor_msgs
#   DEPENDS Boost
# )

include_directories(
  include
  vncpplib/include
 # ${catkin_INCLUDE_DIRS}
)


add_library(imu_vn_100 SHARED
  vncpplib/src/arch/linux/vncp_services.c
  vncpplib/src/vndevice.c
  vncpplib/src/vn100.c
  src/imu_vn_100.cpp
)
target_link_libraries(imu_vn_100 
${Boost_LIBRARIES}
${sensor_msgs_LIBRARIES}
${rclcpp_LIBRARIES}
)

ament_target_dependencies(imu_vn_100
"sensor_msgs"
"rclcpp"
"builtin_interfaces")

add_dependencies(imu_vn_100
  ${${PROJECT_NAME}_EXPORTED_TARGETS}

)

add_executable(imu_vn_100_cont_node
  src/imu_vn_100_cont.cpp)
target_link_libraries(imu_vn_100_cont_node
  imu_vn_100
)

add_dependencies(imu_vn_100_cont_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
ament_target_dependencies(imu_vn_100_cont_node
"sensor_msgs"
"rclcpp"
"builtin_interfaces")


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
=======

project(demo_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lpcl_filters)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "sensor_msgs"
    "pcl_conversions")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Tutorials of Rviz Basic Shapes with Topics
custom_executable(basic_pcl)
target_link_libraries(basic_pcl ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pcl_filters)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
  set(tutorial_tests
    list_parameters_async
    list_parameters
    parameter_events_async
    parameter_events
    set_and_get_parameters_async
    set_and_get_parameters
    "talker:listener")
  set(service_tutorial_tests
    "add_two_ints_server:add_two_ints_client"
    "add_two_ints_server:add_two_ints_client_async"
  )

  macro(tests)
    set(tutorial_tests_to_test ${tutorial_tests})
    list(APPEND tutorial_tests_to_test ${service_tutorial_tests})

    foreach(tutorial_test ${tutorial_tests_to_test})
      string(REPLACE ":" ";" tutorial_executables "${tutorial_test}")
      set(DEMO_NODES_CPP_EXPECTED_OUTPUT "")
      foreach(executable ${tutorial_executables})
        list(APPEND DEMO_NODES_CPP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
      endforeach()

      set(DEMO_NODES_CPP_EXECUTABLE "")
      foreach(executable ${tutorial_executables})
        list(APPEND DEMO_NODES_CPP_EXECUTABLE "$<TARGET_FILE:${executable}>")
      endforeach()

      string(REPLACE ";" "_" exe_list_underscore "${tutorial_executables}")
      configure_file(
        test/test_executables_tutorial.py.in
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      ament_add_pytest_test(test_tutorial_${exe_list_underscore}${target_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        TIMEOUT 30
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation})
      foreach(executable ${tutorial_executables})
        set_property(
          TEST test_tutorial_${exe_list_underscore}${target_suffix}
          APPEND PROPERTY DEPENDS ${executable}${target_suffix})
      endforeach()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

ament_package()
>>>>>>> a06bd5f69344b449233064c301f4145c9e0e2107
