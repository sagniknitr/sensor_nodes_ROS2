cmake_minimum_required(VERSION 3.5)

project(bondcpp)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find_package(Boost REQUIRED)
find_package(bond REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  
find_package(smclib REQUIRED)
#find_package(UUID REQUIRED)

include_directories(
  include
)



add_library(${PROJECT_NAME}
  src/timeout.cpp
  src/bond.cpp
  src/BondSM_sm.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
"rclcpp"
"UUID"
"smclib")



# install stuff
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
