cmake_minimum_required(VERSION 3.5)
project(smclib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


# install the include folder
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)
ament_export_include_directories(include)
ament_package()