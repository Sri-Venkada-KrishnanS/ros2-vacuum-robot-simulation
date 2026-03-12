cmake_minimum_required(VERSION 3.8)
project(vacuum_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install python script as executable
install(PROGRAMS
  scripts/vacuum_logic.py
  scripts/simple_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
