cmake_minimum_required(VERSION 3.8)
project(microduck_gazebo)
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

# XL330 servo with rotor inertia (see src/MicroduckServo.cc).
add_library(MicroduckServo SHARED src/MicroduckServo.cc)
target_compile_features(MicroduckServo PRIVATE cxx_std_17)
target_link_libraries(MicroduckServo gz-sim::gz-sim gz-plugin::register gz-transport::gz-transport)
install(TARGETS MicroduckServo LIBRARY DESTINATION lib)

install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
