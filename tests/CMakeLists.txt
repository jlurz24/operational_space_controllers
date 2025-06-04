cmake_minimum_required(VERSION 2.4.6) # Or higher, matching main CMakeLists
project(operational_space_controllers_tests)

# Link against your controller library (adjust name if necessary)
# We assume the main library is called 'force_controller' as per previous CMakeLists
# Add include directories for your project and GTest
include_directories(
    ${PROJECT_SOURCE_DIR}/include # For force_controller.hpp
    ${GTEST_INCLUDE_DIRS}
)

# Add a test executable
add_executable(test_force_controller test_force_controller.cpp)
target_link_libraries(test_force_controller
    force_controller # Your controller library
    ${GTEST_LIBRARIES}
    pthread # GTest often requires pthread
)

# Add the test to CTest
add_test(NAME test_force_controller COMMAND test_force_controller)
