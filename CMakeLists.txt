cmake_minimum_required(VERSION 3.19)

set(MBED_APP_JSON_PATH mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(lpc1768-serprog LANGUAGES C CXX ASM) 
include(mbed_project_setup)

add_subdirectory(mbed-os)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE inc)
target_link_libraries(${PROJECT_NAME} mbed-baremetal)
target_link_libraries(${PROJECT_NAME} mbed-usb)

mbed_set_post_build(${PROJECT_NAME})
