# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
project(esp32_controller)

string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/ca.crt" TEXT)
