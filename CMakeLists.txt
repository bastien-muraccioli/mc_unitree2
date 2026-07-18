cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_unitree)
set(PROJECT_DESCRIPTION "robot controller for mc_rtc using unitree sdk2")

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(unitree_sdk2 REQUIRED)
find_package(Threads REQUIRED)
find_package(CLI11 REQUIRED)

# Import data tamer library
add_subdirectory(libfort)
add_subdirectory(src)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/etc/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/mc_unitree"
        FILES_MATCHING
        PATTERN "*.yaml")
