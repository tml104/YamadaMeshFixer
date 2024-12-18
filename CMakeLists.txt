cmake_minimum_required(VERSION 3.10)

project(YamadaMeshFixer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find spdlog
find_package(spdlog REQUIRED)
if(spdlog_FOUND)
    message(STATUS "spdlog found: ${spdlog_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "spdlog not found")
endif()

set (targetName YamadaMeshFixer)

add_definitions(-DTINYOBJLOADER_USE_DOUBLE)

# Include directories
include_directories(${spdlog_INCLUDE_DIRS})
include(
    ./include/SpdlogDef.hpp
)
include_directories(
    ./include
)

file(GLOB SOURCES "src/*.cpp" "include/*.cpp")

add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
add_executable (${targetName} ${SOURCES})
target_compile_definitions(${targetName} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${targetName} spdlog::spdlog)