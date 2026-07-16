cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
#设置机器人ID
include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/config/pyro_robot_id_config.cmake)
set(CMAKE_SYSTEM_NAME Generic)
#设置编译器

include(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/gcc-arm-none-eabi.cmake)

#2.项目定义
set(CMAKE_PROJECT_NAME PYRo_Robot)
project(${CMAKE_PROJECT_NAME} C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
# Define the build type

# Set the project name
add_compile_options(
    "-mthumb"
    "-Wdouble-promotion"
    "-funsafe-math-optimizations"
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CMSIS-DSP")

add_compile_definitions(
    ARM_MATH_CM7
    ARM_MATH_LOOPUNROLL
) 
set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)


add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include"
)


add_library(dsppp_lib INTERFACE)

target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)



add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/stm32cubemx)
add_subdirectory(PYRo)
if(ROBOT_ID EQUAL ENGINEER_ID)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Robot/Engineer)
    target_link_libraries(${CMAKE_PROJECT_NAME} Engineer)
endif()
target_link_libraries(${CMAKE_PROJECT_NAME}
    PYRo
    stm32cubemx
    CMSISDSP
    dsppp_lib
    # Add user defined libraries
)

add_dependencies(echo_robot_id ${CMAKE_PROJECT_NAME})