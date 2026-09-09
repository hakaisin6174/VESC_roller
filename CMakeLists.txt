cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME vesc_current_control)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

# Fetch omuraisu-library directly from GitHub during CMake configuration.
include(FetchContent)
set(OMURAISU_CAN_STM32_ENABLE ON CACHE BOOL "Enable STM32 CAN adapter" FORCE)
set(OMURAISU_CAN_STM32_FDCAN_ENABLE OFF CACHE BOOL "Disable FDCAN on STM32F446" FORCE)
set(OMURAISU_SERIAL_STM32_ENABLE OFF CACHE BOOL "Serial adapter is unused" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build dependency examples" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Do not build dependency tests" FORCE)

FetchContent_Declare(
    omuraisu
    GIT_REPOSITORY https://github.com/omuct-robotclub/omuraisu-library.git
    GIT_TAG main
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(omuraisu)

# can_stm32.c includes main.h, so the dependency target needs the Cube/HAL setup.
target_compile_definitions(omuraisu_can PRIVATE
    USE_HAL_DRIVER
    STM32F446xx
)
target_include_directories(omuraisu_can PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    omuraisu_can
    omuraisu_dji
    omuraisu_pid
    omuraisu_vesc
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating HEX and BIN firmware images"
)
