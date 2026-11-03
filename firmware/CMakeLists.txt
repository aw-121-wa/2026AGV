cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 2026AGV)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/Drivers/HWT101/hwt101.c
    User/Drivers/ZDT_X42S/zdt_x42s.c
    User/Chassis/chassis.c
    User/Comm/chassis_cmd.c
    User/Comm/chassis_telemetry.c
    User/Comm/console.c
    User/Comm/uart_callbacks.c
    User/Navigation/map.c
    User/Navigation/astar.c
    User/Navigation/survey.c
    User/App/agv_run.c
    # 比赛模式：串口 / C1 雷达 / 雷达舵机 / 障碍识别 / 视觉协议 / 任务码（其他模式不调用）
    User/Drivers/SerPort/ser_port.c
    User/Drivers/RPLidar/rplidar.c
    User/Drivers/Servo/servo.c
    User/Navigation/lidar_obstacle.c
    User/Comm/vision_link.c
    User/App/competition.c
    User/App/task_code.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/Config
    User/Common
    User/Drivers/HWT101
    User/Drivers/ZDT_X42S
    User/Drivers/SerPort
    User/Drivers/RPLidar
    User/Drivers/Servo
    User/Chassis
    User/Comm
    User/Navigation
    User/App
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Optional build-directory overrides; otherwise edit User/Config/agv_config.h.
if(DEFINED AGV_MODE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AGV_MODE=${AGV_MODE})
endif()
if(DEFINED AGV_START_ZONE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AGV_START_ZONE=${AGV_START_ZONE})
endif()
foreach(key AGV_SURVEY_LAPS AGV_STOP_EACH_SEGMENT)
    if(DEFINED ${key})
        if(NOT "${${key}}" MATCHES "^[0-9]+$")
            message(FATAL_ERROR "${key} must be an integer")
        endif()
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ${key}=${${key}})
    endif()
endforeach()
# 底盘单独调试：-DCHASSIS_USE_IMU=0 不接陀螺仪；-DCHASSIS_WHEEL_DIR_RF=-1 等修正电机安装方向。
foreach(key CHASSIS_USE_IMU CHASSIS_WHEEL_DIR_LF CHASSIS_WHEEL_DIR_RF CHASSIS_WHEEL_DIR_LR
        CHASSIS_WHEEL_DIR_RR)
    if(DEFINED ${key})
        if(NOT "${${key}}" MATCHES "^-?[0-9]+$")
            message(FATAL_ERROR "${key} must be an integer")
        endif()
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ${key}=${${key}})
    endif()
endforeach()
# 巡检速度允许小数，例如 -DAGV_SURVEY_SPEED_MM_S=200 或 =187.5。
if(DEFINED AGV_SURVEY_SPEED_MM_S)
    if(NOT "${AGV_SURVEY_SPEED_MM_S}" MATCHES "^[0-9]+([.][0-9]+)?$")
        message(FATAL_ERROR "AGV_SURVEY_SPEED_MM_S must be a positive number")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        AGV_SURVEY_SPEED_MM_S=${AGV_SURVEY_SPEED_MM_S})
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
