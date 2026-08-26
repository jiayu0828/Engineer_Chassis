
# =========================================================================
# Engineer 工程车底盘代码库
# =========================================================================
#set(BOARD "ARM_BOARD")
set(BOARD "CHASSIS_BOARD")
add_library(Engineer OBJECT)

target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
)

target_compile_options(Engineer PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)
set(BASE_SOURCES
        # DataBoard/pyro_databoard.cpp
)

set(ARM_EXTRA_SOURCES
        Arm/pyro_mission_planer.cpp
        Arm/pyro_init_thread.cpp
        Arm/arm_control_execute.cpp
        Arm/arm_app.cpp
)
set(CHASSIS_EXTRA_SOURCES
        pyro_chassis_new/Chassis/pyro_engineer_chassis.cpp
        pyro_chassis_new/Chassis/fsm/pyro_engineer_passive_state.cpp
        pyro_chassis_new/Chassis/fsm/pyro_engineer_active_state.cpp
        pyro_chassis_new/Application/Chassis_board/pyro_chassis_app.cpp
        pyro_chassis_new/Communication/Chassis_board/pyro_chassis_interboard_com.cpp
        pyro_chassis_new/pyro_init_thread.cpp
        pyro_chassis_new/pyro_mission_planner.cpp
        pyro_chassis_new/pyro_ws2812_test_task.cpp

)
if (BOARD STREQUAL "ARM_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${ARM_EXTRA_SOURCES}
    )
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
    )
    target_compile_definitions(Engineer PRIVATE
            DR16_UART=PYRO_UART5
            SELF_CTRL_UART=PYRO_UART10
            DEBUG_UART=PYRO_UART1
            INTERBOARD_UART=PYRO_UART7
    )
elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${CHASSIS_EXTRA_SOURCES})
    
else()
        message(FATAL_ERROR "Unknown BOARD type: ${BOARD}. Please set BOARD to either ARM_BOARD or CHASSIS_BOARD.")
endif()

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new              # Engineer 根目录
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new/Chassis      # 底盘模块头文件
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new/Communication/Chassis_board  # 板间通信
        
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
