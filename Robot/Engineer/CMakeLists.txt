
# =========================================================================
# Engineer 工程车底盘代码库
# =========================================================================
add_library(Engineer OBJECT)

# ===================== 所有源文件（严格匹配文件名/大小写/路径） =====================
target_sources(Engineer
        PRIVATE
        # === 入口与初始化 ===
        pyro_init_thread.cpp
        pyro_mission_planner.cpp
        #pyro_debug_task.cpp
        #现在我用的不是队里面的debug 模式，我要用的是我们工程自己开发的Debug模式

        # === Chassis 底盘模块 ===
        Chassis/pyro_engineer_chassis.cpp
        Chassis/fsm/pyro_engineer_passive_state.cpp
        Chassis/fsm/pyro_engineer_active_state.cpp
        Chassis/pyro_debug_thread.cpp
        # === Communication 通信 ===
        #Communication/Chassis_board/pyro_chassis_interboard_com.cpp
        Communication/DataBoard/pyro_databoard.cpp
        #Communication/Chassis_board/pyro_chassis_interboard_com_old.cpp
        # === Application 应用层（写好后取消注释） ===
        Application/Chassis_board/pyro_chassis_app.cpp
    )

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}              # Engineer 根目录
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis      # 底盘模块头文件
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication/DataBoard  # DataBoard
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Chassis_board  # 板间通信
        
)

# ===================== 编译宏定义 =====================
target_compile_definitions(Engineer PUBLIC
        CHASSIS_BOARD    # 标记当前是底盘板
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
