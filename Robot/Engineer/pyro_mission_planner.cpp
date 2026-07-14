/**
 * @file pyro_mission_planner.cpp
 * @brief 任务规划器（总入口，按顺序创建所有任务）
 */

#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "pyro_databoard.h"
#include "FreeRTOS.h"
#include "task.h"

// C 链接函数声明（给 FreeRTOS 任务用）
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);
    extern void chassis_interboard_com_init(pyro::databoard *db_ptr);
}

// 命名空间内的全局变量声明
namespace pyro
{
    extern databoard *global_databoard;
}

/**
 * @brief 任务规划器入口
 *
 * CubeMX 配置的默认任务调用此函数
 * 按顺序初始化所有模块，完成后删除自身
 */
extern "C" void Start_mission_planner(void const *argument)
{
    // 1. 先创建初始化任务（最高优先级，最先跑）
    xTaskCreate(
        pyro_init_thread,
        "pyro_init",
        512,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr);

    // 等待底层驱动初始化完成（databoard 创建好了）
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 初始化板间通信（内部会自己创建 1ms 任务）
    chassis_interboard_com_init(pyro::global_databoard);

    // 3. 调试任务
    xTaskCreate(
        start_debug_task,
        "debug_task",
        512,
        nullptr,
        configMAX_PRIORITIES - 2,
        nullptr);

    // TODO: 4. 初始化底盘模块 Application
    // chassis_app_init();

    // TODO: 5. 初始化矿仓模块
    // magazine_app_init();

    // 规划器任务完成使命，删除自己
    vTaskDelete(nullptr);
}
