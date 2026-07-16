/**
 * @file pyro_chassis_app.cpp
 * @brief 工程车底盘 Application 层
 *
 * 参考 Hero 架构 + 旧工程车硬件参数
 * 职责：创建依赖 → 注入模块 → 1ms 喂命令
 */
extern"C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

#include <cstdint>

#include "pyro_engineer_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_databoard.h"
#include "pyro_bsp_can.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"




using namespace pyro;

/* ====================== 全局指针 ====================== */
static engineer_chassis_t *s_chassis   = nullptr;
static engineer_cmd_t     *s_cmd       = nullptr;
static engineer_deps_t    *s_deps      = nullptr;

// DataBoard 全局指针（init_thread 里创建的）
namespace pyro
{
    extern databoard *global_databoard;
}

// DataBoard topic ID
static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;
static uint32_t s_topic_online;

/* ====================== PID 参数（旧工程车原值，电机端转速） ====================== */
// 旧代码：P=0.4, I=0, D=0, 积分限幅=0, 输出限幅=10
// 注意：旧 PID 输入是电机端转速，新模块输入是轮端 RPM
// 轮端 RPM = 电机端 RPM / 19.2，所以 P 理论上要 ÷19.2 ≈ 0.02
// 先按旧值等比例缩放给初始值，实车再细调
static constexpr float MECANUM_PID_KP       = 0.02f;
static constexpr float MECANUM_PID_KI       = 0.0f;
static constexpr float MECANUM_PID_KD       = 0.0f;
static constexpr float MECANUM_PID_ILIMIT   = 0.0f;
static constexpr float MECANUM_PID_MAXOUT   = 10.0f;

/* ====================== 依赖初始化 ====================== */
static void deps_init()
{
    s_deps = new engineer_deps_t();

    /* ---------- 麦轮电机（旧工程车顺序：CAN1, id1~4） ---------- */
    // 数组索引：[0]FL前左, [1]FR前右, [2]BL后左, [3]BR后右
    // 对应 ID：  id_1       id_2        id_4        id_3
    s_deps->motor_deps.mecanum[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can1); // FL
    s_deps->motor_deps.mecanum[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // FR
    s_deps->motor_deps.mecanum[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BL
    s_deps->motor_deps.mecanum[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can1); // BR

    /* ---------- 麦轮速度环 PID ---------- */
    for (int i = 0; i < 4; i++)
    {
        s_deps->pid_deps.mecanum_pid[i] =
            new pid_t(MECANUM_PID_KP,
                      MECANUM_PID_KI,
                      MECANUM_PID_KD,
                      MECANUM_PID_ILIMIT,
                      MECANUM_PID_MAXOUT,
                      pid_t::INTEGRAL_LIMIT);
    }

    /* ---------- 后摇臂电机（预留，暂不创建） ---------- */
    s_deps->motor_deps.lift[0] = nullptr;
    s_deps->motor_deps.lift[1] = nullptr;
    s_deps->pid_deps.lift_pos_pid[0] = nullptr;
    s_deps->pid_deps.lift_pos_pid[1] = nullptr;
    s_deps->pid_deps.lift_vel_pid[0] = nullptr;
    s_deps->pid_deps.lift_vel_pid[1] = nullptr;

    /* ---------- 矿仓电机（预留，暂不创建） ---------- */
    s_deps->motor_deps.magazine = nullptr;
    s_deps->pid_deps.magazine_pos_pid = nullptr;
    s_deps->pid_deps.magazine_vel_pid = nullptr;
}

/* ====================== 绑定 DataBoard topic ====================== */
static void databoard_topics_bind()
{
    s_topic_vx     = global_databoard->get_topic_id("chassis_vx");
    s_topic_vy     = global_databoard->get_topic_id("chassis_vy");
    s_topic_wz     = global_databoard->get_topic_id("chassis_wz");
    s_topic_enable = global_databoard->get_topic_id("chassis_enable");
    s_topic_online = global_databoard->get_topic_id("chassis_online");
}

/* ====================== 从 DataBoard 读数据组装命令 ====================== */
static void chassis_rxcmd()
{
    genenral_data_t data;
    TickType_t timestamp;

    // 1. 检查板间通信是否在线
    global_databoard->read(s_topic_online, &data, timestamp);
    const bool is_online = (data.data_ui != 0);

    if (!is_online)
    {
        // 离线 → 强制被动，速度清零
        s_cmd->mode = cmd_base_t::mode_t::PASSIVE;
        s_cmd->chassis.vx = 0.0f;
        s_cmd->chassis.vy = 0.0f;
        s_cmd->chassis.wz = 0.0f;
        return;
    }

    // 2. 使能开关
    global_databoard->read(s_topic_enable, &data, timestamp);
    s_cmd->mode = (data.data_ui != 0)
                      ? cmd_base_t::mode_t::ACTIVE
                      : cmd_base_t::mode_t::PASSIVE;

    // 3. 速度指令（vx/vy/wz，单位 m/s, rad/s）
    global_databoard->read(s_topic_vx, &data, timestamp);
    s_cmd->chassis.vx = data.data_f;

    global_databoard->read(s_topic_vy, &data, timestamp);
    s_cmd->chassis.vy = data.data_f;

    global_databoard->read(s_topic_wz, &data, timestamp);
    s_cmd->chassis.wz = data.data_f;

    // TODO: 摇臂、矿仓命令（后续加上）
}

/* ====================== 1ms 喂命令任务 ====================== */
static void chassis_app_thread(void *argument)
{
    (void)argument;

    // 等 DataBoard 初始化完成
    while (global_databoard == nullptr)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    databoard_topics_bind();

    while (true)
    {
        chassis_rxcmd();
        s_chassis->set_command(*s_cmd);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ====================== 对外初始化入口 ====================== */
extern "C" void chassis_app_init()
{
    // 1. new 命令对象
    s_cmd = new engineer_cmd_t();

    // 2. 获取底盘单例
    s_chassis = engineer_chassis_t::instance();

    // 3. 创建所有依赖（电机、PID）
    deps_init();

    // 4. 注入依赖
    s_chassis->configure(*s_deps);

    // 5. 启动模块（内部调用 _init → 创建 1ms 模块任务）
    s_chassis->start();

    // 6. 创建 1ms 喂命令任务
    xTaskCreate(chassis_app_thread,
                "chassis_app",
                512,
                nullptr,
                configMAX_PRIORITIES - 2,
                nullptr);
}
