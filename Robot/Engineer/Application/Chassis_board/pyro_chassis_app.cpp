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
    static genenral_data_t safe_read_data(uint32_t topic_id){
        genenral_data_t data = {};
        TickType_t timestamp;
        global_databoard->read(topic_id,&data,timestamp);
        return data;
    }
}

// DataBoard topic ID
static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;
static uint32_t s_topic_online;
static uint32_t s_topic_magazine_ready;
static uint32_t s_topic_magazine_pos;
static uint32_t s_topic_lift_mod;
static uint32_t s_topic_mannual;
static uint32_t s_topic_auto;
static uint32_t s_topic_lift_calib_trigger;
static uint32_t s_topic_lift_calib_state;
static uint32_t s_topic_lift_zero_valid;


/* ====================== PID 参数 ====================== */

static constexpr float MECANUM_PID_KP       = 0.3f;
static constexpr float MECANUM_PID_KI       = 0.5f;
static constexpr float MECANUM_PID_KD       = 0.0f;
static constexpr float MECANUM_PID_ILIMIT   = 10.0f;
static constexpr float MECANUM_PID_MAXOUT   = 15.0f;

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

    s_deps->pid_deps.mecanum_pid[0] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//FL
    s_deps->pid_deps.mecanum_pid[1] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//FR
    s_deps->pid_deps.mecanum_pid[2] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//BL
    s_deps->pid_deps.mecanum_pid[3] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//BR


    /* ---------- 后摇臂电机 ---------- */
    s_deps->motor_deps.lift[0] = new dm_motor_drv_t(0x02,0x03,bsp_can::can2);
    s_deps->motor_deps.lift[1] = new dm_motor_drv_t(0x00,0x01,bsp_can::can2);
    //4340
    //J3507
    //设置达秒电机的range 范围
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_rotate_range(-52.0f,52.0f);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_torque_range(-27.0f,27.0f);

    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_rotate_range(-52.0f,52.0f);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_torque_range(-27.0f,27.0f);

    s_deps->pid_deps.lift_pos_pid[0] = new pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_pos_pid[1] = new pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_vel_pid[0] = new pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_vel_pid[1] = new pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);

    /* ---------- 矿仓电机 ---------- */
    s_deps->motor_deps.magazine = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,bsp_can::can3);
    s_deps->pid_deps.magazine_pos_pid = new pid_t(10.0f,0.01f,0.2f,3.0f,10.0f,pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.magazine_vel_pid = new pid_t(0.5f, 0.2f, 0.0f, 1.0f, 3.0f,pid_t::INTEGRAL_LIMIT);
}

/* ====================== 绑定 DataBoard topic ====================== */
static void databoard_topics_bind()
{
    s_topic_vx     = global_databoard->get_topic_id("chassis_vx");
    s_topic_vy     = global_databoard->get_topic_id("chassis_vy");
    s_topic_wz     = global_databoard->get_topic_id("chassis_wz");
    s_topic_enable = global_databoard->get_topic_id("chassis_enable");
    s_topic_online = global_databoard->get_topic_id("chassis_online");
    s_topic_magazine_pos = global_databoard->get_topic_id("magazine_pos");
    s_topic_magazine_ready = global_databoard->get_topic_id("magazine_ready");
    s_topic_lift_mod = global_databoard->get_topic_id("lift_control_mod");
    s_topic_mannual = global_databoard->get_topic_id("lift_mannual");
    s_topic_auto = global_databoard->get_topic_id("lift_auto");
    s_topic_lift_calib_trigger = global_databoard->get_topic_id("lift_calib_trigger");
    s_topic_lift_calib_state   = global_databoard->get_topic_id("lift_calib_state");
    s_topic_lift_zero_valid    = global_databoard->get_topic_id("lift_zero_valid");
}

/* ====================== 从 DataBoard 读数据组装命令 ====================== */
static void chassis_rxcmd()
{


    // 1. 检查板间通信是否在线
    const bool is_online = (safe_read_data(s_topic_online).data_ui != 0);

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
    s_cmd->mode = (safe_read_data(s_topic_enable).data_ui != 0)
                      ? cmd_base_t::mode_t::ACTIVE
                      : cmd_base_t::mode_t::PASSIVE;

    // 3. 速度指令（vx/vy/wz，单位 m/s, rad/s）
    s_cmd->chassis.vx = safe_read_data(s_topic_vx).data_f;

    s_cmd->chassis.vy = safe_read_data(s_topic_vy).data_f;

    s_cmd->chassis.wz = safe_read_data(s_topic_wz).data_f;
    
    //4.  矿仓指令
    switch(safe_read_data(s_topic_magazine_pos).data_ui){
        case 1:
        s_cmd->magazine.target_pos = pyro::magazine_pos_t::POS_1;
        break;
        case 2:
        s_cmd->magazine.target_pos = pyro::magazine_pos_t::POS_2;
        break;
        case 3:
        s_cmd->magazine.target_pos = pyro::magazine_pos_t::POS_3;
        break;
        case 4:
        s_cmd->magazine.target_pos = pyro::magazine_pos_t::POS_4;
        break;
        default:break;
    }    
    //5.  摇臂指令
    uint32_t lift_mode = safe_read_data(s_topic_lift_mod).data_ui;
    if (lift_mode == 0) {
        // ========== 自动模式 ==========
        s_cmd->lift.mode = lift_mode_t::AUTO;
        uint32_t auto_action = safe_read_data(s_topic_auto).data_ui;
        switch (auto_action) {
            case 0:
                s_cmd->lift.auto_action = lift_action_t::HOLD;      // 保持
            break;
            case 1:
                s_cmd->lift.auto_action = lift_action_t::DEPLOY;    // 放下（目标0）
            break;
            case 2:
                s_cmd->lift.auto_action = lift_action_t::RETRACT;   // 收起（目标6）
            break;
            default:
                s_cmd->lift.auto_action = lift_action_t::HOLD;
            break;
        }

    // 自动模式下手动指令无效，清零
        s_cmd->lift.manual.left_mod  = lift_manual_mod_t::HOLD;
        s_cmd->lift.manual.right_mod = lift_manual_mod_t::HOLD;

    } else {
        // ========== 手动模式 ==========
        s_cmd->lift.mode = lift_mode_t::MANUAL;

        uint32_t manual_cmd = safe_read_data(s_topic_mannual).data_ui;
        lift_manual_mod_t mod = lift_manual_mod_t::HOLD;

        switch (manual_cmd) {
            case 0: mod = lift_manual_mod_t::HOLD;  break;
            case 1: mod = lift_manual_mod_t::UP;    break;   // 向上加
            case 2: mod = lift_manual_mod_t::DOWN;  break;   // 向下减
            default: mod = lift_manual_mod_t::HOLD; break;
    }

    // 左右同步（先做同步，以后需要独立再加topic）
        s_cmd->lift.manual.left_mod  = mod;
        s_cmd->lift.manual.right_mod = mod;

    // 手动模式下自动指令无效
    s_cmd->lift.auto_action = lift_action_t::HOLD;
    }
    //6. 摇臂校准触发（上升沿：0→1 才触发一次）
    static uint32_t last_lift_calib_trigger = 0;
    uint32_t calib_trigger = safe_read_data(s_topic_lift_calib_trigger).data_ui;
    if (calib_trigger != 0 && last_lift_calib_trigger == 0) {
        s_chassis->lift_start_calibrate(0);  // 左
        s_chassis->lift_start_calibrate(1);  // 右，同时触发
    }
    last_lift_calib_trigger = calib_trigger;
    //0是AUTO 1是MANNUAL;
    //小shit代码
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

    // ===== 回写摇臂状态 =====
    auto &ctx = s_chassis->get_ctx();

    // 零点有效标志
    genenral_data_t valid_data = {};
    valid_data.data_ui = (ctx.data.lift_zero_valid[0] && ctx.data.lift_zero_valid[1]) ? 1 : 0;
    global_databoard->write_topic(s_topic_lift_zero_valid, valid_data);

    // 校准状态（取两个里进度更靠后的）
    uint8_t s0 = (uint8_t)ctx.data.lift_calib_state[0];
    uint8_t s1 = (uint8_t)ctx.data.lift_calib_state[1];
    genenral_data_t state_data = {};
    state_data.data_ui = (s0 > s1) ? s0 : s1;
    global_databoard->write_topic(s_topic_lift_calib_state, state_data);

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
