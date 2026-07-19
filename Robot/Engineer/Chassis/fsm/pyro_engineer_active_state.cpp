#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：主动状态
// =========================================================
namespace pyro{


void engineer_chassis_t::state_active_t::enter(owner *owner)
{
    // 进入主动状态：使能电机、清空PID积分

    for (auto *motor : owner->_ctx.motor.mecanum)
    {
        motor->enable();
    }
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        pid->clear();
    }
    owner->_ctx.motor.magazine->enable();
    owner->_ctx.pid.magazine_pos_pid->clear();
    owner->_ctx.pid.magazine_vel_pid->clear();
    // TODO: 摇臂、矿仓同样处理
}

void engineer_chassis_t::state_active_t::execute(owner *owner)
{



    // 主动状态完整控制流水线

    // 1. 运动学逆解
    owner->_kinematics_solve();

    // 2. 麦轮速度环
    owner->_mecanum_control();

    // 3. 摇臂控制
    owner->_lift_control();

    // 4. 矿仓控制
    owner->_magazine_control();

    // 5. 功率限制
    owner->_power_control();

    // 6. 发送所有电机指令
    owner->_send_motor_command();
}

void engineer_chassis_t::state_active_t::exit(owner *owner)
{
    // 退出主动状态（一般空的）
}
}