#include "pyro_engineer_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"
#include <algorithm>
#include <arm_math.h>
#include <cmath>

namespace pyro
{

// =========================================================
// 构造函数
// =========================================================
engineer_chassis_t::engineer_chassis_t()
    : module_base_t("engineer_chassis")

{
    _ctx = {};
    // _ctx 是基类的 protected 成员，会被默认构造
}

// =========================================================
// 获取上下文（非 const 版本，基类只有 const 版本）
// =========================================================
engineer_context_t &engineer_chassis_t::get_ctx()
{
    return _ctx;  // 直接访问基类的 protected 成员 _ctx
}

// =========================================================
// _init()：初始化回调（只调用一次）
// =========================================================
//这里的功率控制我先不加上,怕对我的最基本的代码功能产生影响

status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 2. new 麦轮运动学求解器
    _kinematics = new mecanum_kin_t(WHEELBASE, TRACK_WIDTH);

    // 3. new 功率计并初始化（如果有的话）
    
    // _ctx.powermeter = new powermeter_drv_t(0x212, can_hub_t::can2);
    // _ctx.powermeter->init();

    // 4. 功率控制初始化
    // _power_control_init();
    // 5. 摇臂初始化
    // TODO: 摇臂、矿仓的初始化写在这里
    //先不管它了
    //如果初始化这里都没什么问题，直接返回PYRO_OK
    return PYRO_OK;
}

// =========================================================
// 功率控制初始化
// =========================================================
void engineer_chassis_t::_power_control_init()
{
    //4个麦轮的拟合系数
    // power_fit_params_t params[4] = {
        // {0.0115f, 0.0391f, 0.2739f,  -3.2137f},   // \[0\] FL 前左
        // {0.0114f, 0.0214f, 0.2181f,   0.4862f},   // \[1\] FR 前右
        // {0.0120f, 0.0353f, 0.3015f,  -4.8325f},   // \[2\] BR 后右
        // {0.0111f, 0.0430f, 0.4013f, -11.0010f},   // \[3\] BL 后左
    // }
    //公式是 预测功率 = k1 * 扭矩^2 + k2 * 转速 * 扭矩 + k3 * |扭矩| + K4
    //祝哥的数据
    // for(int i = 0 ; i < 4;i++){
    //     _ctx.power_motor_data[i] = power_controller_t::get_instance().register_motor(parmas[i]);
    // }
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    // ===== 1. 更新所有电机反馈 =====

    // 麦轮电机
    for (auto *motor : _ctx.motor.mecanum)
    {
        motor->update_feedback();
    }

    // 摇臂电机
    // for (auto *motor : _ctx.motor.lift)
    // {
    //     if (motor != nullptr)
    //         motor->update_feedback();
    // }

    // 矿仓电机
    // if (_ctx.motor.magazine != nullptr)
    // {
    //     _ctx.motor.magazine->update_feedback();
    // }

    // ===== 2. 读取麦轮数据 =====
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.mecanum[i]->get_current_rotate() *
                         dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.current_wheel_torque[i] =
            _ctx.motor.mecanum[i]->get_current_torque();
        _ctx.data.current_wheel_temp[i] =
            _ctx.motor.mecanum[i]->get_temperature();
        _ctx.data.wheel_online[i] =
            _ctx.motor.mecanum[i]->is_online();
    }

    // ===== 3. 里程计反算实际速度 =====
    mecanum_kin_t::wheel_speeds_t current_speed{};
    current_speed.fl = rpm_to_mps(_ctx.data.current_wheel_rpm[0], WHEEL_RADIUS);
    current_speed.fr = rpm_to_mps(-_ctx.data.current_wheel_rpm[1], WHEEL_RADIUS);
    current_speed.bl = rpm_to_mps(_ctx.data.current_wheel_rpm[2], WHEEL_RADIUS);
    current_speed.br = rpm_to_mps(-_ctx.data.current_wheel_rpm[3], WHEEL_RADIUS);
    //这里是更新后的库的方便性，相当于直接计算了我麦轮的实际速度

    _kinematics->compute_odometry(current_speed,
                                  _ctx.data.real_vx,
                                  _ctx.data.real_vy,
                                  _ctx.data.real_wz);

    // ===== 4. TODO: 读取摇臂反馈 =====
    // for (int i = 0; i < 2; i++) {
    //     _ctx.data.current_lift_angle[i] = ...;
    //     _ctx.data.current_lift_speed[i] = ...;
    //     _ctx.data.lift_online[i] = ...;
    // }

    // ===== 5. TODO: 读取矿仓反馈 =====
    // _ctx.data.current_magazine_angle = ...;

    // ===== 6. 裁判系统功率数据 =====
    // auto *referee = referee_drv_t::get_instance();
    // const auto &ref_data = referee->get_data();
    // _ctx.data.buffer_energy = ref_data.power_heat.buffer_energy;
    // _ctx.data.total_predicted_power =
        // power_controller_t::get_instance().get_total_predicted_power();
}

// =========================================================
// _kinematics_solve()：麦轮逆运动学
// =========================================================
void engineer_chassis_t::_kinematics_solve()
{
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx,
        _ctx.cmd->chassis.vy,
        _ctx.cmd->chassis.wz,
        mecanum_kin_t::missing_mec_e::NONE
    );

    // 线速度 m/s ->转速 RPM
    _ctx.data.target_wheel_rpm[0] = mps_to_rpm(wheel_speeds.fl,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] = mps_to_rpm(wheel_speeds.fr,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] = mps_to_rpm(wheel_speeds.bl,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] = mps_to_rpm(wheel_speeds.br,WHEEL_RADIUS);

}

// =========================================================
// _mecanum_control()：麦轮速度环PID
// =========================================================
void engineer_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_wheel_torque[i] =
            _ctx.pid.mecanum_pid[i]->calculate(
                _ctx.data.target_wheel_rpm[i],
                _ctx.data.current_wheel_rpm[i]);
    }
}

// =========================================================
// _lift_control()：摇臂控制
// =========================================================
void engineer_chassis_t::_lift_control()
{
    // TODO: 根据模式分发
    // if (_ctx.cmd->lift.mode == lift_mode_t::AUTO) {
    //     自动模式：根据 auto_action 设置目标角度
    // } else {
    //     手动模式：根据 manual.left_mod/right_mod 控制
    // }
    //
    // 然后位置环PID → 速度环PID → 输出扭矩
}

// =========================================================
// _magazine_control()：矿仓控制
// =========================================================
void engineer_chassis_t::_magazine_control()
{
    // TODO: 根据 target_pos 查角度表，位置环PID
    // float target_angle = MAGAZINE_ANGLES[static_cast<int>(_ctx.cmd->magazine.target_pos)];
    // _ctx.data.out_magazine_torque = _ctx.pid.magazine_pos_pid->calculate(...);
}

// =========================================================
// _power_control()：功率限制
// =========================================================
void engineer_chassis_t::_power_control()
{
    // TODO: 参考 Sub_Hero 的写法
    // 把4个麦轮的扭矩送到功率控制器，算出安全扭矩
}

// =========================================================
// _send_motor_command()：发送所有电机指令
// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 麦轮
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.mecanum[i]->send_torque(_ctx.data.out_wheel_torque[i]);
    }

    // TODO: 摇臂
    // for (int i = 0; i < 2; i++) {
    //     _ctx.motor.lift[i]->send_torque(_ctx.data.out_lift_torque[i]);
    // }

    // TODO: 矿仓
    // _ctx.motor.magazine->send_torque(_ctx.data.out_magazine_torque);
}

// =========================================================
// _fsm_execute()：状态机执行
// =========================================================
void engineer_chassis_t::_fsm_execute()
{
    // 1. 把当前命令指针存到 ctx
    _ctx.cmd = &_current_cmd;  // _current_cmd 也是基类的 protected 成员

    // 2. 根据 mode 切换顶层状态
    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    // 3. 执行当前状态
    _main_fsm.execute(this);
}


} // namespace pyro
