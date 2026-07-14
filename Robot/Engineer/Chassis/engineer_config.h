#ifndef __ENGINEER_CONFIG_H__
#define __ENGINEER_CONFIG_H__

// =========================================================
// 麦轮底盘参数
// =========================================================
constexpr float WHEEL_RADIUS  = 0.076f;   // 轮子半径 (m)
constexpr float WHEELBASE     = 0.42f;    // 轴距 (m) — 前后轮中心距离
constexpr float TRACK_WIDTH   = 0.42f;    // 轮距 (m) — 左右轮中心距离

// =========================================================
// 矿仓参数（4个位置对应的角度）
// =========================================================
// TODO: 根据实际机械结构调整这4个角度
constexpr float MAGAZINE_ANGLES[4] = {
    0.0f,       // POS_1
    1.57f,      // POS_2 (90度)
    3.14f,      // POS_3 (180度)
    4.71f       // POS_4 (270度)
};
//零点位移 -1.635

// =========================================================
// 摇臂参数
// =========================================================
// TODO: 根据实际机械结构调整
constexpr float LIFT_DEPLOY_ANGLE  = 1.0f;    // 放下时的角度 (rad)
constexpr float LIFT_RETRACT_ANGLE = 0.0f;    // 收起时的角度 (rad)
constexpr float LIFT_MAX_ANGLE     = 1.5f;    // 最大角度限位
constexpr float LIFT_MIN_ANGLE     = -0.2f;   // 最小角度限位

constexpr float LIFT_MANUAL_SPEED  = 1.0f;    // 手动模式角速度 (rad/s)

// =========================================================
// 最大速度限制
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f;   // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f;   // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 6.0f;   // 最大旋转角速度 rad/s

#endif // __ENGINEER_CONFIG_H__
