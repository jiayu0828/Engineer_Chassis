/**
 * @file pyro_chassis_interboard_com_old.cpp
 * @brief 板间通信 - 旧版协议输入 / 新版Topic输出（无缝切换适配层）
 *
 * 输入：旧版上位机 22 字节帧（遥控器原始通道值）
 * 输出：和新版 interboard_com 完全一致的 DataBoard topic
 * 用法：CMake 里切换编译 old 或 new，上层代码零改动
 */

#include "pyro_bsp_uart.h"
#include "pyro_databoard.h"
#include "pyro_core_def.h"
#include "engineer_config.h"

#include <cstring>
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

using namespace pyro;

/* ==================== 常量 ==================== */
static constexpr uint16_t FRAME_HEADER   = 0xA5A5;
static constexpr uint8_t  RX_QUEUE_SIZE  = 10;
static constexpr uint32_t CALLBACK_OWNER = 0x01;
static constexpr uint32_t TIMEOUT_MS     = 100;

/* ==================== 速度转换系数 ==================== */
// 旧代码速度增益：满摇杆 → 电机端 RPM
static constexpr float VEL_GAIN = 800.0f;
static constexpr float WZ_GAIN  = 1200.0f;

// 电机端 RPM → 轮端线速度 m/s
static constexpr float RPM_TO_MPS =
    2.0f * 3.1415926535f * WHEEL_RADIUS / (60.0f * 19.20320855614973f);

// 电机端 RPM → 车体角速度 rad/s
static constexpr float TURN_RADIUS   = (WHEELBASE + TRACK_WIDTH) * 0.5f;
static constexpr float RPM_TO_RADPS  = RPM_TO_MPS / TURN_RADIUS;

/* ==================== 旧版通信帧（22字节） ==================== */
#pragma pack(push, 1)

struct upper_board_tx_frame_t
{
    uint16_t frame_header;
    uint8_t  sw_l;
    uint8_t  sw_r;
    int16_t  rc_ch_lx;
    int16_t  rc_ch_ly;
    int16_t  rc_ch_rx;
    int16_t  rc_ch_ry;
    uint8_t  zero_force;
    float    magazine_angle;
    uint8_t  which_motion;
    uint8_t  which_mine;
    uint8_t  overpass_pose;
    uint16_t crc16;
};

struct lower_board_tx_frame_t
{
    uint16_t frame_header;
    uint16_t crc16;
};

#pragma pack(pop)

/* ==================== 内部变量 ==================== */
static uart_drv_t     *s_uart      = nullptr;
static databoard      *s_databoard = nullptr;
static QueueHandle_t   s_rx_queue  = nullptr;
static TaskHandle_t    s_com_task  = nullptr;

static TickType_t s_last_rx_tick = 0;
static bool       s_is_online    = false;

// 调试计数
static uint32_t s_debug_cb_count    = 0;
static uint32_t s_debug_parse_count = 0;

/* ==================== DataBoard Topic（和新版完全一致！） ==================== */
static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;
static uint32_t s_topic_online;
static uint32_t s_topic_send_online;//用来发送的是否在线
static uint32_t s_topic_magazine_pos;
static uint32_t s_topic_lift_mode;
static uint32_t s_topic_lift_auto_action;
static uint32_t s_topic_lift_manual;

/* ==================== CRC16（和旧版算法完全一致） ==================== */
static uint16_t crc16_old(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc = ((crc ^ data[i]) & 0x00FF) | (crc & 0xFF00);
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
            else         { crc >>= 1; }
        }
    }
    return crc;
}

/* ==================== 矿仓角度 → 4档位映射 ==================== */
static uint8_t angle_to_magazine_pos(float angle)
{
    const float pos_angles[4] = {
        MAGAZINE_ANGLES[0], MAGAZINE_ANGLES[1],
        MAGAZINE_ANGLES[2], MAGAZINE_ANGLES[3],
    };
    uint8_t best_pos = 0;
    float min_diff = fabsf(angle - pos_angles[0]);
    for (uint8_t i = 1; i < 4; i++)
    {
        float diff = fabsf(angle - pos_angles[i]);
        if (diff < min_diff) { min_diff = diff; best_pos = i; }
    }
    return best_pos;
}

/* ==================== 接收回调（和原版旧代码一致：不校验长度） ==================== */
static bool rx_callback(uint8_t *buf, uint16_t size,
                        BaseType_t &xHigherPriorityTaskWoken)
{
    s_debug_cb_count++;

    if (size == 0) return false;         // 和原版一致：只判断不为0
    if (size > sizeof(upper_board_tx_frame_t)) return false;  // 防越界

    xQueueSendFromISR(s_rx_queue, buf, &xHigherPriorityTaskWoken);
    return true;
}

/* ==================== 解析旧帧 → 写入新版Topic ==================== */
static void rx_parse_and_publish()
{
    uint8_t rx_buf[sizeof(upper_board_tx_frame_t)];
    if (xQueueReceive(s_rx_queue, rx_buf, 0) != pdTRUE) return;

    auto *frame = reinterpret_cast<upper_board_tx_frame_t *>(rx_buf);

    // 1. 帧头校验
    if (frame->frame_header != FRAME_HEADER) return;

    // 2. CRC校验
    uint16_t calc_crc = crc16_old(rx_buf + 2, sizeof(upper_board_tx_frame_t) - 4);
    if (calc_crc != frame->crc16) return;

    s_debug_parse_count++;

    // 3. 更新在线状态
    s_last_rx_tick = xTaskGetTickCount();
    if (!s_is_online)
    {
        s_is_online = true;
        genenral_data_t data;
        data.data_ui = 1;
        s_databoard->write_topic(s_topic_online, data);
    }

    genenral_data_t data;

    // 4. 通道值 → 物理单位（和旧代码速度增益一致）
    float ch_lx = (float)frame->rc_ch_lx / 1000.0f;
    float ch_ly = (float)frame->rc_ch_ly / 1000.0f;
    float ch_rx = (float)frame->rc_ch_rx / 1000.0f;

    float vx = ch_ly * VEL_GAIN * RPM_TO_MPS;   // 前后 m/s
    float vy = ch_lx * VEL_GAIN * RPM_TO_MPS;   // 左右 m/s
    float wz = ch_rx * WZ_GAIN  * RPM_TO_RADPS; // 旋转 rad/s

    data.data_f = vx; s_databoard->write_topic(s_topic_vx, data);
    data.data_f = vy; s_databoard->write_topic(s_topic_vy, data);
    data.data_f = wz; s_databoard->write_topic(s_topic_wz, data);

    // 5. 使能判断（和旧代码逻辑一致）
    bool enable = (frame->zero_force == 0) && (frame->sw_r != 1);
    data.data_ui = enable ? 1u : 0u;
    s_databoard->write_topic(s_topic_enable, data);

    // 6. 矿仓：角度 → 档位
    uint8_t mag_pos = angle_to_magazine_pos(frame->magazine_angle);
    data.data_ui = mag_pos;
    s_databoard->write_topic(s_topic_magazine_pos, data);

    // 7. 摇臂：旧版只有自动模式，映射到新版 AUTO
    data.data_ui = 0;  // 0 = lift_mode_t::AUTO
    s_databoard->write_topic(s_topic_lift_mode, data);

    // overpass_pose：0=正常收回  1=越障展开
    if (frame->overpass_pose == 0)
        data.data_ui = 2;  // RETRACT 收回
    else
        data.data_ui = 1;  // DEPLOY 展开
    s_databoard->write_topic(s_topic_lift_auto_action, data);

    // 手动模式：旧版没有，默认停止
    data.data_ui = 0;
    s_databoard->write_topic(s_topic_lift_manual, data);
}

/* ==================== 超时检测 ==================== */
static void check_timeout()
{
    if (s_is_online &&
        (xTaskGetTickCount() - s_last_rx_tick > pdMS_TO_TICKS(TIMEOUT_MS)))
    {
        s_is_online = false;

        genenral_data_t data;
        data.data_ui = 0;
        s_databoard->write_topic(s_topic_online, data);

        data.data_f = 0.0f;
        s_databoard->write_topic(s_topic_vx, data);
        s_databoard->write_topic(s_topic_vy, data);
        s_databoard->write_topic(s_topic_wz, data);
    }
}

/* ==================== 发送回复（旧版最简格式） ==================== */
static void tx_fill_and_send()
{
    lower_board_tx_frame_t tx;
    tx.frame_header = FRAME_HEADER;
    tx.crc16 = crc16_old(reinterpret_cast<uint8_t *>(&tx) + 2,
                         sizeof(tx) - 4);
    if (s_uart)
    {
        s_uart->write(reinterpret_cast<uint8_t *>(&tx), sizeof(tx));
    }
}

/* ==================== 绑定Topic（名字和新版完全一致） ==================== */
static void databoard_topics_init()
{
    s_topic_vx               = s_databoard->get_topic_id("chassis_vx");
    s_topic_vy               = s_databoard->get_topic_id("chassis_vy");
    s_topic_wz               = s_databoard->get_topic_id("chassis_wz");
    s_topic_enable           = s_databoard->get_topic_id("chassis_enable");
    s_topic_online           = s_databoard->get_topic_id("chassis_online");
    s_topic_magazine_pos     = s_databoard->get_topic_id("magazine_pos");
    s_topic_lift_mode        = s_databoard->get_topic_id("lift_mode");
    s_topic_lift_auto_action = s_databoard->get_topic_id("lift_auto_action");
    s_topic_lift_manual      = s_databoard->get_topic_id("lift_manual");
}

/* ==================== 1ms 通信主任务 ==================== */
static void interboard_com_thread(void *argument)
{
    (void)argument;

    while (s_databoard == nullptr) { vTaskDelay(pdMS_TO_TICKS(1)); }
    databoard_topics_init();

    while (true)
    {
        rx_parse_and_publish();
        check_timeout();
        tx_fill_and_send();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ==================== 对外入口（函数名和新版完全一致） ==================== */
extern "C" void chassis_interboard_com_init(databoard *db_ptr)
{
    s_databoard = db_ptr;
    s_uart      = &bsp_uart::get_uart7();

    s_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(upper_board_tx_frame_t));

    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);
    s_uart->enable_rx_dma();

    xTaskCreate(interboard_com_thread,
                "interboard_com",
                512,                    // 栈改大，避免溢出
                nullptr,
                configMAX_PRIORITIES - 3,
                &s_com_task);
}
