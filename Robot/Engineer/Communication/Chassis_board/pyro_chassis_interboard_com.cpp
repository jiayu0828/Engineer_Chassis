#include "pyro_bsp_uart.h"
#include "pyro_databoard.h"
#include "pyro_crc.h"
#include "pyro_core_def.h"

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


using namespace pyro;

/*=================常量===============*/
static constexpr uint16_t FRAME_HEADER = 0xFFA5;
//更正：老代码是0xFFA5
static constexpr uint8_t RX_QUEUE_SIZE = 10;
static constexpr uint32_t CALLBACK_OWNER = 0x01;
static constexpr uint32_t TIMEOUT_MS = 100;//通信超时时间（ms）

/*==============通信帧定义================*/
#pragma pack(push,1)

//上层版->底盘
struct upper_to_lower_frame_t{
    uint16_t frame_header;//帧头 0xA5A5
    //状态切换：使能/失能
    int16_t enable;
    //底盘速度指令
    float vx;
    float vy;
    float wz;

    //矿仓控制
    int16_t magazine_pos;


    //对于摇臂控制现在先不管他

    uint16_t crc16;//crc16校验位

};
//底层板 -> 上层板
struct lower_to_upper_frame_t
{
    uint16_t frame_header;
    
    //底盘状态检查 没错置0 有错置 -1
    int16_t chassis_online;

    //矿仓位置
    int16_t magazine_pos;

    //矿仓完成动作
    int16_t magazine_ready;

    uint16_t crc16;
};
#pragma pack(pop)

/*===============内部变量=============*/
static uart_drv_t *s_uart = nullptr;
static databoard *s_databoard = nullptr;
static QueueHandle_t s_rx_queue = nullptr;
static TaskHandle_t s_com_task = nullptr;


static upper_to_lower_frame_t s_rx_frame;
static lower_to_upper_frame_t s_tx_frame;

static TickType_t s_last_rx_tick = 0;
static bool       s_is_online   = false;

//DataBoard topic ID
static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;
static uint32_t s_topic_magazine_pos;
static uint32_t s_topic_magazine_ready;
static uint32_t s_topic_online;
static uint32_t s_topic_send_online;
//用来发送的是否在线

/*===========接收回调============*/
static uint32_t debug_rx_count = 0;  // 加个计数变量

static bool rx_callback(uint8_t *buf,uint16_t size,BaseType_t &xHigherPriortyTaskWoken)
{
    if(size == 0) return false;
    debug_rx_count ++;
    xQueueSendFromISR(s_rx_queue,buf,&xHigherPriortyTaskWoken);
    return true;//切换缓冲区
}
/*=========接收解析=============*/
static void rx_parse_and_publish(){
    uint8_t rx_buf[sizeof(upper_to_lower_frame_t)+10];
    //从队列里面取出来一帧
    if(xQueueReceive(s_rx_queue,rx_buf,0) != pdTRUE){
        return;
    }
    auto *frame = reinterpret_cast<upper_to_lower_frame_t *>(rx_buf);
    //帧头校验
    if(frame->frame_header != FRAME_HEADER) return ;


    //第二步：CRC校验
    if(!verify_crc16_check_sum(rx_buf,sizeof(upper_to_lower_frame_t))) return ;
    s_last_rx_tick = xTaskGetTickCount();
    if(!s_is_online){
        s_is_online = true;//表示现在是在线的
        genenral_data_t data;
        data.data_si = 1;
        s_databoard->write_topic(s_topic_online, data);
    }

    memcpy(&s_rx_frame,frame,sizeof(upper_to_lower_frame_t));
    
    genenral_data_t data;
    data.data_f = s_rx_frame.vx;
    s_databoard->write_topic(s_topic_vx, data);

    data.data_f = s_rx_frame.vy;
    s_databoard->write_topic(s_topic_vy, data);

    data.data_f = s_rx_frame.wz;
    s_databoard->write_topic(s_topic_wz, data);

    data.data_si = static_cast<int32_t>(s_rx_frame.enable);//强制类型转换
    s_databoard->write_topic(s_topic_enable, data);

    data.data_si = static_cast<int32_t>(s_rx_frame.magazine_pos);
    s_databoard->write_topic(s_topic_magazine_pos, data);
}
static void check_timeout(){
    if (s_is_online &&
        (xTaskGetTickCount() - s_last_rx_tick > pdMS_TO_TICKS(TIMEOUT_MS)))
    {
        s_is_online = false;

        // 发布离线状态
        genenral_data_t data;
        data.data_si = 0;
        s_databoard->write_topic(s_topic_online, data);

        // 速度清零，防止失控
        data.data_f = 0.0f;
        s_databoard->write_topic(s_topic_vx, data);
        s_databoard->write_topic(s_topic_vy, data);
        s_databoard->write_topic(s_topic_wz, data);
    }
}

/*=============发送================*/
static void tx_fill_and_send()
{
    s_tx_frame.frame_header = FRAME_HEADER;
    //这里留着再填写

    append_crc16_check_sum(
        reinterpret_cast<uint8_t *>(&s_tx_frame),
        sizeof(lower_to_upper_frame_t));
    if(s_uart){
        s_uart->write(reinterpret_cast<uint8_t *>(&s_tx_frame),sizeof(lower_to_upper_frame_t));
        //强制转换后再发送
    }  
}

/* ============ 绑定 DataBoard topic（topic 在 init_thread 里统一创建） ============ */
static void databoard_topics_init()
{
    s_topic_vx               = s_databoard->get_topic_id("chassis_vx");
    s_topic_vy               = s_databoard->get_topic_id("chassis_vy");
    s_topic_wz               = s_databoard->get_topic_id("chassis_wz");
    s_topic_enable           = s_databoard->get_topic_id("chassis_enable");
    s_topic_online           = s_databoard->get_topic_id("chassis_online");
    s_topic_magazine_pos     = s_databoard->get_topic_id("magazine_pos");
    s_topic_magazine_ready   = s_databoard->get_topic_id("magazine_ready");
    s_topic_send_online      = s_databoard->get_topic_id("online_check");//这个是用来发送数据的
}

static void interboard_com_thread(void *argument)
{
    (void)argument;

    // 等待 databoard 初始化完成
    while (s_databoard == nullptr)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    databoard_topics_init();

    while (true)
    {
        rx_parse_and_publish();  // 接收解析 → 发布
        check_timeout();         // 超时检测
        tx_fill_and_send();      // 收集状态 → 发送
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
extern "C" void chassis_interboard_com_init(databoard *db_ptr)
{
    // 1. 保存外部传入的 databoard 指针
    s_databoard = db_ptr;

    // 2. 通过 BSP 层获取 UART7 实例
    s_uart = &bsp_uart::get_uart7();

    // 3. 创建接收队列
    s_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(upper_to_lower_frame_t));

    // 4. 注册接收回调
    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);

    // 5. 启动 DMA 接收（必须手动调用）
    s_uart->enable_rx_dma();

    // 6. 创建 1ms 通信任务
    xTaskCreate(interboard_com_thread,
                "interboard_com",
                512,
                nullptr,
                configMAX_PRIORITIES - 3,
                &s_com_task);
}