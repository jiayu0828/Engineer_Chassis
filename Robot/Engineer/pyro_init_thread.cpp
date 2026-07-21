#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include "pyro_databoard.h"
#include "FreeRTOS.h"
#include "task.h"


namespace pyro{
    
    /*==================全局驱动指针========================*/
    //CAN 驱动指针
    can_drv_t *can1_drv = nullptr;
    can_drv_t *can2_drv = nullptr;
    can_drv_t *can3_drv = nullptr;


    databoard *global_databoard = nullptr;
    //先用我的数据板
    extern "C"{
        void pyro_init_thread(void *argument){
        dwt_drv_t::init(480);//480MHz主频
        /*      UART          */
        bsp_uart::get_uart1().enable_rx_dma();
        bsp_uart::get_uart5().enable_rx_dma();
        bsp_uart::get_uart7().enable_rx_dma();
        bsp_uart::get_uart10().enable_rx_dma();
        /*       CAN            */
        //CAN1    M3508*4
        can1_drv = &bsp_can::get_can1();
        can1_drv->init();
        can1_drv->start();


        //CAN2  后摇臂DM电机 + 功率计
        can2_drv = &bsp_can::get_can2();
        can2_drv->init();
        can2_drv->start();


        //CAN3   矿仓
        can3_drv = &bsp_can::get_can3();
        can3_drv->init();
        can3_drv->start();

        global_databoard = new databoard();
        global_databoard->create_topic("chassis_vx",        FLOAT);
        global_databoard->create_topic("chassis_vy",        FLOAT);
        global_databoard->create_topic("chassis_wz",        FLOAT);
        global_databoard->create_topic("chassis_enable",    UNSIGNED_INT);
        global_databoard->create_topic("chassis_online",    UNSIGNED_INT);
        global_databoard->create_topic("magazine_pos",      UNSIGNED_INT);
        global_databoard->create_topic("magazine_ready",    UNSIGNED_INT);
        global_databoard->create_topic("online_check",      UNSIGNED_INT);
        global_databoard->create_topic("lift_calib_trigger", UNSIGNED_INT);//这个是用来触发校准的
        global_databoard->create_topic("lift_calib_state",  UNSIGNED_INT);//这个是用来写校准状态
        global_databoard->create_topic("lift_zero_valid",   UNSIGNED_INT);//这个是用来写零点是否有效
        global_databoard->create_topic("lift_control_mod",  UNSIGNED_INT);//这个是用来控制我的摇臂的模式的，自动/手动
        global_databoard->create_topic("lift_mannual",      UNSIGNED_INT);//这个是手动控制我的摇臂
        global_databoard->create_topic("lift_auto",         UNSIGNED_INT);//这个是自动控制我的摇臂
        vTaskDelete(nullptr);
        //干掉自己
    }
}
};