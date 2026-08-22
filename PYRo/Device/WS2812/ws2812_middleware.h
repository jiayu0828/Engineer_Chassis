#ifndef WS2812_MIDDLEWARE_H
#define WS2812_MIDDLEWARE_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 *    PWM通道选择
 *    11 = TIM1_CH1 (默认)
 *    13 = TIM1_CH3
 *    21 = TIM2_CH1  
 *    23 = TIM2_CH3
 *    32 = TIM3_CH2
 * ============================================================ */
#ifndef WS2812_PWM_SELECT
#define WS2812_PWM_SELECT  32    /* 默认 TIM1_CH1 */
#endif

#if WS2812_PWM_SELECT == 11
    /* ---------- TIM1_CH1 ---------- */
    #define WS2812_TIM_HANDLE       (&htim1)
    #define WS2812_TIM_CHANNEL      TIM_CHANNEL_1
    #define WS2812_TIM_INSTANCE     TIM1
    #define WS2812_GPIO_PORT        GPIOE
    #define WS2812_GPIO_PIN         GPIO_PIN_9
    #define WS2812_GPIO_AF_NUM      1      

#elif WS2812_PWM_SELECT == 13
    /* ---------- TIM1_CH3 ---------- */
    #define WS2812_TIM_HANDLE       (&htim1)
    #define WS2812_TIM_CHANNEL      TIM_CHANNEL_3
    #define WS2812_TIM_INSTANCE     TIM1
    #define WS2812_GPIO_PORT        GPIOE
    #define WS2812_GPIO_PIN         GPIO_PIN_13
    #define WS2812_GPIO_AF_NUM      1      

#elif WS2812_PWM_SELECT == 21
    /* ---------- TIM2_CH1 ---------- */
    #define WS2812_TIM_HANDLE       (&htim2)
    #define WS2812_TIM_CHANNEL      TIM_CHANNEL_1
    #define WS2812_TIM_INSTANCE     TIM2
    #define WS2812_GPIO_PORT        GPIOA
    #define WS2812_GPIO_PIN         GPIO_PIN_0
    #define WS2812_GPIO_AF_NUM      1      

#elif WS2812_PWM_SELECT == 23
    /* ---------- TIM2_CH3 ---------- */
    #define WS2812_TIM_HANDLE       (&htim2)
    #define WS2812_TIM_CHANNEL      TIM_CHANNEL_3
    #define WS2812_TIM_INSTANCE     TIM2
    #define WS2812_GPIO_PORT        GPIOA
    #define WS2812_GPIO_PIN         GPIO_PIN_2
    #define WS2812_GPIO_AF_NUM      1       
#elif WS2812_PWM_SELECT == 32
    /* ---------- TIM3_CH1 ---------- */
    #define WS2812_TIM_HANDLE       (&htim3)
    #define WS2812_TIM_CHANNEL      TIM_CHANNEL_2
    #define WS2812_TIM_INSTANCE     TIM3
    #define WS2812_GPIO_PORT        GPIOA
    #define WS2812_GPIO_PIN         GPIO_PIN_7
    #define WS2812_GPIO_AF_NUM      2
#else
    #error "WS2812_PWM_SELECT 值无效! 可选: 11(TIM1_CH1), 13(TIM1_CH3), 21(TIM2_CH1), 23(TIM2_CH3), 32(TIM3_CH2)"
#endif

#if defined(STM32H7)
    #if WS2812_PWM_SELECT == 11 || WS2812_PWM_SELECT == 13
        #define WS2812_GPIO_AF    GPIO_AF1_TIM1
    #elif WS2812_PWM_SELECT == 21 || WS2812_PWM_SELECT == 23
        #define WS2812_GPIO_AF    GPIO_AF1_TIM2
    #elif WS2812_PWM_SELECT == 32
        #define WS2812_GPIO_AF    GPIO_AF2_TIM3
    #endif
#endif
//初始化
extern void ws2812_hw_init(void);
//反初始化
extern void ws2812_hw_deinit(void);
//启动DMA
extern void ws2812_hw_start_dma(const uint8_t *pulse_buf, uint16_t length);
//停止DMA
extern void ws2812_hw_stop_dma(void);
//查询DMA是否正在传输
extern uint8_t ws2812_hw_is_dma_busy(void);
//获取定时器配置
extern void ws2812_hw_get_tim_config(uint32_t *tim_clk, uint32_t *psc, uint32_t *arr);
extern void ws2812_hw_delay_us(uint16_t us);
extern void ws2812_hw_delay_ms(uint16_t ms);


#ifdef __cplusplus
extern "C" {
#endif
void ws2812_on_dma_complete(void);
#ifdef __cplusplus
}
#endif

#endif // WS2812_MIDDLEWARE_H
