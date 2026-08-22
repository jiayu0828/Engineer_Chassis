#include "ws2812_middleware.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "pyro_dwt_drv.h"   

/* 
 *  STM32H7: 定时器时钟 = APB时钟 × 2 (当APB分频≠1时)
 *  */

static uint32_t ws2812_get_timer_clock(void)
{
    uint32_t apb_clk;
    uint32_t psc_div;

#if (WS2812_PWM_SELECT == 11) || (WS2812_PWM_SELECT == 13)
    /* TIM1 在 APB2 */
    apb_clk = HAL_RCC_GetPCLK2Freq();
    /* 读取D2PPRE2分频位 (RCC->D2CFGR bit8-10) */
    psc_div = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos;
#else
    /* TIM2/TIM3 在 APB1 */
    apb_clk = HAL_RCC_GetPCLK1Freq();
    /* 读取D2PPRE1分频位 (RCC->D2CFGR bit4-6) */
    psc_div = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
#endif

    /* 分频值: 0xx=不分频(×1), 100=×2, 101=×4, 110=×8, 111=×16
     * 如果分频≠1(即psc_div >= 4), 定时器时钟 = APB × 2
     * 如果分频=1(即psc_div < 4), 定时器时钟 = APB
     */
    if (psc_div >= 4U) {
        return apb_clk * 2U;
    } else {
        return apb_clk;
    }
}

void ws2812_hw_init(void)
{
    /* 硬件配置由CubeMX完成, 此处无需操作 */
}

void ws2812_hw_deinit(void)
{
    ws2812_hw_stop_dma();
}

void ws2812_hw_start_dma(const uint8_t *pulse_buf, uint16_t length)
{
    HAL_TIM_PWM_Start_DMA(WS2812_TIM_HANDLE,
                           WS2812_TIM_CHANNEL,
                           (uint32_t*)pulse_buf,
                           (uint32_t)length);
}

void ws2812_hw_stop_dma(void)
{
    HAL_TIM_PWM_Stop_DMA(WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);
}


uint8_t ws2812_hw_is_dma_busy(void)
{
    if (WS2812_TIM_HANDLE->hdma[WS2812_TIM_CHANNEL] != NULL) {
        if (WS2812_TIM_HANDLE->hdma[WS2812_TIM_CHANNEL]->State == HAL_DMA_STATE_BUSY) {
            return 1U;
        }
    }
    return 0U;
}

void ws2812_hw_get_tim_config(uint32_t *tim_clk, uint32_t *psc, uint32_t *arr)
{
    if (tim_clk != NULL) {
        *tim_clk = ws2812_get_timer_clock();
    }
    if (psc != NULL) {
        *psc = WS2812_TIM_HANDLE->Init.Prescaler;
    }
    if (arr != NULL) {
        *arr = WS2812_TIM_HANDLE->Init.Period;
    }
}

void ws2812_hw_delay_us(uint16_t us)
{
    pyro::dwt_drv_t::delay_us((uint32_t)us);
}

void ws2812_hw_delay_ms(uint16_t ms)
{
    pyro::dwt_drv_t::delay_us((uint32_t)ms * 1000U);
}
