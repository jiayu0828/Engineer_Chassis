#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "pyro_core_def.h"
#include "ws2812_protocol.h"
#include "ws2812_middleware.h"

namespace pyro {

class ws2812_t
{
public:
    static ws2812_t& get_instance(void);

    /* 禁止拷贝 */
    ws2812_t(const ws2812_t&)            = delete;
    ws2812_t& operator=(const ws2812_t&) = delete;
   
   
    status_t init(uint16_t led_num);
    //初始化，并计算每个PWM参数，计算完成后会更新_code0_ccr和_code1_ccr
    status_t reset_led(uint16_t led_num);
    //重新初始化硬件，改变LED数量
    status_t reset(void);
    //重新初始化硬件，避免出现DMA传输失败的情况，LED数量不变
    void deinit(void);
    //反初始化，释放缓冲区，停止DMA传输
    
    //基础部分




    //数组操作
    //1.设置单个LED颜色
    status_t set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    
    status_t set_led(uint16_t index, const ws2812_color_t& color);

    //2.设置所有LED颜色
    status_t set_all(uint8_t r, uint8_t g, uint8_t b);

    status_t clear(void);

    //3.设置连续一段LED颜色
    status_t set_range(uint16_t start, uint16_t end, uint8_t r, uint8_t g, uint8_t b);

    status_t set_buffer(const ws2812_color_t* colors, uint16_t count);


    //4.读取某个LED的颜色
    [[nodiscard]] ws2812_color_t get_led(uint16_t index) const;





    //更新方式

    //阻塞更新
    status_t update(void);

    //异步更新，以及DMA传输完成后自动调用的回调函数 仿照UART的DMA传输完成回调函数设计
    status_t update_async(void);
    
    void _on_dma_complete(void);
    
    //查询DMA是否正在传输
    [[nodiscard]] bool is_busy(void) const;




    //亮度控制
    //思路，成比例控制，对于每个LED的颜色值，乘以一个比例系数，比例系数 = 亮度/255
    void set_brightness(uint8_t brightness);

    //获取当前亮度值
    [[nodiscard]] uint8_t get_brightness(void) const;



    //HSV控制
    //思路，将HSV颜色转换为RGB颜色，再设置LED
    static ws2812_color_t hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v);


    status_t set_led_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v);


    status_t set_all_hsv(uint16_t h, uint8_t s, uint8_t v);



    //效果部分
    //两种颜色线性渐变
    status_t gradient(uint16_t start, uint16_t end,const ws2812_color_t& color1, const ws2812_color_t& color2);
    //彩虹效果
    status_t rainbow(uint16_t start_hue, uint8_t density);

    //呼吸灯
    status_t breathing(const ws2812_color_t& color, uint8_t step);

    //跑马灯
    status_t chase(const ws2812_color_t& color, uint16_t position, uint8_t length);

    //随机闪烁星空效果
    status_t twinkle(const ws2812_color_t& color, uint8_t probability);

    //颜色填充动画
    status_t color_wipe(const ws2812_color_t& color, uint16_t position);

    //获得LED总数
    [[nodiscard]] uint16_t get_num_leds(void) const  { return _led_num; }
    //判断是否已经初始化
    [[nodiscard]] bool     is_initialized(void) const { return _initialized; }

    //测试模式
    void test_pattern(void);
    //自检，逐个点亮
    void self_test(void);

 

private:
    /* ---------- 私有构造/析构 (单例) ---------- */
    explicit ws2812_t(void);
    ~ws2812_t(void);

    /// 验证GPIO配置 (只读寄存器)
    status_t _verify_gpio(void);

    /// 验证定时器配置, 读取PSC/ARR (只读寄存器)
    status_t _verify_timer(void);

    /// 验证DMA配置 (只读HAL句柄)
    status_t _verify_dma(void);

    /// 从RCC寄存器推算定时器时钟频率
    uint32_t _calc_tim_clock(void);

    /// 自动计算PWM频率/周期/0码CCR/1码CCR
    void _calc_pwm_params(void);

    /// 颜色缓冲区 → 脉冲缓冲区 (应用亮度, GRB顺序)
    void _convert_color_to_pulse(void);

    /// 分配颜色缓冲区和脉冲缓冲区
    bool _allocate_buffers(uint16_t led_num);

    /// 释放缓冲区
    void _free_buffers(void);

    uint16_t  _led_num;          
    bool      _initialized;       
    volatile bool _busy;          //判断DMA的情况
    uint8_t   _brightness;       

    /* 自动读取/计算的硬件参数 */
    uint32_t _tim_clk;            
    uint32_t _psc;                
    uint32_t _arr;                
    uint32_t _pwm_freq;           
    float    _pwm_period_us;      
    uint8_t  _code0_ccr;          ///< 0码CCR 
    uint8_t  _code1_ccr;          ///< 1码CCR 

    
    uint8_t  _breath_brightness;  // 呼吸灯当前亮度
    bool     _breath_direction;    // 呼吸灯方向 (true=渐亮, false=渐暗)

    /* 缓冲区 */
    uint8_t*  _color_buf;         // 颜色缓冲区 (GRB顺序, 大小 = _led_num × 3)
    uint32_t* _pulse_buf;         // 脉冲缓冲区 (每bit一个CCR值, uint32_t, 大小 = _led_num × 24 + 复位码)
};

} // namespace pyro


extern "C" {
void ws2812_on_dma_complete(void);
}


#endif // WS2812_DRIVER_H
