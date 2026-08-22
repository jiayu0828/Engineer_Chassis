#include "ws2812_driver.h"
#include "pyro_core_def.h"
#include "pyro_core_dma_heap.h"   /* PYRo DMA堆: 脉冲缓冲区必须从DMA堆分配 */
#include "pyro_dwt_drv.h"         /* PYRo DWT驱动: 随机数种子 */
#include <string.h>
#include <stdlib.h>
#include "pyro_dwt_drv.h"   /* DWT驱动: 用于获取当前时间戳, 用于随机数种子 */
//为方便起见，统一使用DMA堆分配内存，避免DMA传输失败

namespace pyro {

ws2812_t& ws2812_t::get_instance(void)
{
    static ws2812_t instance;
    return instance;
}

ws2812_t::ws2812_t(void)
    : _led_num(0)
    , _initialized(false)
    , _busy(false)
    , _brightness(255)
    , _tim_clk(0)
    , _psc(0)
    , _arr(0)
    , _pwm_freq(0)
    , _pwm_period_us(0.0f)
    , _code0_ccr(0)
    , _code1_ccr(0)
    , _breath_brightness(0)
    , _breath_direction(true)
    , _color_buf(nullptr)
    , _pulse_buf(nullptr)
{
}

ws2812_t::~ws2812_t(void)
{
    deinit();
}

status_t ws2812_t::init(uint16_t led_num)
{
    if (led_num == 0) {
        return PYRO_PARAM_ERROR;
    }
    if (_initialized) {
        deinit();
    }

    _tim_clk = _calc_tim_clock();
    _calc_pwm_params();

    /* PWM频率校验: WS2812要求800kHz左右, 容差±25% */
    if (_pwm_freq < 600000 || _pwm_freq > 1000000) {
        return PYRO_ERROR; 
    }

    //分配内存给颜色缓冲区和脉冲缓冲区
    if (!_allocate_buffers(led_num)) {
        return PYRO_NO_MEMORY;
    }
    memset(_color_buf, 0, WS2812_COLOR_BUF_SIZE(led_num));

    _led_num = led_num;
    _initialized = true;
    _busy = false;

    /* 用DWT周期计数器作为随机数种子, 每次上电种子不同 */
    srand(pyro::dwt_drv_t::get_current_ticks());

    ws2812_hw_init();

    return PYRO_OK;
}

//更新LED数量
status_t ws2812_t::reset_led(uint16_t led_num)
{
    deinit();
    return init(led_num);
}
//重新初始化硬件
status_t ws2812_t::reset(void)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }

    ws2812_hw_deinit();
    _busy = false;

    ws2812_hw_init();

    uint32_t tim_clk, psc, arr;
    ws2812_hw_get_tim_config(&tim_clk, &psc, &arr);
    _tim_clk = tim_clk;
    _psc = psc;
    _arr = arr;

    _calc_pwm_params();


    return PYRO_OK;
}

//代码层面的反初始化
void ws2812_t::deinit(void)
{
    if (!_initialized) {
        return;
    }

    ws2812_hw_deinit();
    _busy = false;

    _free_buffers();

    _led_num = 0;
    _initialized = false;
}



status_t ws2812_t::set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (index >= _led_num) {
        return PYRO_PARAM_ERROR;
    }

    /* 颜色缓冲区是GRB顺序: G在第0字节, R在第1字节, B在第2字节
     * 输入是RGB, 这里要转换成GRB存入缓冲区
     */
    uint32_t offset = (uint32_t)index * WS2812_BYTES_PER_LED;
    _color_buf[offset + WS2812_OFFSET_G] = g;
    _color_buf[offset + WS2812_OFFSET_R] = r;
    _color_buf[offset + WS2812_OFFSET_B] = b;

    return PYRO_OK;
}

status_t ws2812_t::set_led(uint16_t index, const ws2812_color_t& color)
{
    return set_led(index, color.r, color.g, color.b);
}

status_t ws2812_t::set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    for (uint16_t i = 0; i < _led_num; i++) {
        set_led(i, r, g, b);
    }
    return PYRO_OK;
}

status_t ws2812_t::clear(void)
{
    return set_all(0,0,0);
}

status_t ws2812_t::set_range(uint16_t start, uint16_t end,
                              uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (start >= _led_num || end >= _led_num || start > end) {
        return PYRO_PARAM_ERROR;
    }

    for (uint16_t i = start; i <= end; i++) {
        set_led(i, r, g, b);
    }
    return PYRO_OK;
}

status_t ws2812_t::set_buffer(const ws2812_color_t* colors, uint16_t count)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (colors == nullptr) {
        return PYRO_PARAM_ERROR;
    }

    /* 超过LED数量的部分截断 */
    uint16_t actual = (count < _led_num) ? count : _led_num;

    for (uint16_t i = 0; i < actual; i++) {
        set_led(i, colors[i].r, colors[i].g, colors[i].b);
    }

    return PYRO_OK;
}

ws2812_color_t ws2812_t::get_led(uint16_t index) const
{
    ws2812_color_t color = WS2812_COLOR_BLACK;

    if (!_initialized || index >= _led_num) {
        return color;
    }

    /* 从GRB缓冲区读出, 转换成RGB返回 */
    uint32_t offset = (uint32_t)index * WS2812_BYTES_PER_LED;
    color.g = _color_buf[offset + WS2812_OFFSET_G];
    color.r = _color_buf[offset + WS2812_OFFSET_R];
    color.b = _color_buf[offset + WS2812_OFFSET_B];

    return color;
}

//阻塞发送
status_t ws2812_t::update(void)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (_busy) {
        return PYRO_BUSY;
    }

    /* 1. 颜色缓冲区 → 脉冲缓冲区 (应用亮度, GRB顺序, MSB先发送) */
    _convert_color_to_pulse();

    /* 2. 启动DMA传输 */
    _busy = true;
    uint32_t total_pulses = WS2812_PULSE_BUF_SIZE(_led_num) + WS2812_RESET_PULSE_COUNT;
    ws2812_hw_start_dma((const uint8_t*)_pulse_buf, (uint16_t)total_pulses);

    /* 3. 阻塞等待DMA传输完成 (超时保护) */
    uint32_t timeout = 0;
    while (_busy) {
        ws2812_hw_delay_us(10);
        if (++timeout > 2000) {  // 20ms超时 (足够传输几百颗LED)
            _busy = false;
            ws2812_hw_stop_dma();
            return PYRO_TIMEOUT;
        }
    }

    /* 4. DMA传输完成后, 脉冲缓冲区末尾的复位码已经输出了低电平(50us)
     *    额外延时确保复位可靠
     */
    ws2812_hw_delay_us(WS2812_RESET_DELAY_US);

    return PYRO_OK;
}

/* ---------- update_async: 非阻塞发送 ---------- */
status_t ws2812_t::update_async(void)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (_busy) {
        return PYRO_BUSY;
    }

    /* 1. 颜色缓冲区 → 脉冲缓冲区 */
    _convert_color_to_pulse();

    /* 2. 启动DMA传输, 立即返回 */
    _busy = true;
    uint32_t total_pulses = WS2812_PULSE_BUF_SIZE(_led_num) + WS2812_RESET_PULSE_COUNT;
    ws2812_hw_start_dma((const uint8_t*)_pulse_buf, (uint16_t)total_pulses);

    /* 3. 传输完成后会在中断里自动调用 _on_dma_complete()
     *    用 is_busy() 查询是否完成
     */
    return PYRO_OK;
}


bool ws2812_t::is_busy(void) const
{
    return _busy;
}

void ws2812_t::_on_dma_complete(void)
{
    /* 停止DMA和PWM输出 */
    ws2812_hw_stop_dma();
    _busy = false;
}


void ws2812_t::set_brightness(uint8_t brightness)
{
    _brightness = brightness;
}

uint8_t ws2812_t::get_brightness(void) const
{
    return _brightness;
}

//hsv转rgb
/*

*/
ws2812_color_t ws2812_t::hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
{
    ws2812_color_t rgb = WS2812_COLOR_BLACK;

    if (s == 0) {
        rgb.r = rgb.g = rgb.b = v;
        return rgb;
    }
    //避免越界
    float h1 = (float)(h % 360) / 60.0f;
    float s1 = (float)s / 255.0f;
    float v1 = (float)v / 255.0f;
    
    
    uint8_t i = (uint8_t)h1;
    float f = h1 - (float)i;


    

    float p = v1 * (1.0f - s1);
    float q = v1 * (1.0f - s1 * f);
    float t = v1 * (1.0f - s1 * (1.0f - f));

    float r, g, b;
    switch (i) {
        case 0: r = v1; g = t;   b = p;   break;
        case 1: r = q;   g = v1; b = p;   break;
        case 2: r = p;   g = v1; b = t;   break;
        case 3: r = p;   g = q;   b = v1; break;
        case 4: r = t;   g = p;   b = v1; break;
        default: r = v1; g = p;   b = q;   break;
    }

    rgb.r = (uint8_t)(r * 255.0f );
    rgb.g = (uint8_t)(g * 255.0f);
    rgb.b = (uint8_t)(b * 255.0f );

    return rgb;
}

status_t ws2812_t::set_led_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v)
{
    ws2812_color_t rgb = hsv_to_rgb(h, s, v);
    return set_led(index, rgb);
}

status_t ws2812_t::set_all_hsv(uint16_t h, uint8_t s, uint8_t v)
{
    ws2812_color_t rgb = hsv_to_rgb(h, s, v);
    return set_all(rgb.r, rgb.g, rgb.b);
}




status_t ws2812_t::gradient(uint16_t start, uint16_t end,
                             const ws2812_color_t& color1, const ws2812_color_t& color2)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (start >= _led_num || end >= _led_num || start > end) {
        return PYRO_PARAM_ERROR;
    }

    uint16_t range = end - start;
    if (range == 0) {
        return set_led(start, color1);
    }

    for (uint16_t i = start; i <= end; i++) {
        float ratio = (float)(i - start) / (float)range;
        ws2812_color_t c;
        c.r = (uint8_t)((float)color1.r + ((float)color2.r - (float)color1.r) * ratio);
        c.g = (uint8_t)((float)color1.g + ((float)color2.g - (float)color1.g) * ratio);
        c.b = (uint8_t)((float)color1.b + ((float)color2.b - (float)color1.b) * ratio);
        set_led(i, c);
    }

    
    return update();
}

status_t ws2812_t::rainbow(uint16_t start_hue, uint8_t density)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (density == 0) {
        return PYRO_PARAM_ERROR;
    }

    for (uint16_t i = 0; i < _led_num; i++) {
        /* 每颗LED的色相 = 起始色相 + 索引 × 密度
         * density越大, 色带越密 (相同长度内色相变化越大)
         */
        uint16_t hue = (start_hue + (uint32_t)i * density) % 360;
        ws2812_color_t rgb = hsv_to_rgb(hue, 255, 255);
        set_led(i, rgb);
    }

    
    return update();
}


status_t ws2812_t::breathing(const ws2812_color_t& color, uint8_t step)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (step == 0) {
        return PYRO_PARAM_ERROR;
    }

    /* 更新亮度: 渐亮→到顶→渐暗→到底→循环 */
    if (_breath_direction) {
        /* 渐亮 */
        if (_breath_brightness + step >= 255) {
            _breath_brightness = 255;
            _breath_direction = false;
        } else {
            _breath_brightness += step;
        }
    } else {
        /* 渐暗 */
        if (_breath_brightness <= step) {
            _breath_brightness = 0;
            _breath_direction = true;
        } else {
            _breath_brightness -= step;
        }
    }

    /* 用当前亮度缩放颜色, 设置所有LED */
    ws2812_color_t scaled;
    scaled.r = (uint8_t)((uint16_t)color.r * _breath_brightness / 255);
    scaled.g = (uint8_t)((uint16_t)color.g * _breath_brightness / 255);
    scaled.b = (uint8_t)((uint16_t)color.b * _breath_brightness / 255);
    set_all(scaled.r, scaled.g, scaled.b);

    /*  动态效果每帧立即显示, 延时由调用者在循环里控制 */
    return update();
}

/* ---------- chase: 跑马灯效果 (动态效果, position由调用者维护) ---------- */
status_t ws2812_t::chase(const ws2812_color_t& color, uint16_t position, uint8_t length)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (length == 0 || length > _led_num) {
        return PYRO_PARAM_ERROR;
    }

    /* 先全部熄灭 */
    clear();

    /* 点亮跑马灯区域 (环形, 超出部分从开头继续) */
    for (uint8_t i = 0; i < length; i++) {
        uint16_t idx = (position + i) % _led_num;
        set_led(idx, color);
    }

    /* : 动态效果每帧立即显示, position和延时由调用者控制 */
    return update();
}
//备注：这里应该是一块长度为legth的区域，随着position的增加而移动，形成跑马灯效果。
/* ---------- twinkle: 随机闪烁星空效果 (动态效果) ---------- */
status_t ws2812_t::twinkle(const ws2812_color_t& color, uint8_t probability)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (probability > 100) {
        probability = 100;
    }

    /* 先全部熄灭 */
    clear();

    /* 每颗LED按概率随机点亮 */
    for (uint16_t i = 0; i < _led_num; i++) {
        if ((rand() % 100) < probability) {
            set_led(i, color);
        }
    }

    /* 阻塞发送: 动态效果每帧立即显示, 延时由调用者在循环里控制 */
    return update();
}

/* ---------- color_wipe: 颜色填充动画 (动态效果, position由调用者维护) ---------- */
status_t ws2812_t::color_wipe(const ws2812_color_t& color, uint16_t position)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }
    if (position >= _led_num) {
        position = _led_num - 1;
    }
    /* 先全部熄灭, 然后点亮0~position的LED (单帧, position由调用者在循环里递增) */
    clear();
    for (uint16_t i = 0; i <= position; i++) {
        set_led(i, color);
    }

    /* 阻塞发送: 动态效果每帧立即显示, position和延时由调用者控制 */
    return update();
}

void ws2812_t::test_pattern(void)
{
    if (!_initialized) {
        return;
    }

    /* 红 */
    set_all(255, 0, 0);
    update();
    ws2812_hw_delay_ms(500);

    /* 绿 */
    set_all(0, 255, 0);
    update();
    ws2812_hw_delay_ms(500);

    /* 蓝 */
    set_all(0, 0, 255);
    update();
    ws2812_hw_delay_ms(500);

    /* 白 */
    set_all(255, 255, 255);
    update();
    ws2812_hw_delay_ms(500);

    /* 熄灭 */
    clear();
    update();
}

void ws2812_t::self_test(void)
{
    if (!_initialized) {
        return;
    }

    for (uint16_t i = 0; i < _led_num; i++) {
        clear();
        set_led(i, WS2812_COLOR_WHITE);
        update();
        ws2812_hw_delay_ms(100);
    }

    /* 最后全部熄灭 */
    clear();
    update();
}




uint32_t ws2812_t::_calc_tim_clock(void)
{
    uint32_t tim_clk, psc, arr;
    ws2812_hw_get_tim_config(&tim_clk, &psc, &arr);
    _tim_clk = tim_clk;
    _psc = psc;
    _arr = arr;
    return tim_clk;
}

void ws2812_t::_calc_pwm_params(void)
{
    
    _pwm_freq = _tim_clk / ((_psc + 1) * (_arr + 1));

    /* PWM周期(us) = 1,000,000 / PWM频率 */
    if (_pwm_freq > 0) {
        _pwm_period_us = 1000000.0f / (float)_pwm_freq;
    } else {
        _pwm_period_us = 0.0f;
    }

    /* 0码CCR = (0码高电平时间 / PWM周期) × (ARR+1)
     * 典型值: 0.4us / 1.25us × 300 = 96
     */
    float ccr0 = (WS2812_CODE0_HIGH_US / _pwm_period_us) * (float)(_arr + 1);
    _code0_ccr = (uint8_t)(ccr0 + 0.5f);  /* 四舍五入 */

    /* 1码CCR = (1码高电平时间 / PWM周期) × (ARR+1)
     * 典型值: 0.8us / 1.25us × 300 = 192
     */
    float ccr1 = (WS2812_CODE1_HIGH_US / _pwm_period_us) * (float)(_arr + 1);
    _code1_ccr = (uint8_t)(ccr1 + 0.5f);  /* 四舍五入 */

    /* 边界检查: CCR不能超过ARR */
    if (_code0_ccr > _arr) _code0_ccr = (uint8_t)_arr;
    if (_code1_ccr > _arr) _code1_ccr = (uint8_t)_arr;
}

void ws2812_t::_convert_color_to_pulse(void)
{
    uint32_t pulse_idx = 0;

    for (uint16_t led = 0; led < _led_num; led++) {
        uint32_t color_offset = (uint32_t)led * WS2812_BYTES_PER_LED;

        /* GRB顺序, 每颗LED 3字节, 每字节8bit, MSB先发送 */
        for (uint8_t byte = 0; byte < WS2812_BYTES_PER_LED; byte++) {
            /* 应用亮度缩放: 颜色值 × brightness / 255 */
            uint16_t scaled = (uint16_t)_color_buf[color_offset + byte] * _brightness / 255;
            uint8_t val = (uint8_t)scaled;

            /* 每bit对应一个CCR值, MSB先发送 (bit7→bit0) */
            for (int8_t bit = 7; bit >= 0; bit--) {
                if (val & (1 << bit)) {
                    _pulse_buf[pulse_idx++] = _code1_ccr;  /* 1码 */
                } else {
                    _pulse_buf[pulse_idx++] = _code0_ccr;  /* 0码 */
                }
            }
        }
    }

    /* 复位码: 末尾填若干个0 (CCR=0 → 输出低电平)
     * 40个脉冲 × 1.25us = 50us, 满足WS2812复位要求
     */
    for (uint32_t i = 0; i < WS2812_RESET_PULSE_COUNT; i++) {
        _pulse_buf[pulse_idx++] = 0;
    }
}

/* ---------- _allocate_buffers: 分配颜色缓冲区和脉冲缓冲区 ---------- */
bool ws2812_t::_allocate_buffers(uint16_t led_num)
{
    /* 颜色缓冲区: led_num × 3 字节 */
    uint32_t color_size = WS2812_COLOR_BUF_SIZE(led_num);

    /* 脉冲缓冲区: (led_num × 24 + 复位码) × sizeof(uint32_t) 字节
     * 每个元素是uint32_t, 对应一个PWM周期的CCR值
     */
    uint32_t pulse_count = WS2812_PULSE_BUF_SIZE(led_num) + WS2812_RESET_PULSE_COUNT;
    uint32_t pulse_size = pulse_count * sizeof(uint32_t);

    _color_buf = (uint8_t*)pvPortDmaMalloc(color_size);
    if (_color_buf == nullptr) {
        return false;
    }

    _pulse_buf = (uint32_t*)pvPortDmaMalloc(pulse_size);
    if (_pulse_buf == nullptr) {

        vPortDmaFree(_color_buf);
        _color_buf = nullptr;
        return false;
    }

    return true;
}

void ws2812_t::_free_buffers(void)
{
    if (_color_buf != nullptr) {
        vPortDmaFree(_color_buf);
        _color_buf = nullptr;
    }
    if (_pulse_buf != nullptr) {
        vPortDmaFree(_pulse_buf);
        _pulse_buf = nullptr;
    }
}

} // namespace pyro



extern "C" {
void ws2812_on_dma_complete(void)
{
    pyro::ws2812_t::get_instance()._on_dma_complete();
}
__attribute__((section(".itcm_text")))
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == WS2812_TIM_INSTANCE) {
        pyro::ws2812_t::get_instance()._on_dma_complete();
    }
    /* 如果以后其他驱动也用了TIM的PWM+DMA, 在这里加 else if 分支 */
}


}
