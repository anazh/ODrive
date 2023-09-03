/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>
#include <adc.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ADC_CHANNEL_COUNT 16 // 每个ADC 有 16 个外部通道，每个通道都有一个对应的GPIO引脚
extern const float adc_full_scale; // extern关键字表示这些变量是在别的地方定义的；ADC（模数转换器）的全量程范围
extern const float adc_ref_voltage; // ADC（模数转换器）的参考电压
/* Exported variables --------------------------------------------------------*/
// extern关键字用于声明全局变量和函数
extern float vbus_voltage; // 母线电压
extern float ibus_; // 母线电流
extern bool brake_resistor_armed; //  这是一个布尔型变量，可能表示制动电阻器是否已经准备就绪。制动电阻器通常用于消耗发电机或电机在制动过程中的能量，以防止过电压
extern bool brake_resistor_saturated; // 这也是一个布尔型变量，可能表示制动电阻器是否已经饱和。当制动电阻器饱和时，它已经达到了它的最大耗散能力，不能再消耗更多的能量
extern float brake_resistor_current; // 制动电阻器电流
extern uint16_t adc_measurements_[ADC_CHANNEL_COUNT]; // ADC测量值
extern osThreadId analog_thread; // 用于存储指向analog_thread线程的指针
extern const uint32_t stack_size_analog_thread; // 用于存储analog_thread线程的堆栈大小
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void safety_critical_arm_brake_resistor();
void safety_critical_disarm_brake_resistor();
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on);

// called from STM platform code
extern "C" {
void vbus_sense_adc_cb(uint32_t adc_value);
void pwm_in_cb(TIM_HandleTypeDef *htim);
}

// Initalisation
void start_adc_pwm();
void start_pwm(TIM_HandleTypeDef* htim);
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
                uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                TIM_HandleTypeDef* htim_refbase = nullptr);
void start_general_purpose_adc();
void pwm_in_init();
void start_analog_thread();

// ADC getters
uint16_t channel_from_gpio(Stm32Gpio gpio);
float get_adc_voltage(Stm32Gpio gpio);
float get_adc_relative_voltage(Stm32Gpio gpio);
float get_adc_relative_voltage_ch(uint16_t channel);

void update_brake_current();

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
