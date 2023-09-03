/* Includes ------------------------------------------------------------------*/

#include <board.h>

#include <cmsis_os.h>
#include <cmath>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.hpp>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
constexpr float adc_full_scale = static_cast<float>(1UL << 12UL); // <<将1左移12位，实际上是将1乘以2的12次方，得到4096
constexpr float adc_ref_voltage = 3.3f; // ADC的参考电压
const uint32_t stack_size_analog_thread = 1024;  // Bytes 处理模拟信号的线程的堆栈大小，值为1024字节
/* Global variables ----------------------------------------------------------*/

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;
float ibus_ = 0.0f; // exposed for monitoring only
bool brake_resistor_armed = false;
bool brake_resistor_saturated = false;
float brake_resistor_current = 0.0f;
osThreadId analog_thread = 0;
/* Private constant data -----------------------------------------------------*/
/* CPU critical section helpers ----------------------------------------------*/

/* Safety critical functions -------------------------------------------------*/

/*
* This section contains all accesses to safety critical hardware registers.
* Specifically, these registers:
本节讨论安全关键硬件寄存器的访问，特别是与电机和制动电阻器PWM有关的访问。
*   Motor0 PWMs:
*     Timer1.MOE (master output enabled) // 主输出使能
*     Timer1.CCR1 (counter compare register 1) // 计数器比较寄存器1
*     Timer1.CCR2 (counter compare register 2) // 计数器比较寄存器2
*     Timer1.CCR3 (counter compare register 3)
*   Motor1 PWMs:
*     Timer8.MOE (master output enabled)
*     Timer8.CCR1 (counter compare register 1)
*     Timer8.CCR2 (counter compare register 2)
*     Timer8.CCR3 (counter compare register 3)
*   Brake resistor PWM:
*     Timer2.CCR3 (counter compare register 3)
*     Timer2.CCR4 (counter compare register 4)
* 
* The following assumptions are made:
*   - The hardware operates as described in the datasheet:
*     http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
*     This assumption also requires for instance that there are no radiation
*     caused hardware errors.
*   - After startup, all variables used in this section are exclusively modified
*     by the code in this section (this excludes function parameters)
*     This assumption also requires that there is no memory corruption.
*   - This code is compiled by a C standard compliant compiler.
*
* Furthermore:
*   - Between calls to safety_critical_arm_motor_pwm and
*     safety_critical_disarm_motor_pwm the motor's Ibus current is
*     set to the correct value and update_brake_resistor is called
*     at a high rate.
*/


// @brief Arms the brake resistor 安全地启动制动电阻器
// htim2 实例的 CCR3 和 CCR4 寄存器分别设置为0和 TIM_APB1_PERIOD_CLOCKS + 1。
// CCR3 寄存器控制着定时器的第3个比较输出，将其设置为0可以确保不会产生任何输出。
// 而 CCR4 寄存器则设置了定时器的周期时钟，这里将其设置为 TIM_APB1_PERIOD_CLOCKS + 1，
// 表示定时器的周期为 TIM_APB1_PERIOD_CLOCKS 加上1，这可以确保定时器在计时到指定的时间后触发中断
void safety_critical_arm_brake_resistor() {
    CRITICAL_SECTION() {
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            axes[i].motor_.I_bus_ = 0.0f;
        }
        brake_resistor_armed = true;
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    }
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
// 这段代码是用于解除制动电阻器的武装，并间接地解除所有电机PWM输出的武装。
// 在调用此函数之后，只有通过再次调用safety_critical_arm_brake_resistor()才能重新武装制动电阻器。
// 函数首先将brake_resistor_was_armed设置为brake_resistor_armed的旧值。
// 然后在临界区内，将brake_resistor_armed设置为false，表示制动电阻器已经解除武装。接着，将htim2实例的CCR3和CCR4寄存器分别设置为0和TIM_APB1_PERIOD_CLOCKS + 1。
// 这一步骤的目的是停止定时器的第三个比较输出，并重新设置定时器的周期时钟。
// 在离开临界区之后，函数会检查brake_resistor_was_armed的值。如果该值为true，说明在调用此函数之前制动电阻器是已经武装的状态，那么就需要解除所有电机的武装。
// 这是通过遍历axes数组并对每个电机调用disarm()函数来实现的。
// 这个检查的目的是防止无限递归。只有当之前制动电阻器确实处于武装状态时，才需要解除电机的武装。如果之前制动电阻器并未武装，那么就不需要做任何事情。
void safety_critical_disarm_brake_resistor() {
    bool brake_resistor_was_armed = brake_resistor_armed;

    CRITICAL_SECTION() {
        brake_resistor_armed = false;
        htim2.Instance->CCR3 = 0;
        htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    }

    // Check necessary to prevent infinite recursion
    if (brake_resistor_was_armed) {
        for (auto& axis: axes) {
            axis.motor_.disarm();
        }
    }
}

// @brief Updates the brake resistor PWM timings unless
// the brake resistor is disarmed.
// 这段代码的功能是更新制动电阻器的PWM时序，但仅在制动电阻器已武装时进行更新。
// 如果给定的时序违反了TIM的死区要求（即high_on - low_off小于TIM_APB1_DEADTIME_CLOCKS），则会解除电机的武装并报告错误。
// 函数首先检查给定的时序是否违反了TIM的死区要求。如果违反了死区要求，那么就调用`odrv.disarm_with_error(O一会在后面介绍)。
// 然后，在临界区内，函数检查brake_resistor_armed的值。
// 如果该值为true，表示制动电阻器已经武装，那么就可以安全地更新低侧和高侧的时序。为了避免竞争条件，首先将时序重置为安全状态（即都设置为0），然后分别设置低侧和高侧的时序。
// 这里的ch3是低侧的通道，ch4是高侧的通道。CCR3寄存器控制着定时器的第3个比较输出（即低侧通道的输出），而CCR4寄存器控制着定时器的第4个比较输出（即高侧通道的输出）。
// 如果brake_resistor_armed的值为false，说明制动电阻器已经解除武装，那么就不需要做任何事情。
// 需要注意的是，这个函数的安全性是通过使用临界区来保证的，确保在更新时序的过程中不会被其他线程中断。同时，通过检查死区要求，确保了电机的安全操作。
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on) {
    if (high_on - low_off < TIM_APB1_DEADTIME_CLOCKS) {
        odrv.disarm_with_error(ODrive::ERROR_BRAKE_DEADTIME_VIOLATION);
    }

    CRITICAL_SECTION() {
        if (brake_resistor_armed) {
            // Safe update of low and high side timings
            // To avoid race condition, first reset timings to safe state
            // ch3 is low side, ch4 is high side
            htim2.Instance->CCR3 = 0;
            htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
            htim2.Instance->CCR3 = low_off;
            htim2.Instance->CCR4 = high_on;
        }
    }
}

/* Function implementations --------------------------------------------------*/

/*
首先，通过遍历axes数组，将所有电机的武装状态解除，即调用disarm()函数。
然后，遍历motors数组，对每个电机进行初始化。计算出半个负载周期的值（half_load），然后根据该值设置电机的三个比较寄存器（CCR1、CCR2、CCR3）。
接下来，通过设置CCER寄存器的相应位，启用PWM输出（虽然此时仍被MOE屏蔽）。对每个通道都进行这样的操作。
然后，启用ADC及其中断，并等待一段时间以进行字段稳定。
接下来，调用start_timers()函数，该函数的具体实现未在代码中给出，可能是用来启动其他定时器的操作。
然后，开始启动制动电阻器的PWM。设置CCR3和CCR4寄存器的值为0和TIM_APB1_PERIOD_CLOCKS + 1，然后通过调用HAL_TIM_PWM_Start()函数来启动定时器通道3和通道4的PWM输出。
最后，如果配置中启用了制动电阻器，就调用safety_critical_arm_brake_resistor()函数来武装制动电阻器。
*/
void start_adc_pwm() {
    // Disarm motors
    for (auto& axis: axes) {
        axis.motor_.disarm();
    }

    for (Motor& motor: motors) {
        // Init PWM
        int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
        motor.timer_->Instance->CCR1 = half_load;
        motor.timer_->Instance->CCR2 = half_load;
        motor.timer_->Instance->CCR3 = half_load;

        // Enable PWM outputs (they are still masked by MOE though)
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_3);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_3);
    }

    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);


    start_timers();


    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    if (odrv.config_.enable_brake_resistor) {
        safety_critical_arm_brake_resistor();
    }
}

// @brief ADC1 measurements are written to this buffer by DMA
/*
这段代码定义了一个名为adc_measurements_的数组，数组的长度为ADC_CHANNEL_COUNT。
数组的每个元素是一个16位无符号整数（uint16_t），用于存储ADC的测量值。初始值全部为0。
*/
uint16_t adc_measurements_[ADC_CHANNEL_COUNT] = { 0 };

// @brief Starts the general purpose ADC on the ADC1 peripheral.
// The measured ADC voltages can be read with get_adc_voltage().
//
// ADC1 is set up to continuously sample all channels 0 to 15 in a
// round-robin fashion.
// DMA is used to copy the measured 12-bit values to adc_measurements_.
//
// The injected (high priority) channel of ADC1 is used to sample vbus_voltage.
// This conversion is triggered by TIM1 at the frequency of the motor control loop.
/**
这段代码是用于启动ADC1的通用功能，ADC1是一个模拟-数字转换器。该代码用于配置ADC1并开始使用DMA（直接内存访问）将ADC的测量值复制到adc_measurements_数组中。
以下是代码的详细解释：
ADC_ChannelConfTypeDef sConfig;：定义一个结构体，用于配置ADC通道。
hadc1.Instance = ADC1;：设置hadc1实例为ADC1。
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;：设置ADC的时钟预分频因子为PCLK的1/4。
hadc1.Init.Resolution = ADC_RESOLUTION_12B;：设置ADC的分辨率为12位。
hadc1.Init.ScanConvMode = ENABLE;：启用连续扫描模式，这意味着ADC将连续地转换所有选定的通道。
hadc1.Init.ContinuousConvMode = ENABLE;：启用连续转换模式，这意味着ADC将在一次启动后持续进行转换，而不是仅转换一次。
hadc1.Init.DiscontinuousConvMode = DISABLE;：禁用不连续转换模式。
hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;：禁用外部触发转换边沿。
hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;：将外部触发转换为软件启动。
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;：设置数据对齐方式为右对齐。
hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;：设置要转换的通道数量。
hadc1.Init.DMAContinuousRequests = ENABLE;：启用DMA连续请求。
hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;：在EOC（结束选择）事件发生时选择单个转换。
if (HAL_ADC_Init(&hadc1) != HAL_OK) {...}：使用HAL库初始化ADC。如果初始化失败，则将misconfigured_标志设置为true，并返回。
sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;：设置采样时间为15个周期。
接下来的循环用于配置每个通道。对于每个通道，它将通道编号左移到ADC_CR1_AWDCH位，并设置等级（rank）为当前通道的等级加一（因为等级从1开始）。
HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK {...}：使用HAL库配置ADC通道。如果配置失败，则将misconfigured_标志设置为true，并返回。
HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);：使用HAL库启动ADC的DMA传输。这会将ADC的测量值传输到adc_measurements_数组中。
总的来说，这段代码配置了ADC1以进行连续扫描，启用了DMA传输，并将所有通道的测量值复制到了adc_measurements_数组中。
 */
void start_general_purpose_adc() {
    ADC_ChannelConfTypeDef sConfig;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
        return;
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
            return;
        }
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}

// @brief Returns the ADC voltage associated with the specified pin.
// This only works if the GPIO was not used for anything else since bootup, otherwise
// it must be put to analog mode first.
// Returns -1.0f if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
/*
这是一段C语言的代码，它与STM32微控制器的ADC（模拟-数字转换器）和GPIO（通用输入/输出）相关。
代码中定义了两个函数：`get_adc_voltage`和`get_adc_relative_voltage`。
1. `get_adc_voltage(Stm32Gpio gpio)`: 这个函数接收一个`Stm32Gpio`类型的参数，然后返回一个浮点数。
这个函数通过调用`get_adc_relative_voltage`函数来获取相对电压值，然后将这个相对电压值乘以一个参考电压值`adc_ref_voltage`。参考电压值可能是在其他地方定义的一个常量。
2. `get_adc_relative_voltage(Stm32Gpio gpio)`: 这个函数也是接收一个`Stm32Gpio`类型的参数，然后返回一个浮点数。
这个函数调用`channel_from_gpio`函数将GPIO转换为ADC通道编号，然后通过调用`get_adc_relative_voltage_ch`函数来获取该通道的相对电压值。
这段代码似乎是用于从STM32微控制器的特定GPIO引脚获取ADC电压读数的。需要注意的是，如果GPIO在启动后被用于其他功能，那么在调用此函数之前，GPIO必须先被设置为模拟模式。
如果引脚没有关联的ADC1通道，这个函数将返回-1.0f。
在ODrive 3.3和3.4上，可以使用GPIO_1、GPIO_2、GPIO_3、GPIO_4以及一些连接到板载传感器的引脚（M0_TEMP、M1_TEMP、AUX_TEMP）来使用此函数。
此外，这段代码提到ADC的更新率大约为32kHz。
这意味着每个ADC转换需要15+26个ADC时钟周期，并且整个序列是连续进行的。但是实际的频率可能会稍微低一些，因为还存在注入的vbus测量。
*/
float get_adc_voltage(Stm32Gpio gpio) {
    return get_adc_relative_voltage(gpio) * adc_ref_voltage;
}

float get_adc_relative_voltage(Stm32Gpio gpio) {
    const uint16_t channel = channel_from_gpio(gpio);
    return get_adc_relative_voltage_ch(channel);
}

// @brief Given a GPIO_port and pin return the associated adc_channel.
// returns UINT16_MAX if there is no adc_channel;
/*
这段代码是用于从STM32的GPIO引脚获取相应的ADC通道。
函数 channel_from_gpio 接受一个 Stm32Gpio 类型的参数，然后返回一个 uint16_t 类型的值。这个函数的目的是找出与给定的GPIO端口和引脚相对应的ADC通道。
在函数内部，首先将 channel 初始化为 UINT32_MAX，这是无ADC通道的返回值。然后通过一系列的if条件语句，检查每个可能的GPIO端口和引脚组合。
如果GPIO的端口是 GPIOA，并且引脚是0到7，那么对应的ADC通道就会是0到7。
如果GPIO的端口是 GPIOB，并且引脚是0到1，那么对应的ADC通道就会是8到9。
如果GPIO的端口是 GPIOC，并且引脚是0到5，那么对应的ADC通道就会是10到15。
如果GPIO的端口和引脚不匹配上述任何一种情况，那么函数将返回 UINT32_MAX，表示没有对应的ADC通道。
这个函数可以根据需要添加更多的GPIO端口和引脚组合，只要确保每个组合都是唯一的，并且与ADC通道编号相对应。
*/
uint16_t channel_from_gpio(Stm32Gpio gpio) {
    uint32_t channel = UINT32_MAX;
    if (gpio.port_ == GPIOA) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 0;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 1;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 2;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 3;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 4;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 5;
        else if (gpio.pin_mask_ == GPIO_PIN_6)
            channel = 6;
        else if (gpio.pin_mask_ == GPIO_PIN_7)
            channel = 7;
    } else if (gpio.port_ == GPIOB) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 8;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 9;
    } else if (gpio.port_ == GPIOC) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 10;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 11;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 12;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 13;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 14;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 15;
    }
    return channel;
}

// @brief Given an adc channel return the voltage as a ratio of adc_ref_voltage
// returns -1.0f if the channel is not valid.
/*
这段代码是一个函数，名为get_adc_relative_voltage_ch，它接收一个uint16_t类型的参数channel，并返回一个float类型的值。
函数的目的是根据给定的ADC通道返回相对电压值，相对电压值被定义为ADC测量值与ADC满量程的比值。如果通道无效，则返回-1.0f。
在函数内部，首先检查通道是否小于ADC_CHANNEL_COUNT（这是一个预定义的常量，表示ADC通道的总数）。
如果通道有效，则将该通道的ADC测量值除以adc_full_scale（这可能是一个表示ADC满量程的常量）并返回结果。
如果通道无效（即通道值大于ADC_CHANNEL_COUNT），则直接返回-1.0f。
*/
float get_adc_relative_voltage_ch(uint16_t channel) {
    if (channel < ADC_CHANNEL_COUNT)
        return (float)adc_measurements_[channel] / adc_full_scale;
    else
        return -1.0f;
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

/*
这段代码是一个中断服务例程（Interrupt Service Routine，ISR）回调函数，名为vbus_sense_adc_cb。
这个函数是用来处理VBUS传感器的ADC（模拟-数字转换器）读数的。
函数接受一个32位无符号整数adc_value作为参数，这个值是VBUS传感器的ADC读数。
在函数内部，首先定义了一个名为voltage_scale的常量，其值等于adc_ref_voltage乘以VBUS_S_DIVIDER_RATIO再除以adc_full_scale。这个值用于将ADC读数转换为实际的电压值。
然后，将计算得到的电压值存储到全局变量vbus_voltage中。
总的来说，这个函数的作用是将VBUS传感器的ADC读数转换为电压值，并存储到全局变量中，供其他部分的代码使用。
*/
void vbus_sense_adc_cb(uint32_t adc_value) {
    constexpr float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    vbus_voltage = adc_value * voltage_scale;
}

// @brief Sums up the Ibus contribution of each motor and updates the
// brake resistor PWM accordingly.
/*
这段代码是用于更新制动电阻的PWM占空比，并计算出总Ibus电流。

首先，代码初始化了一个浮点数变量Ibus_sum为0。然后，通过遍历所有的轴（axes[i]），如果该轴的电机已启动（axes[i].motor_.is_armed_ == true），则将该电机的I_bus值加到Ibus_sum上。
接着，代码检查是否启用了制动电阻（odrv.config_.enable_brake_resistor）。如果启用了，那么需要确保制动电阻的阻值大于0，否则会报错并退出函数。
然后，代码根据Ibus总和、最大再生电流和制动电阻的阻值来计算制动占空比（brake_duty）和制动电流（brake_current）。
其中，制动电流的计算公式为：制动电流 = -Ibus总和 - 最大再生电流。
接下来，代码根据启用了enable_dc_bus_overvoltage_ramp以及制动电阻的阻值范围，进一步调整制动占空比。
然后，代码检查计算得到的制动占空比是否为NaN（不是一个数字）。如果是，那么会报错并退出函数，同时关闭所有电机和制动电阻。
接着，代码检查制动占空比是否大于等于95%。如果是，那么将brake_resistor_saturated设置为true，表示制动电阻已经饱和。
然后，代码将制动占空比限制在0到95%之间。
最后，如果未启用制动电阻，那么将制动占空比设置为0。
将计算得到的制动电流赋值给brake_resistor_curr。
然后，通过将所有电机的Ibus值相加，并乘以一个滤波器系数odrv.ibus_report_filter_k_，得到新的Ibus总和ibus_。
接着，代码检查新的Ibus总和是否超过了允许的最大正向电流和最大反向电流。如果超过了，就会报错并退出函数。
然后，根据制动占空比计算出高电平持续时间和低电平持续时间。其中，高电平持续时间是指在一个周期内，制动电阻导通的时长；低电平持续时间是指在一个周期内，制动电阻关闭的时长。
最后，调用safety_critical_apply_brake_resistor_timings函数，将计算得到的低电平持续时间和高电平持续时间应用到制动电阻的TIM定时器上。
// 占空比越大，高电平或低电平的持续时间就越长，输出电压或电流的平均值就越大。
*/
void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (axes[i].motor_.is_armed_) {
            Ibus_sum += axes[i].motor_.I_bus_;
        }
    }

    float brake_duty = 0.0f;
    float brake_current = 0.0f;
    if (odrv.config_.enable_brake_resistor) {
        if (!(odrv.config_.brake_resistance > 0.0f)) {
            odrv.disarm_with_error(ODrive::ERROR_INVALID_BRAKE_RESISTANCE);
            return;
        }
    
        // Don't start braking until -Ibus > regen_current_allowed
        brake_current = -Ibus_sum - odrv.config_.max_regen_current;
        brake_duty = brake_current * odrv.config_.brake_resistance / vbus_voltage;
        
        if (odrv.config_.enable_dc_bus_overvoltage_ramp && (odrv.config_.brake_resistance > 0.0f) && (odrv.config_.dc_bus_overvoltage_ramp_start < odrv.config_.dc_bus_overvoltage_ramp_end)) {
            brake_duty += std::max((vbus_voltage - odrv.config_.dc_bus_overvoltage_ramp_start) / (odrv.config_.dc_bus_overvoltage_ramp_end - odrv.config_.dc_bus_overvoltage_ramp_start), 0.0f);
        }

        if (is_nan(brake_duty)) {
            // Shuts off all motors AND brake resistor, sets error code on all motors.
            odrv.disarm_with_error(ODrive::ERROR_BRAKE_DUTY_CYCLE_NAN);
            return;
        }

        if (brake_duty >= 0.95f) {
            brake_resistor_saturated = true;
        }

        // Duty limit at 95% to allow bootstrap caps to charge
        brake_duty = std::clamp(brake_duty, 0.0f, 0.95f);

        // This cannot result in NaN (safe for race conditions) because we check
        // brake_resistance != 0 further up.
        brake_current = brake_duty * vbus_voltage / odrv.config_.brake_resistance;
        Ibus_sum += brake_duty * vbus_voltage / odrv.config_.brake_resistance;
    } else {
        brake_duty = 0;
    }

    brake_resistor_current = brake_current;
    ibus_ += odrv.ibus_report_filter_k_ * (Ibus_sum - ibus_);

    if (Ibus_sum > odrv.config_.dc_max_positive_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_CURRENT);
        return;
    }
    if (Ibus_sum < odrv.config_.dc_max_negative_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_REGEN_CURRENT);
        return;
    }
    
    int high_on = (int)(TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty));
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;
    safety_critical_apply_brake_resistor_timings(low_off, high_on);
}


/* Analog speed control input */
/*
这段代码是在使用C语言编写，它涉及到了一些嵌入式系统编程的概念，比如GPIO（General Purpose Input/Output，通用输入输出）和ADC（Analog to Digital Converter，模拟到数字转换器）。
代码中还使用了一个名为fibre的库，这个库可能是一个提供线程间通信或并行处理的库。
让我们逐个分析这些函数：
update_analog_endpoint：这个函数接收一个指向PWMMapping_t结构体的指针和一个GPIO引脚编号。
    首先，它通过调用get_adc_voltage函数获取GPIO引脚的ADC读数，并将其转换为电压值。
    然后，它计算该电压值在映射范围内的比例。这个比例是通过将ADC读数与最小和最大值之间的差值相除来得到的。最后，该比例被用来更新指定的端点值。
analog_polling_thread：这个函数是一个无限循环，它会遍历所有的GPIO引脚，对于每一个引脚，它会检查对应的端点是否有效，如果有效，就调用update_analog_endpoint函数更新端点值。
    然后，它会在10毫秒的延迟后再次进行这个过程。
start_analog_thread：这个函数定义了一个名为"analog_thread_def"的线程，这个线程会运行analog_polling_thread函数。然后，它创建了这个线程，并返回线程的指针。
总的来说，这段代码实现了一个定时轮询所有GPIO引脚的程序。对于每个GPIO引脚，它都会读取其ADC值，并根据映射规则更新对应的端点值。这个过程会每10毫秒进行一次。
*/
static void update_analog_endpoint(const struct PWMMapping_t *map, int gpio)
{
    float fraction = get_adc_voltage(get_gpio(gpio)) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    fibre::set_endpoint_from_float(map->endpoint, value);
}

static void analog_polling_thread(void *)
{
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &odrv.config_.analog_mappings[i];

            if (fibre::is_endpoint_ref_valid(map->endpoint))
                update_analog_endpoint(map, i);
        }
        osDelay(10);
    }
}

void start_analog_thread() {
    osThreadDef(analog_thread_def, analog_polling_thread, osPriorityLow, 0, stack_size_analog_thread / sizeof(StackType_t));
    analog_thread = osThreadCreate(osThread(analog_thread_def), NULL);
}
