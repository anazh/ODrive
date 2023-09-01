
#include <utils.hpp>
#include <board.h>


// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns true on success, and false if the input was out of range
// 根据 alpha 和 beta 计算上升沿时序（0.0 - 1.0）的函数。此计算基于幅值不变克拉克变换（magnitude invariant clarke transform）。
// alpha-beta 向量的幅值不能大于 sqrt(3)/2。如果输入超出范围，则返回 false；否则返回 true。
// 该函数的目的是根据 alpha 和 beta 值计算出 tA、tB 和 tC 的值，并进行范围检查，确保结果在有效范围内。
// 它使用了不同的计算公式，这些公式根据 alpha 和 beta 属于不同的扇区（Sextant），选择不同的计算方式。

// 上升沿时序（0.0 - 1.0）是指在电子电路或控制系统中，以时间为轴的一个时间段，在这个时间段内电信号由低电平（0）逐渐过渡到高电平（1）。
// 这个时间段通常用单位时间来表示，单位时间可以是秒、毫秒或任何其他时间单位。
// 上升沿时序（0.0 - 1.0）用来表示三个分量 tA、tB 和 tC 的值。这些值表示在一个固定的时间段内，相对于一个基准时间，电信号从低电平过渡到高电平的时间比例。
// 这些比例值范围在 0.0 到 1.0 之间，其中 0.0 表示信号在整个时间段内都处于低电平，而 1.0 表示信号在整个时间段内都处于高电平。
// 因此，上升沿时序（0.0 - 1.0）是用来描述一个信号在一个时间段内从低电平逐渐过渡到高电平的过程中，各个时间点的相对比例。
/*
幅值不变克拉克变换（Magnitude Invariant Clarke Transform）是一种数学变换，常用于三相交流电功率和电流的控制和分析。
它的主要目的是将三个相互垂直的交流分量转换为两个直流分量和一个零序分量，同时保持向量的幅值不变。
克拉克变换是一种将三相交流信号变换为两相信号的方法。克拉克变换可以通过将三个相互垂直的相量（a、b、c）映射到一个二维平面上来实现。
在克拉克变换中，通常选择相量 (alpha, beta) 表示。
幅值不变的特性意味着经过幅值不变变换后，虽然相量的方向发生了改变，但长度（幅值）保持不变。
这在控制系统中很重要，因为通过保持向量长度不变，可以确保按比例调节两个相量在控制过程中的贡献。
具体而言，幅值不变克拉克变换的计算过程如下：
从三相信号 (a, b, c) 计算出直流分量和零序分量：

直流分量 d = (2/3) * a - (1/3) * b - (1/3) * c
零序分量 q = (1/√3) * b - (1/√3) * c

计算 alpha 和 beta 值：
alpha = d
beta = (2/√3) * a + (1/√3) * b + (1/√3) * c
幅值不变克拉克变换的结果是 alpha 和 beta，它们表示了原始三相信号 (a, b, c) 转换后的等效二维信号。这些二维信号可以用于控制和分析三相交流电功率和电流，在电力系统和电机控制中具有广泛的应用。
*/
std::tuple<float, float, float, bool> SVM(float alpha, float beta) {
    float tA, tB, tC;
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
    }

    bool result_valid =
            tA >= 0.0f && tA <= 1.0f
            && tB >= 0.0f && tB <= 1.0f
            && tC >= 0.0f && tC <= 1.0f;
    return {tA, tB, tC, result_valid};
}

// based on https://math.stackexchange.com/a/1105038/81278
float fast_atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = std::abs(y);
    float abs_x = std::abs(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = std::min(abs_x, abs_y) / (std::max(abs_x, abs_y) + std::numeric_limits<float>::min());
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f)
        r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f)
        r = -r;

    return r;
}

// @brief: Returns how much time is left until the deadline is reached.
// If the deadline has already passed, the return value is 0 (except if
// the deadline is very far in the past)
uint32_t deadline_to_timeout(uint32_t deadline_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    uint32_t timeout_ms = deadline_ms - now_ms;
    return (timeout_ms & 0x80000000) ? 0 : timeout_ms;
}

// @brief: Converts a timeout to a deadline based on the current time.
uint32_t timeout_to_deadline(uint32_t timeout_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    return now_ms + timeout_ms;
}

// @brief: Returns a non-zero value if the specified system time (in ms)
// is in the future or 0 otherwise.
// If the time lies far in the past this may falsely return a non-zero value.
int is_in_the_future(uint32_t time_ms) {
    return deadline_to_timeout(time_ms);
}

// @brief: Returns number of microseconds since system startup
uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = TIM_TIME_BASE->CNT;
     } while (ms != HAL_GetTick());

    return (ms * 1000) + cycle_cnt;
}

// @brief: Busy wait delay for given amount of microseconds (us)
void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while (micros() - start < (uint32_t) us) {
        asm volatile ("nop");
    }
}

