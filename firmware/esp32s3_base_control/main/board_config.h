#pragma once

// ============================================================================
//  底盘硬件配置 —— 换电机/换驱动板时**只改这个文件**
//
//  两个开关：
//    DRIVER_WSDC2412D   选驱动板型号（决定 drive() 怎么输出）
//    WHEEL_CIRC_MM / COUNTS_PER_REV   选机械参数（决定里程换算）
//
//  旧硬件（AT8236 + 66mm 轮 + 1468 计数）这样编译：
//      idf.py -DDRIVER_WSDC2412D=0 build
//    然后在下面把 WHEEL_CIRC_MM / COUNTS_PER_REV 改成 210.0f / 1468.0f
//  （或者直接用 git 切回 tag `fw-at8236-baseline`）
// ============================================================================

// ---------------------------------------------------------------------------
// 1. 驱动板型号
//    0 = 旧 AT8236 双 PWM：每个电机两路 PWM（正转一路、反转一路）
//    1 = 新 WSDC2412D-V3.0：每电机 1 路 PWM + 2 路方向（INA/INB）
// ---------------------------------------------------------------------------
#ifndef DRIVER_WSDC2412D
#define DRIVER_WSDC2412D 1
#endif

// ---------------------------------------------------------------------------
// 2. 引脚分配（两种驱动板共用 GPIO4~7 的位置，新板多占 12/13）
// ---------------------------------------------------------------------------
#if DRIVER_WSDC2412D
// 通道 0 = 左轮：P1 → GPIO4，A1 → GPIO5，B1 → GPIO12
// 通道 1 = 右轮：P2 → GPIO6，A2 → GPIO7，B2 → GPIO13
#define PWM_PIN_0 GPIO_NUM_4
#define PWM_PIN_1 GPIO_NUM_6
#define INA_PIN_0 GPIO_NUM_5
#define INA_PIN_1 GPIO_NUM_7
#define INB_PIN_0 GPIO_NUM_12
#define INB_PIN_1 GPIO_NUM_13
#else
// 旧板：GPIO4/5 = 左 正转/反转，GPIO6/7 = 右 反转/正转
#define PWM_PIN_0 GPIO_NUM_4
#define PWM_PIN_1 GPIO_NUM_5
#define PWM_PIN_2 GPIO_NUM_6
#define PWM_PIN_3 GPIO_NUM_7
#endif

// ---------------------------------------------------------------------------
// 3. PWM 频率
//    新板手册要求 18~60 kHz（低于 10 kHz 有明显电流噪声）
//    旧板原来用 10 kHz
// ---------------------------------------------------------------------------
#if DRIVER_WSDC2412D
#define PWM_FREQ_HZ 18000
#else
#define PWM_FREQ_HZ 10000
#endif

// ---------------------------------------------------------------------------
// 4. 机械参数 → 里程换算
//    每 count 距离 = 轮周长(mm) / 每圈计数
//    轮周长 = π × 轮径
// ---------------------------------------------------------------------------
#ifndef WHEEL_CIRC_MM
#if DRIVER_WSDC2412D
#define WHEEL_CIRC_MM 314.159f   // 新：100 mm 轮
#else
#define WHEEL_CIRC_MM 210.0f     // 旧：66 mm 轮
#endif
#endif

#ifndef COUNTS_PER_REV
#if DRIVER_WSDC2412D
#define COUNTS_PER_REV 6400.0f   // 新：16PPR × 4(AB双沿) × 100:1
#else
#define COUNTS_PER_REV 1468.0f   // 旧
#endif
#endif

#define WHEEL_MM_PER_COUNT (WHEEL_CIRC_MM / COUNTS_PER_REV)

// ---------------------------------------------------------------------------
// 5. 编码器引脚（两种板相同：左 A/B = 8/9，右 A/B = 10/11）
//    ⚠️ 编码器 5V 供电，必须经电平转换后再进 GPIO
// ---------------------------------------------------------------------------
#define ENC_EA_0 GPIO_NUM_8
#define ENC_EB_0 GPIO_NUM_9
#define ENC_EA_1 GPIO_NUM_10
#define ENC_EB_1 GPIO_NUM_11
