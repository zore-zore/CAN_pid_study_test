/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "bsp_rc.h"
/* 转速环 PID（官方文件，未改动） */
#include "pid.h"
/* sinf()，正弦波流用 */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 摇杆减掉 1024 后的行程，约 ±660 */
#define RC_CH_VALUE_RANGE       660
/* 回中零点漂移，此范围内一律当 0 */
#define RC_DEAD_ZONE            20
/* 开环电流上限（仅 USE_SPEED_PID=0 时使用）。
   16384 对应 C620 满量程 20A，1000 约 1.2A。 */
#define CHASSIS_CURRENT_MAX     1000
/* 底盘四个轮子：下标 0~3 对应 C620 ID1~4 / 反馈帧 0x201~0x204。
   M1=左上FL，M2=左下RL，M3=右上FR，M4=右下RR（麦轮全向底盘）。 */
#define MOTOR_NUM               4
/* 驱动用的三个遥控通道（按本机实测的 DBUS 通道号）：
   左摇杆上下 = 前进/后退（推上为正）
   左摇杆左右 = 左移/右移（推左为正）
   右摇杆左右 = 原地旋转（推右为顺时针）
   注：本机实测 ch2 是左右、ch3 是上下，与常见的 ch2=上下 相反。 */
#define RC_MOTOR_CH_VERT        3
#define RC_MOTOR_CH_HORZ        2
#define RC_MOTOR_CH_ROT         0
/* 板载指示灯 PH11：上电后亮 RUN_LED_ON_MS，然后灭（程序已启动的标志）。
   gpio.c 已配成推挽输出。 */
#define RUN_LED_PORT            GPIOH
#define RUN_LED_PIN             GPIO_PIN_11
#define RUN_LED_ON_MS           8000
/* 遥控静默超时。DBUS 约 14ms 一帧，100ms 约漏 7 帧 */
#define RC_SIGNAL_TIMEOUT_MS    100
/* 单路电机反馈超时。C620 反馈约 1kHz，100ms 很宽松。
   与遥控超时分开：两个独立故障域，调整一个不应影响另一个。 */
#define MOTOR_FEEDBACK_TIMEOUT_MS 100
/* UART6 调试输出（PG14=TX, PG9=RX, 115200 8N1）。发送字符：
     '1' 正弦波流        '2' 停
     '3' 左摇杆流        '4' 停
     '5' 转速环曲线流，8 列：M1~M4 目标转速、M1~M4 实测转速（rpm）
   ASCII 十进制、逗号分列（SerialPlot 格式），同一时刻只跑一条流。 */
#define UART6_STREAM_OFF        0
#define UART6_STREAM_SINE       1
#define UART6_STREAM_STICK      2
#define UART6_STREAM_PID        3

/* --- 正弦波流 ('1') --- */
#define SINE_PERIOD_MS          400      /* 一个周期 400ms = 2.5Hz */
#define SINE_AMPLITUDE          2047     /* 相对中点的最大偏移 */
#define SINE_MID                2048     /* 中点（0~4095 的中心） */
#define SINE_PI                 3.14159265f

/* --- 摇杆流 ('3') --- */
/* 输出左摇杆两个轴，通道号与上面的驱动映射保持一致
   （本机：3=上下，2=左右；右摇杆是 1/0） */
#define RC_STICK_CH_VERT        3
#define RC_STICK_CH_HORZ        2
/* 满行程对应的千分比读数。上下用满幅，左右用半幅让两条曲线视觉等高。
   这是连续量，不能用 uart6_send_int()（会被压成 -1/0/+1）。 */
#define RC_STICK_VERT_MILLI     1000
#define RC_STICK_HORZ_MILLI     500

/* --- 转速环 PID ---
   摇杆 -> 三自由度 -> 麦轮逆运动学 -> 每轮目标转速 -> 四路 PID -> 电流。
   控制环跑在 SysTick 的 1kHz 中断里（motor_control_1khz）。 */

/* 回退开关：置 0 退回开环（摇杆直接给电流），仍走同一套麦轮运动学 */
#define USE_SPEED_PID           1

/* 摇杆推到底的目标转速(rpm)。C620 反馈的是转子转速，3508 空载约 9000，
   取 3000 留余量。首次上电建议先改成 1000。 */
#define SPEED_TARGET_MAX_RPM    3000

/* --- 麦轮全向运动 ---
   前后、左右、斜向都由逆运动学合成到四轮目标转速。 */
/* 置 0 关闭旋转，只剩平移 */
#define CHASSIS_ROTATE_ENABLE   1
/* 旋转满杆的差速幅值(rpm)，取线速度的一半 */
#define SPEED_ROTATE_MAX_RPM    (SPEED_TARGET_MAX_RPM / 2)
/* 滚子朝向。标准 X 形填 1；实测横移/旋转反向就把对应项改成 -1。
   前后方向不受这两个宏影响，滚子装法只能实测确认。 */
#define MECANUM_STRAFE_SIGN     1
#define MECANUM_ROTATE_SIGN     1

/* PID 参数，参考学长的整定值（Kp=3.5 / Ki=0.01 / Kd=0）。
   注意 pid.c 的积分是 Iout += Ki*error，没有 dt，Ki 与 1kHz 调用周期绑定，
   换到别的调用频率这组数要重新算。 */
#define SPEED_PID_KP            3.5f
#define SPEED_PID_KI            0.01f
#define SPEED_PID_KD            0.0f

/* C620 电调保护：最终输出电流硬限幅 16A。
   C620 官方只标「最大持续电流 20A」，没有单独的峰值指标，16A 留 20% 余量。
   CAN 电流指令 ±16384 线性对应 ±20A，16A = 16384 × 16 / 20 = 13107。
   这一层保护的是电调：不管 PID 或开环算出多少，越不过去。 */
#define C620_CURRENT_MAX_16A    13107

/* M3508 电机保护：PID 输出上限，真正决定电机电流的是这里。
   M3508 P19 额定电流 10A（16384 = 20A → 10A = 8192），额定扭矩 3N·m。
   室内调车取 3000 ≈ 3.7A，约额定的 37%，四轮合计约 14.6A；
   推十公斤级的麦轮车够用，电机和电调都不会烫。
   要提速按 3000 → 6000(≈7.3A) → 8192(10A，额定上限) 往上加。 */
#define SPEED_PID_MAX_OUT_CNT   3000
#define SPEED_PID_MAX_OUT       ((fp32)SPEED_PID_MAX_OUT_CNT)

/* 积分项上限与输出上限同值，积分不能单独堆到超过输出。
   真正的抗饱和仍在 motor_control_1khz()：目标为 0 时 PID_clear()。 */
#define SPEED_PID_MAX_IOUT      ((fp32)SPEED_PID_MAX_OUT_CNT)

/* 编译期兜底：输出上限一旦超过 M3508 额定 10A 直接编译报错，
   防止以后改参数时手滑把电机烧了。 */
#if (SPEED_PID_MAX_OUT_CNT > 8192)
#error "SPEED_PID_MAX_OUT exceeds M3508 rated current (10A = 8192 counts)"
#endif

/* 每轮方向符号见 motor_dir_sign[]。闭环里必须把「反馈」和「目标」一起翻，
   只翻输出会把负反馈变成正反馈，电机发散。 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 遥控链路看门狗。rc_frame_count 由 DBUS 中断自增，其余在 1kHz 中断里更新 */
static uint32_t rc_last_frame_count = 0;
static uint32_t rc_last_frame_tick  = 0;
static volatile uint8_t rc_online   = 0;

/* 转速环实例，一轮一个。麦轮运动时四轮目标转速本来就不同，共享单环做不到 */
static pid_type_def speed_pid[MOTOR_NUM];

/* 每轮方向符号，下标对应 M1~M4。1 = 正电流使该轮朝车头转，-1 = 相反。
   {1,1,-1,-1} 只是左右镜像安装的常见起点，必须实测核对。 */
static const int8_t motor_dir_sign[MOTOR_NUM] = {1, 1, -1, -1};

/* 控制环总开关。初始化完成前为 0；bus-off 恢复期间也会临时清 0 */
volatile uint8_t control_enabled = 0;

/* 观测变量，供 '5' 流和 Watch 使用。均单次 16 位读写，对中断原子。
   target/actual 以「朝车头方向滚」为正；current 乘回了方向符号，坐标系不同 */
volatile int16_t motor_target_rpm[MOTOR_NUM];
volatile int16_t motor_actual_rpm[MOTOR_NUM];
volatile int16_t motor_current_cmd[MOTOR_NUM];

/* 上电时间戳，用于 PH11 点亮窗口 */
static uint32_t boot_tick = 0;

/* 当前 UART6 流，取值 UART6_STREAM_*，只在主循环里改 */
static uint8_t uart6_stream = UART6_STREAM_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  把 1~8 个整数作为一行、逗号分隔发出
  * @param  values: 每个夹到 ±99999
  * @param  count: 1~8
  * @note   手写格式化，不用 printf（USART3 已被 DBUS 占用）。
  *         从 '\n' 往前倒拼；最坏 8×"-99999"+7×','+'\n' = 56 字符，缓冲 64 字节。
  */
static void uart6_send_int(const int32_t *values, uint8_t count)
{
  char     text[64];
  char    *p;
  uint32_t magnitude;
  int32_t  value;
  uint8_t  i;

  /* 缓冲区按 8 列设计，多传也按 8 列发，不能写穿栈 */
  if (count > 8u)
  {
    count = 8u;
  }

  p = &text[sizeof(text) - 1];
  *p = '\n';

  for (i = count; i > 0u; i--)
  {
    value = values[i - 1u];

    if (value < -99999)
    {
      value = -99999;
    }
    else if (value > 99999)
    {
      value = 99999;
    }

    /* 取绝对值走 64 位，避免 INT32_MIN 溢出 */
    magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

    /* 数字是倒着出来的，从缓冲区尾部往前写 */
    do
    {
      *--p = (char)('0' + (magnitude % 10u));
      magnitude /= 10u;
    }
    while (magnitude > 0u);

    if (value < 0)
    {
      *--p = '-';
    }

    if (i > 1u)
    {
      *--p = ',';
    }
  }

  /* (尾指针 - 拼写指针) 即行长，不含 '\n' */
  HAL_UART_Transmit(&huart6, (uint8_t *)p,
                    (uint16_t)(&text[sizeof(text) - 1] - p) + 1u, 10u);
}

/**
  * @brief  把 1~2 个定点小数作为一行、逗号分隔发出
  * @param  values: 每个都乘过 1000，夹到 ±9999
  * @param  count: 1 或 2
  * @note   摇杆是连续量，整数输出会被压成 -1/0/+1。
  *         最坏 2×"-9.999"+','+'\n' = 14 字符。
  */
static void uart6_send_milli(const int32_t *values, uint8_t count)
{
  char     text[24];
  char    *p;
  uint32_t magnitude;
  int32_t  value;
  uint8_t  i;
  uint8_t  digit;

  p = &text[sizeof(text) - 1];
  *p = '\n';

  for (i = count; i > 0u; i--)
  {
    value = values[i - 1u];

    if (value < -9999)
    {
      value = -9999;
    }
    else if (value > 9999)
    {
      value = 9999;
    }

    /* 取绝对值走 64 位，避免 INT32_MIN 溢出 */
    magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

    /* 先出三位小数（最低有效位先出） */
    for (digit = 0u; digit < 3u; digit++)
    {
      *--p = (char)('0' + (magnitude % 10u));
      magnitude /= 10u;
    }

    *--p = '.';

    /* 再拼整数部分。do-while 保证小于 1.000 时也先打一个 0 */
    do
    {
      *--p = (char)('0' + (magnitude % 10u));
      magnitude /= 10u;
    }
    while (magnitude > 0u);

    if (value < 0)
    {
      *--p = '-';
    }

    if (i > 1u)
    {
      *--p = ',';
    }
  }

  /* (尾指针 - 拼写指针) 即行长，不含 '\n' */
  HAL_UART_Transmit(&huart6, (uint8_t *)p,
                    (uint16_t)(&text[sizeof(text) - 1] - p) + 1u, 10u);
}

/**
  * @brief  把 DT7 摇杆通道归一化到对称区间
  * @param  ch: rc.ch[] 值（已减 1024），约 ±660，回中为 0
  * @param  full_milli: 满行程读数，单位千分之一（1000 即 ±1.000）
  * @retval -full_milli ~ +full_milli；死区内恒为 0
  * @note   死区避免松杆时曲线在中线附近抖动。
  */
static int32_t rc_stick_normalize(int16_t ch, int32_t full_milli)
{
  int32_t milli;

  if ((ch > -RC_DEAD_ZONE) && (ch < RC_DEAD_ZONE))
  {
    return 0;
  }

  milli = ((int32_t)ch * full_milli) / RC_CH_VALUE_RANGE;

  /* 摇杆实际能推过标称满量程一点点 */
  if (milli > full_milli)
  {
    milli = full_milli;
  }
  else if (milli < -full_milli)
  {
    milli = -full_milli;
  }

  return milli;
}

/**
  * @brief  转速环控制，跑在 SysTick 的 1kHz 中断里
  * @note   放中断里是因为 pid.c 的积分没有 dt，Ki 与调用周期绑定；主循环周期
  *         会抖到 2~4ms。SysTick 正好 1ms，无需额外定时器。
  *         优先级 15（最低），CAN 接收中断(0) 会抢占；两者访问的寄存器不重叠，
  *         读 speed_rpm 是单条 LDRH，安全。
  *         由 SysTick_Handler 在 HAL_IncTick() 之后调用。
  */
void motor_control_1khz(void)
{
  const RC_ctrl_t *rc = get_remote_control_point();
  uint32_t frame_count;
  uint32_t now;
  uint8_t  i;
  uint8_t  valid_num;
  int16_t  current[MOTOR_NUM];
  uint8_t  rx_ok[MOTOR_NUM];
  int32_t  target[MOTOR_NUM];
  int32_t  vx = 0;
  int32_t  vy = 0;
  int32_t  rot = 0;
  int32_t  max_abs = 0;
#if USE_SPEED_PID
  const int32_t limit = SPEED_TARGET_MAX_RPM;   /* 目标转速上限 */
#else
  const int32_t limit = CHASSIS_CURRENT_MAX;    /* 开环时 target[] 存的是电流 */
#endif

  /* 初始化完成前挡住整个控制环 */
  if (!control_enabled)
  {
    return;
  }

  now = HAL_GetTick();
  frame_count = rc_frame_count;

  /* 遥控看门狗：rc_ctrl 会保留上一帧的值，只能看帧计数判断链路是否活着 */
  if (frame_count != rc_last_frame_count)
  {
    rc_last_frame_count = frame_count;
    rc_last_frame_tick  = now;
    rc_online = 1;
  }
  else if ((now - rc_last_frame_tick) >= RC_SIGNAL_TIMEOUT_MS)
  {
    rc_online = 0;
  }

  /* 逐轮取反馈。speed_rpm 是转子转速，单条 16 位读对中断原子。
     超过 MOTOR_FEEDBACK_TIMEOUT_MS 没收到反馈的轮子命令置 0，
     否则初值 0 会让该路积分一直外推。 */
  valid_num = 0;
  for (i = 0u; i < MOTOR_NUM; i++)
  {
    rx_ok[i] = chassis_motor_is_online(i, now, MOTOR_FEEDBACK_TIMEOUT_MS);

    if (rx_ok[i])
    {
      /* 乘方向符号，统一成「朝车头方向滚为正」 */
      motor_actual_rpm[i] = (int16_t)((int32_t)motor_dir_sign[i]
                                    * (int32_t)get_chassis_motor_measure_point(i)->speed_rpm);
      valid_num++;
    }
    else
    {
      motor_actual_rpm[i] = 0;
    }
  }

  /* 摇杆 -> 三个自由度；遥控掉线或四轮全无反馈时全部归零 */
  if (rc_online && (valid_num > 0u))
  {
#if USE_SPEED_PID
    vx = rc_stick_normalize(rc->rc.ch[RC_MOTOR_CH_VERT], SPEED_TARGET_MAX_RPM);
    vy = rc_stick_normalize(rc->rc.ch[RC_MOTOR_CH_HORZ], SPEED_TARGET_MAX_RPM);
#if CHASSIS_ROTATE_ENABLE
    rot = rc_stick_normalize(rc->rc.ch[RC_MOTOR_CH_ROT], SPEED_ROTATE_MAX_RPM);
#endif
#else
    vx = rc_stick_normalize(rc->rc.ch[RC_MOTOR_CH_VERT], CHASSIS_CURRENT_MAX);
    vy = rc_stick_normalize(rc->rc.ch[RC_MOTOR_CH_HORZ], CHASSIS_CURRENT_MAX);
#endif
  }

  /* 横移/旋转符号跟随滚子朝向，前后不受影响 */
  vy  *= MECANUM_STRAFE_SIGN;
  rot *= MECANUM_ROTATE_SIGN;

  /* 麦轮逆运动学。vy 向左为正，rot 逆时针为正（俯视）。 */
  target[0] = vx - vy - rot;
  target[1] = vx + vy - rot;
  target[2] = vx + vy + rot;
  target[3] = vx - vy + rot;

  /* 等比例压回 limit：单轮限幅会把合成运动方向扭歪 */
  for (i = 0u; i < MOTOR_NUM; i++)
  {
    int32_t magnitude = (target[i] < 0) ? -target[i] : target[i];

    if (magnitude > max_abs)
    {
      max_abs = magnitude;
    }
  }

  if (max_abs > limit)
  {
    for (i = 0u; i < MOTOR_NUM; i++)
    {
      target[i] = target[i] * limit / max_abs;
    }
  }

  for (i = 0u; i < MOTOR_NUM; i++)
  {
    motor_target_rpm[i] = (int16_t)target[i];
  }

#if USE_SPEED_PID
  for (i = 0u; i < MOTOR_NUM; i++)
  {
    /* 掉线轮给 0，同时清积分，避免重新接上时猛冲 */
    if (!rx_ok[i])
    {
      current[i] = 0;
      PID_clear(&speed_pid[i]);
      continue;
    }

    /* 目标为 0 时清积分：只保留 P 项，刹车照样有力但不会甩尾 */
    if (target[i] == 0)
    {
      PID_clear(&speed_pid[i]);
    }

    /* 反馈和目标都在「朝车头滚为正」的坐标系里，PID 算出的电流再乘回
       方向符号。实参顺序 (pid, 反馈, 目标)：pid.c 算的是 set - ref。 */
    {
      fp32 out = PID_calc(&speed_pid[i], (fp32)motor_actual_rpm[i], (fp32)target[i]);

      current[i] = (int16_t)((int32_t)motor_dir_sign[i] * (int32_t)out);
    }
  }
#else
  /* 开环回退：摇杆直接映射电流，四轮仍走同一套运动学 */
  for (i = 0u; i < MOTOR_NUM; i++)
  {
    PID_clear(&speed_pid[i]);

    /* 开环没有反馈，方向符号只能乘在输出上 */
    current[i] = rx_ok[i]
               ? (int16_t)((int32_t)motor_dir_sign[i] * target[i])
               : 0;
  }
#endif

  for (i = 0u; i < MOTOR_NUM; i++)
  {
    /* C620 保护：不管上面算出多少，最终电流硬限幅在 ±16A */
    if (current[i] > C620_CURRENT_MAX_16A)
    {
      current[i] = C620_CURRENT_MAX_16A;
    }
    else if (current[i] < -C620_CURRENT_MAX_16A)
    {
      current[i] = -C620_CURRENT_MAX_16A;
    }

    motor_current_cmd[i] = current[i];
  }

  /* 四个 C620 的电流在同一帧 0x200 发出，data[0:1]~[6:7] 对应 ID1~ID4 */
  CAN_cmd_chassis(current[0], current[1], current[2], current[3]);
}

/**
  * @brief  UART6 调试流
  * @param  now: 当前 HAL_GetTick()
  * @param  rc: 最新遥控帧
  * @param  rc_online: 1 表示遥控链路正常
  * @note   '1' 正弦波，'3' 摇杆，'5' 转速环曲线，'2'/'4' 停。
  */
static void uart6_task(uint32_t now, const RC_ctrl_t *rc, uint8_t rc_online)
{
  uint8_t cmd;
  int32_t value;
  /* '5' 流一次发 8 列：M1~M4 目标、M1~M4 实测 */
  int32_t columns[8];

  /* 非阻塞取命令字节：查 RXNE 再读 DR，PC 不说话时循环不会卡 */
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
  {
    cmd = (uint8_t)(huart6.Instance->DR & 0xFFu);

    if (cmd == '1')
    {
      uart6_stream = UART6_STREAM_SINE;
    }
    else if (cmd == '3')
    {
      uart6_stream = UART6_STREAM_STICK;
    }
    else if (cmd == '5')
    {
      uart6_stream = UART6_STREAM_PID;
    }
    else if (cmd == '2' || cmd == '4')
    {
      uart6_stream = UART6_STREAM_OFF;
    }
  }

  if (uart6_stream == UART6_STREAM_SINE)
  {
    /* 相位取自毫秒时钟而非采样计数，周期不随循环抖动 */
    value = SINE_MID + (int32_t)(sinf(2.0f * SINE_PI
                                      * (float)(now % SINE_PERIOD_MS)
                                      / (float)SINE_PERIOD_MS)
                                  * (float)SINE_AMPLITUDE);
    columns[0] = value;
    uart6_send_int(columns, 1u);
  }
  else if (uart6_stream == UART6_STREAM_STICK)
  {
    if (rc_online)
    {
      /* 第 1 列 = 左摇杆上下，第 2 列 = 左摇杆左右 */
      columns[0] = rc_stick_normalize(rc->rc.ch[RC_STICK_CH_VERT],
                                      RC_STICK_VERT_MILLI);
      columns[1] = rc_stick_normalize(rc->rc.ch[RC_STICK_CH_HORZ],
                                      RC_STICK_HORZ_MILLI);
    }
    else
    {
      /* 链路断开时 rc->rc.ch[] 仍是旧值，报中位而不是过期的偏量 */
      columns[0] = 0;
      columns[1] = 0;
    }

    uart6_send_milli(columns, 2u);
  }
  else if (uart6_stream == UART6_STREAM_PID)
  {
    /* 第 1~4 列 = M1~M4 目标转速，第 5~8 列 = M1~M4 实测转速，单位 rpm，
       均以「朝车头滚」为正。目标/实测一对可整定每路 PID，也能看出掉线
       （恒 0）或方向符号反了（一正一负）。 */
    columns[0] = motor_target_rpm[0];
    columns[1] = motor_target_rpm[1];
    columns[2] = motor_target_rpm[2];
    columns[3] = motor_target_rpm[3];
    columns[4] = motor_actual_rpm[0];
    columns[5] = motor_actual_rpm[1];
    columns[6] = motor_actual_rpm[2];
    columns[7] = motor_actual_rpm[3];
    uart6_send_int(columns, 8u);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  /* 配置 USART3 接收 DMA 和空闲中断，不调用则遥控数据不会更新 */
  remote_control_init();

  /* PH11 点亮，RUN_LED_ON_MS 后灭 */
  boot_tick = HAL_GetTick();
  HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN, GPIO_PIN_SET);

  /* 转速环初始化：CAN 和遥控都配好后才能放开控制环 */
  {
    const fp32 speed_pid_para[3] = {SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD};
    uint8_t i;

    /* 四路参数相同；需要单独补偿某一路时按 i 分开填 */
    for (i = 0u; i < MOTOR_NUM; i++)
    {
      PID_init(&speed_pid[i], PID_POSITION, speed_pid_para,
               SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
    }
  }

  control_enabled = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const RC_ctrl_t *rc = get_remote_control_point();
    uint32_t now = HAL_GetTick();

    /* PH11：上电后亮 RUN_LED_ON_MS，之后灭 */
    HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN,
                      ((now - boot_tick) < RUN_LED_ON_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* --- CAN1 bus-off 自动恢复 ---
       can.c 里 AutoBusOff=DISABLE、AutoRetransmission=ENABLE。电调不在总线上时
       0x200 无人 ACK，TEC 超限进入 bus-off 且不会自愈，只能重新上电。
       这里检测到 BOFF 就重新初始化，让 CAN1 自己恢复。
       放在主循环而非中断：HAL_CAN_Init() 会复位整个外设，而 CAN1_RX0 中断
       优先级更高、可能正在访问它。 */
    if ((CHASSIS_CAN.Instance->ESR & CAN_ESR_BOFF) != 0u)
    {
      control_enabled = 0;              /* 挡住中断里的 CAN 发送 */
      HAL_CAN_Init(&CHASSIS_CAN);
      can_filter_init();
      control_enabled = 1;
    }

    /* 让云台电机保持不动；没有云台就删掉这行 */
    CAN_cmd_gimbal(0, 0, 0, 0);

    /* 电机命令在 motor_control_1khz() 里发（周期更稳）。
       UART6 放最后，慢串口不拖慢别的东西。 */
    uart6_task(now, rc, rc_online);

    HAL_Delay(2);

    /* Watch 窗口可加：rc_ctrl.rc.ch、motor_target_rpm[0..3]、
       motor_actual_rpm[0..3]、motor_current_cmd[0..3] */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
