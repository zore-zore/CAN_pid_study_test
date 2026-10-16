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
/* 转速环 PID。这个文件由官方提供，本次没有改动 */
#include "pid.h"
/* sinf()，UART6 正弦波输出要用 */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 摇杆值减掉 RC_CH_VALUE_OFFSET(1024) 之后的行程，约 -660~+660 */
#define RC_CH_VALUE_RANGE       660
/* 摇杆回中后的零点漂移，这个范围内一律当 0 */
#define RC_DEAD_ZONE            20
/* 摇杆推到底时发给 3508 的电流。C620 内部自己跑电流环，16384 对应它的满
   量程 20A，所以 1000 大约 1.2A。
   现在默认走转速闭环（下面 USE_SPEED_PID = 1），这个值只在把它置 0、退回
   开环时才用得到。
   开环本来就治不了"同一个电流在不同温度/摩擦下转速会飘"这件事，要控转速得
   闭环，见下面 --- 转速环 PID --- 那段。
   还嫌快 -> 继续调小；不动了 -> 调大。 */
#define CHASSIS_CURRENT_MAX     1000
/* 台架测试：只让这一个 3508 动，另外三个强制给 0。
   0=M1 1=M2 2=M3 3=M4，换电机改这里。 */
#define SINGLE_MOTOR_INDEX      0
/* 用哪个遥控器通道驱动电机：2=左摇杆上下，3=左摇杆左右。
   单独一个宏，不复用下面显示用的那个 —— 驱动用哪个轴、显示哪几个轴是两件
   事，绑在一起的话，为了改显示动一下，电机就悄悄换了轴。 */
#define RC_MOTOR_CHANNEL        2
/* 板载指示灯。gpio.c 已经把它配成推挽输出并拉低，这里只管写。
   PH11：上电后亮 RUN_LED_ON_MS，然后灭 —— 它是"程序跑起来了"的标志，
   不是心跳灯。 */
#define RUN_LED_PORT            GPIOH
#define RUN_LED_PIN             GPIO_PIN_11
/* PH11 上电后亮多久 */
#define RUN_LED_ON_MS           8000
/* 遥控器静默超过多久算掉线。DBUS 约 14ms 一帧，100ms 差不多漏 7 帧。 */
#define RC_SIGNAL_TIMEOUT_MS    100
/* UART6 调试输出。用串口助手接 UART6（PG14=TX, PG9=RX, 115200 8N1 无校验），
   发送对应字符：
     '1' - 开始正弦波流，一个主循环一行
     '2' - 停
     '3' - 开始左摇杆流，两个通道：上下 -1.000~+1.000，左右 +0.500~-0.500，
           中间都是 0.000
     '4' - 停
     '5' - 开始转速环曲线流，两个通道：第1列目标转速(rpm)，第2列实测转速(rpm)
   '2' 和 '4' 都能停任何一条流。
   三条流都是 ASCII 十进制、逗号分列，这是 SerialPlot 要的格式。其它字节一律
   忽略。同一时刻只能跑一条流，它们共用 UART6 这一根 TX 线。
   '5' 是整定 Kp/Ki 的唯一实用手段：一次性读几个数看不出超调和振荡，曲线能。 */
#define UART6_STREAM_OFF        0
#define UART6_STREAM_SINE       1
#define UART6_STREAM_STICK      2
#define UART6_STREAM_PID        3

/* --- 正弦波流 ('1') --- */
/* 一个完整周期的时长，单位 ms。400ms = 2.5Hz */
#define SINE_PERIOD_MS          400
/* 相对区间中点的最大偏移 */
#define SINE_AMPLITUDE          2047
/* 正弦为 0 时的输出值（0~4095 的中点） */
#define SINE_MID                2048
#define SINE_PI                 3.14159265f

/* --- 摇杆流 ('3') --- */
/* 左摇杆有两个轴，'3' 一次发两个，逗号分列。2 是上下轴，3 是左右轴
   （1 和 0 是右摇杆的上下和左右）。 */
#define RC_STICK_CH_VERT        2
#define RC_STICK_CH_HORZ        3
/* 每个轴推到底时的读数，单位是千分之一。上是 +1.000，下是 -1.000；
   左是 +0.500，右是 -0.500。这些值不是编出来的：摇杆往左，ch[3] 本来就是正
   的，所以只需要把幅度减半，不用取反。左右用半幅，是为了让两条曲线在
   SerialPlot 里视觉上一样大（上下轴是它的两倍）。
   整数输出会把整条曲线压成 -1/0/+1 三条线，所以这条流不能用 uart6_send_int()。 */
#define RC_STICK_VERT_MILLI     1000
#define RC_STICK_HORZ_MILLI     500

/* --- 转速环 PID ---
   摇杆 -> 目标转速 -> PID -> 电流 -> C620 -> 3508，测回来的 speed_rpm 当反馈。
   控制环跑在 SysTick 的 1kHz 中断里，见 motor_control_1khz()。 */

/* 回退开关：置 0 就退回「摇杆直接给电流」的开环控制，和接 PID 之前一模一样，
   用来对比闭环到底有没有起作用。 */
#define USE_SPEED_PID           1

/* 摇杆推到底对应的目标转速，单位 rpm。
   C620 反馈的是电机转子转速（不是减速箱输出轴），3508 空载约 9000，取 3000
   留余量。
   第一次上台架建议先改成 1000，确认转向和控制量都对了再往上加。 */
#define SPEED_TARGET_MAX_RPM    3000

/* PID 三参数。还没整定过 —— 电机和电调现在都拆着，测不了，所以这几个数只是
   一个保守的起点，不是整定结果。
   顺序建议：Kd 保持 0、Ki 先给 0，只加 Kp 到「响应够快、刚出现一点过冲」再
   退一档；然后慢慢加 Ki 把稳态残差压下去。全程开着 '5' 曲线流看。
   Ki 不能随手给：pid.c 的积分是 Iout += Ki * error，纯累加、没有乘 dt，所以
   Ki 的物理含义和 1kHz 这个调用周期是绑死的，换了控制周期就得重新整定。 */
#define SPEED_PID_KP            10.0f
#define SPEED_PID_KI            0.2f
#define SPEED_PID_KD            0.0f

/* PID 输出上限，也就是电流上限。3508 额定 10A 对应约 8192 计数；这里取 3000
   （约 3.7A），空载台架够用，也更安全。转不动再往上加。 */
#define SPEED_PID_MAX_OUT       3000.0f

/* 积分项独立上限，防止堵转时积分一直往里累（积分饱和），一松手电机猛冲。 */
#define SPEED_PID_MAX_IOUT      2000.0f

/* 电机物理装反了改这里：填 1 或 -1。
   注意它翻的是「反馈」和「目标」这两个一起翻，不是翻输出 —— 闭环里只翻输出
   会把负反馈变成正反馈，电机直接发散。推导（填 -1 的情形）：
     正电流物理上产生负转速；摇杆推上，目标 +3000，翻完 target = -3000；
     PID 算出负电流 -> 物理上产生正转速 -> 翻完 actual 为负 -> 误差往 0 收。
     负反馈成立。 */
#define MOTOR_DIR_SIGN          1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 遥控链路看门狗。rc_frame_count 在 remote_control.c 里，由 DBUS 中断自增；
   其余三个在 motor_control_1khz() 里更新，也就是 SysTick 中断上下文。
   rc_online 主循环里的 uart6_task() 也要读，所以是 volatile。 */
static uint32_t rc_last_frame_count = 0;
static uint32_t rc_last_frame_tick  = 0;
static volatile uint8_t rc_online   = 0;

/* 转速环 PID 实例。参数在 main() 的 PID_init() 里填。 */
static pid_type_def speed_pid;

/* 控制环总开关。main() 里 can_filter_init() 和 PID_init() 都做完了才置 1 ——
   HAL_Init() 一返回 SysTick 就开始跑，那时 CAN 和 PID 都还没准备好。
   主循环里那段落 bus-off 恢复也会临时清 0，挡住中断里的 CAN 发送。 */
volatile uint8_t control_enabled = 0;

/* 观测变量，给 UART6 的 '5' 曲线流和 Keil Watch 用。
   都在 1kHz 中断里写、主循环里读，但全是单次 16 位读写，对中断天然原子，
   不需要临界区。 */
volatile int16_t motor_target_rpm  = 0;
volatile int16_t motor_actual_rpm  = 0;
volatile int16_t motor_current_cmd = 0;

/* 上电时抓的时间戳，PH11 用它算自己的点亮窗口 */
static uint32_t boot_tick = 0;

/* 当前选中的 UART6 流，取值 UART6_STREAM_*。由 UART6 收到的 '1'~'4' 决定，
   只会在主循环里被改。 */
static uint8_t uart6_stream = UART6_STREAM_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief          把 1~3 个整数作为一行、逗号分隔发出去
  * @param[in]      values: 每个都会被夹到 -99999~99999
  * @param[in]      count: 发几个，1~3
  * @retval         none
  * @note           数字是手写的。这里没有 printf（USART3 被 DBUS 接收占了），
  *                 为了几个整数把整个格式化库拖进来不划算。
  *                 整行是从 '\n' 往前倒着拼的，所以列的先后顺序是对的，
  *                 写入顺序是反的。
  *                 最坏情况 3 x "-16384" + 2 x ',' + '\n' = 23 个字符。
  */
static void uart6_send_int(const int32_t *values, uint8_t count)
{
  char     text[32];
  char    *p;
  uint32_t magnitude;
  int32_t  value;
  uint8_t  i;

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

    /* 取绝对值走 64 位，避免 INT32_MIN 像 -value 那样溢出 */
    magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

    /* 数字是倒着出来的，所以从缓冲区尾部往前写，发送起点就是拼到哪儿算哪儿 */
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

  /* (尾指针 - 拼写指针) 就是行长，还不含那个 '\n' */
  HAL_UART_Transmit(&huart6, (uint8_t *)p,
                    (uint16_t)(&text[sizeof(text) - 1] - p) + 1u, 10u);
}

/**
  * @brief          把 1~2 个定点小数作为一行、逗号分隔发出去
  * @param[in]      values: 每个都乘过 1000，会被夹到 -9999~9999
  * @param[in]      count: 发几个，1 或 2
  * @retval         none
  * @note           和 uart6_send_int() 一样是手写的，多了个小数点。摇杆是连续
  *                 的，整数输出会把整条曲线压成 -1/0/+1。
  *                 整行同样是从 '\n' 往前倒着拼的。
  *                 最坏情况 2 x "-9.999" + ',' + '\n' = 14 个字符。
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

    /* 取绝对值走 64 位，避免 INT32_MIN 像 -value 那样溢出 */
    magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

    /* 三位小数是最低有效位先出来 */
    for (digit = 0u; digit < 3u; digit++)
    {
      *--p = (char)('0' + (magnitude % 10u));
      magnitude /= 10u;
    }

    *--p = '.';

    /* 再拼整数部分。用 do-while，这样小于 1.000 的值也会先打一个 0，
       而不是直接从点开始 */
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

  /* (尾指针 - 拼写指针) 就是行长，还不含那个 '\n' */
  HAL_UART_Transmit(&huart6, (uint8_t *)p,
                    (uint16_t)(&text[sizeof(text) - 1] - p) + 1u, 10u);
}

/**
  * @brief          把一个 DT7 摇杆通道归一化到对称区间
  * @param[in]      ch: rc.ch[] 的值，已经减过 1024，约 -660~+660，回中为 0
  * @param[in]      full_milli: 推到底时的读数，单位千分之一：
  *                 1000 对应 -1.000~+1.000，500 对应 -0.500~+0.500
  * @retval         -full_milli~+full_milli；死区内恒为 0
  * @note           死区是为了松开摇杆时曲线不在中线上下抖：真实摇杆静止时离零
  *                 有几个数的偏差，没有死区图线永远稳不到 0.000。
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
  * @brief          转速环控制，跑在 SysTick 的 1kHz 中断里
  * @param[in]      none
  * @retval         none
  * @note           为什么放中断里而不是主循环里：pid.c 的积分是
  *                 Iout += Ki * error，纯累加、没有乘 dt，所以 Ki 的物理含义
  *                 直接绑死在调用周期上，周期一抖积分就在瞎积。而主循环里有
  *                 HAL_Delay(2) 和会阻塞的串口发送，周期在 2~4ms 之间抖 ——
  *                 更巧的是整定时必然开着 '5' 曲线流，抖动正好发生在最要它稳
  *                 的时候。
  *                 工程里没有定时器（HAL_TIM_MODULE_ENABLED 是注释掉的），但
  *                 SysTick 本来就跑着，而且正好是 1ms，直接白捡一个准周期，
  *                 不用动时钟配置，也不用加外设。
  *                 PID 一次计算约 50 条浮点指令，168MHz 下不到 1µs，1ms 的预算
  *                 有三位数余量。
  *                 优先级 15（最低），CAN 接收中断（优先级 0）会抢占这里。这是
  *                 安全的：这里读 speed_rpm 是单条 LDRH；这里写发送邮箱碰的是
  *                 TSR + sTxMailBox[].TIR/TDTR/TDHR/TDLR，而接收路径
  *                 HAL_CAN_GetRxMessage 只碰 RF0R/RDLR/RDHR/RDTR/RI0R，两组寄
  *                 存器不重叠，hcan->State 双方都只读。
  *                 由 stm32f4xx_it.c 的 SysTick_Handler 调用，必须在
  *                 HAL_IncTick() 之后 —— 这样这里 HAL_GetTick() 拿到的是本 tick
  *                 的值。
  */
void motor_control_1khz(void)
{
  const RC_ctrl_t       *rc      = get_remote_control_point();
  const motor_measure_t *measure = get_chassis_motor_measure_point(SINGLE_MOTOR_INDEX);
  uint32_t frame_count;
  uint32_t now;
  int16_t  current;

  /* 上电后 SysTick 就在跑，那时 PID_init() 还没执行、CAN 滤波器也还没配。
     这个标志把控制环整个挡住，直到 main() 里初始化完成。 */
  if (!control_enabled)
  {
    return;
  }

  now = HAL_GetTick();
  frame_count = rc_frame_count;

  /* 遥控链路看门狗。rc_ctrl 的内容会一直保留上一帧的值，所以判断链路是否活着
     要看帧计数而不是数据本身。第一帧到来之前 rc_online 一直是 0。 */
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

  /* speed_rpm 是电调反馈的电机转子转速（3508 空载约 9000），不是减速箱输出轴。
     单条 16 位读，对 CAN 接收中断是原子的。
     放在 #if 外面：开环回退时也要更新，否则 '5' 流的第二列恒为 0。 */
  motor_actual_rpm = measure->speed_rpm;

#if USE_SPEED_PID
  if (rc_online)
  {
    /* 摇杆位置 -> 目标转速，回中为 0，推到底 ±SPEED_TARGET_MAX_RPM */
    motor_target_rpm = (int16_t)rc_stick_normalize(rc->rc.ch[RC_MOTOR_CHANNEL],
                                                   SPEED_TARGET_MAX_RPM);
  }
  else
  {
    /* 掉线：目标归零，并且清掉积分。不清的话掉线期间积分项会继续往里累，
       摇杆一回来电机会猛地弹一下。
       只发 0 电流也不行 —— 那样连 P 项都不起作用，残余转速要慢慢才停。 */
    motor_target_rpm = 0;
    PID_clear(&speed_pid);
  }

  /* 方向反转只能把「反馈」和「目标」两个一起翻，输出保持原样。只翻输出会把
     负反馈变成正反馈，电机发散。推导见 MOTOR_DIR_SIGN 的注释。 */
  {
    fp32 actual = (fp32)MOTOR_DIR_SIGN * (fp32)motor_actual_rpm;
    fp32 target = (fp32)MOTOR_DIR_SIGN * (fp32)motor_target_rpm;

    /* 实参顺序是 (pid, 反馈, 目标)：pid.c 里算的是 set - ref，所以实测转速在
       前、目标转速在后。写反了同样变成正反馈。 */
    current = (int16_t)PID_calc(&speed_pid, actual, target);
  }
#else
  /* 回退到开环：摇杆直接映射电流，和接 PID 之前的行为一致，方便对比 */
  {
    int32_t open_loop = rc_online
                      ? rc_stick_normalize(rc->rc.ch[RC_MOTOR_CHANNEL],
                                           CHASSIS_CURRENT_MAX)
                      : 0;

    /* 开环没有「目标转速」这个概念，motor_target_rpm 保持 0，不要拿电流去填它。 */
    PID_clear(&speed_pid);
    current = (int16_t)open_loop;
  }
#endif

  motor_current_cmd = current;

  /* 台架测试：只驱动 SINGLE_MOTOR_INDEX 那一个，另外三个恒给 0，不然没接电机
     的那几路也一直在往总线上打命令。 */
  CAN_cmd_chassis((SINGLE_MOTOR_INDEX == 0) ? current : 0,
                  (SINGLE_MOTOR_INDEX == 1) ? current : 0,
                  (SINGLE_MOTOR_INDEX == 2) ? current : 0,
                  (SINGLE_MOTOR_INDEX == 3) ? current : 0);
}

/**
  * @brief          UART6 调试流
  * @param[in]      now: 调用方抓的 HAL_GetTick() 值
  * @param[in]      rc: 最新的遥控帧
  * @param[in]      rc_online: 1 表示遥控器还在跟我们说话
  * @retval         none
  * @note           '1' 开正弦波流，'3' 开摇杆流，'5' 开转速环曲线流，'2'/'4' 停。
  *                 每个主循环调一次，一次最多阻塞一短行的时间。
  */
static void uart6_task(uint32_t now, const RC_ctrl_t *rc, uint8_t rc_online)
{
  uint8_t cmd;
  int32_t value;
  int32_t columns[3];

  /* PC 发来一个字节就取走。手动查 RXNE 标志再读 DR 是非阻塞的 —— 没有超时
     要等，PC 不说话时循环也不会卡。UART6 的接收只在这里读，所以两条流不会
     互相吃掉对方的命令字节。 */
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
    /* 相位取自毫秒时钟而不是采样计数：循环周期会抖，这样不管循环跑多快，
       一个完整周期都正好是 SINE_PERIOD_MS。 */
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
      /* 链路断了，rc->rc.ch[] 还停在最后一帧的值上。这里报中位而不是那个
         过期的偏量 —— 和上面把电机命令强制成 0 是同一个道理。 */
      columns[0] = 0;
      columns[1] = 0;
    }

    uart6_send_milli(columns, 2u);
  }
  else if (uart6_stream == UART6_STREAM_PID)
  {
    /* 第 1 列 = 目标转速，第 2 列 = 实测转速，单位都是 rpm。
       两个都是中断里写的 volatile，这里读到的是同一时刻附近的快照 —— 曲线的
       周期比控制周期慢两个数量级，差一个采样周期看不出来。
       用整数流就够：这里的分辨率是 1rpm，不是摇杆流那种会被压成几条线的量。 */
    columns[0] = motor_target_rpm;
    columns[1] = motor_actual_rpm;
    uart6_send_int(columns, 2u);
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
  /* 把 USART3 的接收 DMA 指到双缓冲上并打开空闲中断；
     不调这句，遥控数据永远不会更新 */
  remote_control_init();

  /* PH11 现在点亮，RUN_LED_ON_MS 之后灭 */
  boot_tick = HAL_GetTick();
  HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN, GPIO_PIN_SET);

  /* 转速环 PID。CAN 和遥控都配好了再初始化，最后才放开控制环 ——
     SysTick 从 HAL_Init() 起就在跑，放开的时机决定了中断里第一帧会不会打到
     一个没配好的外设上。 */
  {
    const fp32 speed_pid_para[3] = {SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD};

    PID_init(&speed_pid, PID_POSITION, speed_pid_para,
             SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
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
       can.c 里 hcan1 是 AutoBusOff = DISABLE、AutoRetransmission = ENABLE 的。
       电调不在总线上时，0x200 发出去没人 ACK，无限重传，TEC 涨过 255 就进
       bus-off，而且 AutoBusOff 关着意味着它不会自己回来 —— 之后把电调接回去，
       CAN1 依然是死的，只能重新上电。所以这里自己接回来。
       同时 AutoRetransmission 开着时 3 个发送邮箱几毫秒就占满，
       HAL_CAN_AddTxMessage 之后全部静默返回 HAL_ERROR。
       电调一直不在的话这段会每隔几毫秒跑一次（重新配好、又 bus-off、再配），
       属于预期行为，不是死循环。
       放在主循环而不是 1kHz 中断里：HAL_CAN_Init() 会把 CAN 外设整个复位重配，
       而 CAN1_RX0 中断（优先级 0）能抢占 SysTick（优先级 15），在中断里重配一个
       会被更高优先级中断访问的外设是自找麻烦。bus-off 恢复也不是实时任务。 */
    if ((CHASSIS_CAN.Instance->ESR & CAN_ESR_BOFF) != 0u)
    {
      control_enabled = 0;              /* 挡住中断里的 CAN 发送 */
      HAL_CAN_Init(&CHASSIS_CAN);
      can_filter_init();
      control_enabled = 1;
    }

    /* 让云台电机保持不动；没有云台就删掉这行 */
    CAN_cmd_gimbal(0, 0, 0, 0);

    /* 电机命令不在这里发：遥控看门狗和 CAN_cmd_chassis 都在 1kHz 的
       motor_control_1khz() 里，那边周期才稳。

       UART6 调试：'1' 正弦波，'3' 左摇杆，'5' 转速环曲线，'2'/'4' 停。
       故意放在最后，这样慢的串口不会拖慢别的东西。 */
    uart6_task(now, rc, rc_online);

    HAL_Delay(2);

    /* 想看原始遥控值时，打开 Keil 的 Watch 窗口加 "rc_ctrl.rc.ch" 就行，
       不用 printf（USART3 被 DBUS 接收占着，打不了）。
       Watch 里加 motor_target_rpm / motor_actual_rpm / motor_current_cmd
       能直接看闭环在干什么，不想开曲线流的时候用。 */
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
