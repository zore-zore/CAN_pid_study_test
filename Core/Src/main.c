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
   注意这不是转速命令：电调只管把电流喂进去，电机靠摩擦和反电动势自己找平衡
   点，所以同一个电流在不同温度、不同摩擦下转速会飘。要真正控制转速得闭环，
   见 user/pid.c（目前还没接上）。
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
   '2' 和 '4' 都能停任何一条流。
   两条流都是 ASCII 十进制、逗号分列，这是 SerialPlot 要的格式。其它字节一律
   忽略。同一时刻只能跑一条流，它们共用 UART6 这一根 TX 线。 */
#define UART6_STREAM_OFF        0
#define UART6_STREAM_SINE       1
#define UART6_STREAM_STICK      2

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 遥控链路看门狗。rc_frame_count 在 remote_control.c 里，由 DBUS 中断自增；
   其余三个在主循环里更新。 */
static uint32_t rc_last_frame_count = 0;
static uint32_t rc_last_frame_tick  = 0;
static uint8_t  rc_online           = 0;

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
  * @brief          把一个 DT7 通道值换算成电机电流
  * @param[in]      channel: 已经减掉 RC_CH_VALUE_OFFSET 的通道值，
  *                 约 -660~+660，摇杆回中时为 0
  * @param[in]      max_current: 摇杆推到底时发的电流
  * @retval         电机电流，-max_current~+max_current，回中时为 0
  */
static int16_t rc_to_current(int16_t channel, int16_t max_current)
{
  int32_t value = channel;

  if (value > -RC_DEAD_ZONE && value < RC_DEAD_ZONE)
  {
    value = 0;
  }

  if (value > RC_CH_VALUE_RANGE)
  {
    value = RC_CH_VALUE_RANGE;
  }
  else if (value < -RC_CH_VALUE_RANGE)
  {
    value = -RC_CH_VALUE_RANGE;
  }

  return (int16_t)(value * max_current / RC_CH_VALUE_RANGE);
}

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
  * @brief          UART6 调试流
  * @param[in]      now: 调用方抓的 HAL_GetTick() 值
  * @param[in]      rc: 最新的遥控帧
  * @param[in]      rc_online: 1 表示遥控器还在跟我们说话
  * @retval         none
  * @note           '1' 开正弦波流，'3' 开摇杆流，'2'/'4' 停。每个主循环调一次，
  *                 一次最多阻塞一短行的时间。
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const RC_ctrl_t *rc = get_remote_control_point();
    uint32_t frame_count = rc_frame_count;
    uint32_t now = HAL_GetTick();
    int16_t throttle;

    /* 遥控器还在说话吗？rc_ctrl 的值会一直保留，所以判断链路是否活着要看帧
       计数而不是数据本身。第一帧到来之前 rc_online 一直是 0。 */
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

    /* PH11：上电后亮 RUN_LED_ON_MS，之后灭 */
    HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN,
                      ((now - boot_tick) < RUN_LED_ON_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 台架测试：左摇杆上下驱动一个 3508，另外三个给 0。SINGLE_MOTOR_INDEX
       决定是哪一个。
       这里只读上下轴，左右推到底电机也不动。
       上是正转（正电流），下是反转。
       掉线时命令 0 很重要：不然 rc_ctrl 会保留最后的摇杆位置，电机就飞了。 */
    throttle = rc_online ? rc_to_current(rc->rc.ch[RC_MOTOR_CHANNEL],
                                         CHASSIS_CURRENT_MAX) : 0;

    /* 电机转向反了，就把下面 "throttle" 前面加个负号 */
    CAN_cmd_chassis((SINGLE_MOTOR_INDEX == 0) ? throttle : 0,
                    (SINGLE_MOTOR_INDEX == 1) ? throttle : 0,
                    (SINGLE_MOTOR_INDEX == 2) ? throttle : 0,
                    (SINGLE_MOTOR_INDEX == 3) ? throttle : 0);

    /* 让云台电机保持不动；没有云台就删掉这行 */
    CAN_cmd_gimbal(0, 0, 0, 0);

    /* UART6 调试：'1' 正弦波，'3' 左摇杆，'2'/'4' 停。故意放在 CAN 发完之后，
       这样慢的串口不会拖慢电机命令。 */
    uart6_task(now, rc, rc_online);

    HAL_Delay(2);

    /* 想看原始遥控值时，打开 Keil 的 Watch 窗口加 "rc_ctrl.rc.ch" 就行，
       不用 printf（USART3 被 DBUS 接收占着，打不了）。 */
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
