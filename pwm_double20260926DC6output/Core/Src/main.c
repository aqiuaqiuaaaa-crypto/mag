/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "current_sense.h"
#include "uart_telemetry.h"
#include "../SYSTEM/delay/delay.h"
#include "../BSP/EXTI/exti.h"
#include "../BSP/LED/led.h"

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 上位机发送格式：
 *
 * a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n
 *
 * 总长度：
 *
 * a0:+30,  -> 7
 * a1:-45,  -> 7
 * a2:+60,  -> 7
 * a3:-75,  -> 7
 * a4:+90,  -> 7
 * a5:-10   -> 6
 * \r\n     -> 2
 *
 * 总计：7*5 + 6 + 2 = 43
 */

#define RX_BUF_SIZE     64
#define FRAME_LEN       43

/* 六路电流允许范围 */
#define AMP_MIN         (-99)
#define AMP_MAX         (99)

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t pwm_started = 0;

/* UART DMA接收缓冲区 */
static uint8_t rx_buf[RX_BUF_SIZE];

/*
 * DMA接收完成后，把数据复制到这里。
 * 避免重新启动DMA以后，DMA继续修改rx_buf，
 * 而主循环同时读取rx_buf造成数据竞争。
 */
static uint8_t process_buf[RX_BUF_SIZE];

/* UART发送缓冲区 */
static uint8_t tx_buf[16];

static volatile uint8_t rx_done = 0;
static volatile uint16_t rx_len = 0;

/* 六路电磁铁电流 */
static volatile int a0_amp = 0;
static volatile int a1_amp = 0;
static volatile int a2_amp = 0;
static volatile int a3_amp = 0;
static volatile int a4_amp = 0;
static volatile int a5_amp = 0;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void Start_PWM(void);
void Stop_PWM(void);

/*
 * 解析带符号三位数：
 *
 * +30 -> 30
 * -45 -> -45
 * +00 -> 0
 * -05 -> -5
 */
static int PARSE_SIGNED(const uint8_t *buf, uint16_t idx);

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 解析固定3字符的带符号电流值
 *
 * 格式：
 *      +30
 *      -45
 *      +00
 *      -05
 *
 * @param buf  数据缓冲区
 * @param idx  符号所在位置
 *
 * @return 解析后的有符号整数
 */
static int PARSE_SIGNED(const uint8_t *buf, uint16_t idx)
{
    int sign;
    int value;

    /* 检查符号 */
    if (buf[idx] == '+')
    {
        sign = 1;
    }
    else if (buf[idx] == '-')
    {
        sign = -1;
    }
    else
    {
        return 0;
    }

    /* 检查十位数字 */
    if (buf[idx + 1] < '0' || buf[idx + 1] > '9')
    {
        return 0;
    }

    /* 检查个位数字 */
    if (buf[idx + 2] < '0' || buf[idx + 2] > '9')
    {
        return 0;
    }

    value = (buf[idx + 1] - '0') * 10
          + (buf[idx + 2] - '0');

    value *= sign;

    /* 限制范围 */
    if (value > AMP_MAX)
    {
        value = AMP_MAX;
    }

    if (value < AMP_MIN)
    {
        value = AMP_MIN;
    }

    return value;
}


/**
 * @brief 检查接收到的数据是不是完整的43字节协议帧
 */
static uint8_t Check_Frame(const uint8_t *buf, uint16_t len)
{
    if (len < FRAME_LEN)
    {
        return 0;
    }

    /*
     * 检查固定格式中的关键字符
     */

    /* a0:+30, */
    if (buf[0]  != 'a' ||
        buf[1]  != '0' ||
        buf[2]  != ':' ||
        (buf[3]  != '+' && buf[3] != '-') ||
        buf[6]  != ',')
    {
        return 0;
    }

    /* a1:-45, */
    if (buf[7]  != 'a' ||
        buf[8]  != '1' ||
        buf[9]  != ':' ||
        (buf[10] != '+' && buf[10] != '-') ||
        buf[13] != ',')
    {
        return 0;
    }

    /* a2:+60, */
    if (buf[14] != 'a' ||
        buf[15] != '2' ||
        buf[16] != ':' ||
        (buf[17] != '+' && buf[17] != '-') ||
        buf[20] != ',')
    {
        return 0;
    }

    /* a3:-75, */
    if (buf[21] != 'a' ||
        buf[22] != '3' ||
        buf[23] != ':' ||
        (buf[24] != '+' && buf[24] != '-') ||
        buf[27] != ',')
    {
        return 0;
    }

    /* a4:+90, */
    if (buf[28] != 'a' ||
        buf[29] != '4' ||
        buf[30] != ':' ||
        (buf[31] != '+' && buf[31] != '-') ||
        buf[34] != ',')
    {
        return 0;
    }

    /* a5:-10 */
    if (buf[35] != 'a' ||
        buf[36] != '5' ||
        buf[37] != ':' ||
        (buf[38] != '+' && buf[38] != '-'))
    {
        return 0;
    }

    /* 检查帧尾 */
    if (buf[41] != '\r' ||
        buf[42] != '\n')
    {
        return 0;
    }

    return 1;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();


  /* USER CODE BEGIN 2 */

  /* 上电时关闭PWM */
  Stop_PWM();

  /* TIM2周期 */
  TIM2->ARR = 41999;

  /* Measurement-only ADC path; failures do not change PWM. */
  (void)CurrentSense_Start();

  /*
   * 启动UART DMA + IDLE接收
   */
  HAL_UARTEx_ReceiveToIdle_DMA(
      &huart1,
      rx_buf,
      RX_BUF_SIZE
  );

  /*
   * 禁止DMA半传输中断
   */
  __HAL_DMA_DISABLE_IT(
      huart1.hdmarx,
      DMA_IT_HT
  );

  /* USER CODE END 2 */


  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */

    if (rx_done)
    {
        uint16_t len;

        int t0;
        int t1;
        int t2;
        int t3;
        int t4;
        int t5;

        /*
         * 先清除标志
         */
        rx_done = 0;

        /*
         * 读取本次接收长度
         */
        len = rx_len;


        /*
         * 必须是完整43字节帧
         */
        if (len >= FRAME_LEN)
        {
            /*
             * 检查协议格式
             */
            if (Check_Frame(process_buf, len))
            {
                /*
                 * 解析6路电流
                 *
                 * a0:+30
                 *     ^
                 *     3
                 */
                t0 = PARSE_SIGNED(process_buf, 3);

                /*
                 * a1:-45
                 *        ^
                 *        10
                 */
                t1 = PARSE_SIGNED(process_buf, 10);

                /*
                 * a2:+60
                 *               ^
                 *               17
                 */
                t2 = PARSE_SIGNED(process_buf, 17);

                /*
                 * a3:-75
                 *                      ^
                 *                      24
                 */
                t3 = PARSE_SIGNED(process_buf, 24);

                /*
                 * a4:+90
                 *                             ^
                 *                             31
                 */
                t4 = PARSE_SIGNED(process_buf, 31);

                /*
                 * a5:-10
                 *                                    ^
                 *                                    38
                 */
                t5 = PARSE_SIGNED(process_buf, 38);


                /*
                 * 更新PWM使用的6路电流变量
                 */
                __disable_irq();

                a0_amp = t0;
                a1_amp = t1;
                a2_amp = t2;
                a3_amp = t3;
                a4_amp = t4;
                a5_amp = t5;

                __enable_irq();


                /*
                 * 第一次收到有效指令后启动PWM
                 */
                if (!pwm_started)
                {
                    Start_PWM();
                    pwm_started = 1;
                }


                /*
                 * =========================================================
                 * 回传数据
                 * =========================================================
                 *
                 * 这里暂时保持你原来的12字节回传协议：
                 *
                 * a0 -> 两位数字
                 * a1 -> 两位数字
                 * ...
                 *
                 * 注意：这个回传协议不包含正负号。
                 *
                 * 如果上位机需要读取正负号，
                 * 后面可以改成18字节有符号回传。
                 */

                tx_buf[0]  = process_buf[4];
                tx_buf[1]  = process_buf[5];

                tx_buf[2]  = process_buf[11];
                tx_buf[3]  = process_buf[12];

                tx_buf[4]  = process_buf[18];
                tx_buf[5]  = process_buf[19];

                tx_buf[6]  = process_buf[25];
                tx_buf[7]  = process_buf[26];

                tx_buf[8]  = process_buf[32];
                tx_buf[9]  = process_buf[33];

                tx_buf[10] = process_buf[39];
                tx_buf[11] = process_buf[40];


                /*
                 * UART空闲后发送12字节
                 */
                UARTTelemetry_QueueEcho(tx_buf, 12U);
            }
        }
    }

    /* Foreground-only UART owner; ADC continues at 500 Hz. */
    UARTTelemetry_Poll();
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

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(
      PWR_REGULATOR_VOLTAGE_SCALE1
  );

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState =
      RCC_HSE_ON;

  RCC_OscInitStruct.PLL.PLLState =
      RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource =
      RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource =
      RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider =
      RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider =
      RCC_HCLK_DIV4;

  RCC_ClkInitStruct.APB2CLKDivider =
      RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(
          &RCC_ClkInitStruct,
          FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */


/**
 * @brief 启动PWM
 */
void Start_PWM(void)
{
    /*
     * 开启光耦
     */
    HAL_GPIO_WritePin(
        GPIOF,
        GPIO_PIN_2 |
        GPIO_PIN_7 |
        GPIO_PIN_10,
        GPIO_PIN_SET
    );


    /*
     * 启动TIM2中断
     */
    HAL_TIM_Base_Start_IT(&htim2);


    /*
     * TIM1 CH1/CH2/CH3
     */
    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_1
    );

    HAL_TIMEx_PWMN_Start(
        &htim1,
        TIM_CHANNEL_1
    );

    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_2
    );

    HAL_TIMEx_PWMN_Start(
        &htim1,
        TIM_CHANNEL_2
    );

    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_3
    );

    HAL_TIMEx_PWMN_Start(
        &htim1,
        TIM_CHANNEL_3
    );


    /*
     * TIM8 CH1/CH2/CH3
     */
    HAL_TIM_PWM_Start(
        &htim8,
        TIM_CHANNEL_1
    );

    HAL_TIMEx_PWMN_Start(
        &htim8,
        TIM_CHANNEL_1
    );

    HAL_TIM_PWM_Start(
        &htim8,
        TIM_CHANNEL_2
    );

    HAL_TIMEx_PWMN_Start(
        &htim8,
        TIM_CHANNEL_2
    );

    HAL_TIM_PWM_Start(
        &htim8,
        TIM_CHANNEL_3
    );

    HAL_TIMEx_PWMN_Start(
        &htim8,
        TIM_CHANNEL_3
    );
}


/**
 * @brief 停止PWM
 */
void Stop_PWM(void)
{
    /*
     * 关闭光耦
     */
    HAL_GPIO_WritePin(
        GPIOF,
        GPIO_PIN_2 |
        GPIO_PIN_7 |
        GPIO_PIN_10,
        GPIO_PIN_RESET
    );


    /*
     * 停止TIM2
     */
    HAL_TIM_Base_Stop_IT(&htim2);


    /*
     * TIM1
     */
    HAL_TIM_PWM_Stop(
        &htim1,
        TIM_CHANNEL_1
    );

    HAL_TIMEx_PWMN_Stop(
        &htim1,
        TIM_CHANNEL_1
    );

    HAL_TIM_PWM_Stop(
        &htim1,
        TIM_CHANNEL_2
    );

    HAL_TIMEx_PWMN_Stop(
        &htim1,
        TIM_CHANNEL_2
    );

    HAL_TIM_PWM_Stop(
        &htim1,
        TIM_CHANNEL_3
    );

    HAL_TIMEx_PWMN_Stop(
        &htim1,
        TIM_CHANNEL_3
    );


    /*
     * TIM8
     */
    HAL_TIM_PWM_Stop(
        &htim8,
        TIM_CHANNEL_1
    );

    HAL_TIMEx_PWMN_Stop(
        &htim8,
        TIM_CHANNEL_1
    );

    HAL_TIM_PWM_Stop(
        &htim8,
        TIM_CHANNEL_2
    );

    HAL_TIMEx_PWMN_Stop(
        &htim8,
        TIM_CHANNEL_2
    );

    HAL_TIM_PWM_Stop(
        &htim8,
        TIM_CHANNEL_3
    );

    HAL_TIMEx_PWMN_Stop(
        &htim8,
        TIM_CHANNEL_3
    );
}


/**
 * @brief UART DMA + IDLE接收回调
 */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size
)
{
    if (huart == &huart1)
    {
        /*
         * 防止超过缓冲区
         */
        if (Size > RX_BUF_SIZE)
        {
            Size = RX_BUF_SIZE;
        }


        /*
         * ==========================================================
         * 关键修改：
         *
         * DMA当前使用rx_buf。
         * 收到一帧后立即复制到process_buf，
         * 然后DMA可以继续使用rx_buf。
         *
         * 这样主循环读取process_buf时，
         * 不会被下一帧DMA覆盖。
         * ==========================================================
         */

        memcpy(
            process_buf,
            rx_buf,
            Size
        );


        /*
         * 保存本次帧长度
         */
        rx_len = Size;


        /*
         * 通知主循环
         */
        rx_done = 1;


        /*
         * 重新启动DMA接收
         */
        HAL_UARTEx_ReceiveToIdle_DMA(
            &huart1,
            rx_buf,
            RX_BUF_SIZE
        );


        /*
         * 禁止半传输中断
         */
        __HAL_DMA_DISABLE_IT(
            huart1.hdmarx,
            DMA_IT_HT
        );
    }
}


/**
 * @brief TIM周期中断
 *
 * TIM2负责更新6路PWM占空比
 */
void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim
)
{
    if (htim == &htim2)
    {
        /*
         * TIM1：
         *
         * CH1 -> a0
         * CH2 -> a1
         * CH3 -> a2
         *
         * TIM8：
         *
         * CH1 -> a3
         * CH2 -> a4
         * CH3 -> a5
         *
         *
         * 中心值：
         *      4200
         *
         * 电流控制：
         *      每1单位 -> 42
         *
         * 因此：
         *
         * +30 -> 4200 + 30*42
         * -30 -> 4200 - 30*42
         */

        TIM1->CCR1 =
            4200 + a0_amp * 42;

        TIM1->CCR2 =
            4200 + a1_amp * 42;

        TIM1->CCR3 =
            4200 + a2_amp * 42;

        TIM8->CCR1 =
            4200 + a3_amp * 42;

        TIM8->CCR2 =
            4200 + a4_amp * 42;

        TIM8->CCR3 =
            4200 + a5_amp * 42;
    }
}


/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();

  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line
  */
void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */