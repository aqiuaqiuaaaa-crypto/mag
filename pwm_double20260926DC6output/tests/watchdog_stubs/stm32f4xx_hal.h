#ifndef WATCHDOG_TEST_HAL_H
#define WATCHDOG_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { volatile uint32_t ODR; } GPIO_TypeDef;
typedef struct {
  volatile uint32_t CCR1, CCR2, CCR3, ARR, SR;
} TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;

extern GPIO_TypeDef test_gpiof;
extern TIM_TypeDef test_tim1, test_tim2, test_tim8;
extern TIM_HandleTypeDef htim1, htim2, htim8;
extern UART_HandleTypeDef huart1;
#define GPIOF (&test_gpiof)
#define TIM1 (&test_tim1)
#define TIM2 (&test_tim2)
#define TIM8 (&test_tim8)
#define GPIO_PIN_2 (1U << 2U)
#define GPIO_PIN_7 (1U << 7U)
#define GPIO_PIN_10 (1U << 10U)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define DMA_IT_HT 16U
#define RESET 0U
#define TIM_FLAG_UPDATE 1U
#define __HAL_TIM_GET_FLAG(timer, flag) ((timer)->Instance->SR & (flag))
/* Model the production HAL's write-zero-to-clear register semantics. */
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((timer)->Instance->SR &= ~(flag))

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t value);
void __DMB(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *buffer, uint16_t size);
void test_disable_dma_it(DMA_HandleTypeDef *dma, uint32_t interrupt);
#define __HAL_DMA_DISABLE_IT(dma, interrupt) test_disable_dma_it(dma, interrupt)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
HAL_StatusTypeDef CurrentSense_Start(void);
void UARTTelemetry_QueueEcho(const uint8_t *data, uint16_t size);
void UARTTelemetry_Poll(void);
#endif
