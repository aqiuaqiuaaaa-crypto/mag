/**
 ****************************************************************************************************
 * @file        exti.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       外部中断 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台：正点原子 F407电机开发板
 * 在线视频：www.yuanzige.com
 * 技术论坛：http://www.openedv.com/forum.php
 * 公司网址：www.alientek.com
 * 购买地址：zhengdianyuanzi.tmall.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "../SYSTEM/sys/sys.h"
#include "../SYSTEM/delay/delay.h"
#include "../BSP/LED/led.h"
#include "../BSP/KEY/key.h"
#include "../BSP/EXTI/exti.h"
#include "tim.h"

extern int fre;

/**
 * @brief       KEY0 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY0_INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(KEY0_INT_GPIO_PIN);         /* 调用中断处理公用函数 清除KEY0所在中断线 的中断标志位 */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY0_INT_GPIO_PIN);         /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       KEY1 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY1_INT_IRQHandler(void)
{ 
    HAL_GPIO_EXTI_IRQHandler(KEY1_INT_GPIO_PIN);         /* 调用中断处理公用函数 清除KEY1所在中断线 的中断标志位，中断下半部在HAL_GPIO_EXTI_Callback执行 */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);         /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       KEY2 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY2_INT_IRQHandler(void)
{ 
    HAL_GPIO_EXTI_IRQHandler(KEY2_INT_GPIO_PIN);        /* 调用中断处理公用函数 清除KEY2所在中断线 的中断标志位，中断下半部在HAL_GPIO_EXTI_Callback执行 */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY2_INT_GPIO_PIN);        /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       中断服务程序中需要做的事情
                在HAL库中所有的外部中断服务函数都会调用此函数
 * @param       GPIO_Pin:中断引脚号
 * @retval      无
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(10);      /* 消抖 */
    
    switch(GPIO_Pin)
    {
        
        case KEY0_INT_GPIO_PIN:
            if (KEY0 == 0)
            {
                /*
                int arr =168000000/(400*0.5)-1;
                LED0_TOGGLE();  
                HAL_TIM_Base_Stop_IT(&htim2); //关闭定时器2
                HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_1);//开启四路pwm波
                HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_2);
                HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_2);
                TIM2->ARR=arr;
                HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
                HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);//开启四路pwm波
                HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
                HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
                HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
							*/
            }
            break;

        case KEY1_INT_GPIO_PIN:
            if (KEY1 == 0)
            {/*
               fre += 1;
                int arr =168000000/(400*fre)-1;
                LED0_TOGGLE();  // LED0 状态取反 
                HAL_TIM_Base_Stop_IT(&htim2); //关闭定时器2
                HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_1);//开启四路pwm波
                HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop (&htim1,TIM_CHANNEL_2);
                HAL_TIMEx_PWMN_Stop (&htim1,TIM_CHANNEL_2);
                TIM2->ARR=arr;
                HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
                HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);//开启四路pwm波
                HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
                HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
                HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
							*/
            }
            break;

        case KEY2_INT_GPIO_PIN:
            if (KEY2 == 0)
            {
                LED0_TOGGLE();  /* LED0 状态取反 */ 
                LED1_TOGGLE();  /* LED1 状态取反 */
            }
            break;

        default : break;
    }
}

/**
 * @brief       外部中断初始化程序
 * @param       无
 * @retval      无
 */
void extix_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    key_init();
    gpio_init_struct.Pin = KEY0_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* 下降沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY0_INT_GPIO_PORT, &gpio_init_struct);    /* KEY0配置为下降沿触发中断 */

    gpio_init_struct.Pin = KEY1_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* 下降沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY1_INT_GPIO_PORT, &gpio_init_struct);    /* KEY1配置为下降沿触发中断 */
    
    gpio_init_struct.Pin = KEY2_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* 下降沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY2_INT_GPIO_PORT, &gpio_init_struct);    /* KEY2配置为下降沿触发中断 */
    

    HAL_NVIC_SetPriority(KEY0_INT_IRQn, 0, 2);               /* 抢占0，子优先级2 */
    HAL_NVIC_EnableIRQ(KEY0_INT_IRQn);                       /* 使能中断线1 */

    HAL_NVIC_SetPriority(KEY1_INT_IRQn, 1, 2);               /* 抢占1，子优先级2 */
    HAL_NVIC_EnableIRQ(KEY1_INT_IRQn);                       /* 使能中断线15 */
    
    HAL_NVIC_SetPriority(KEY2_INT_IRQn, 2, 2);               /* 抢占2，子优先级2 */
    HAL_NVIC_EnableIRQ(KEY2_INT_IRQn);                       /* 使能中断线15 */

}












