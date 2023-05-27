/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum Keywork
{
    NULL_KEY=0,
    Short_Key=1,
    Long_Key=2
}MyKeyworkTypeDef;

typedef enum Keystate
{
    Key_Check=0,
    Key_Confirm=1,
    Key_Release=2
}MyKeystateTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//LCD变量定义
uint8_t str[30];
uint8_t LCD_State;
//Tim
uint8_t Testting_flag=0;//Tim定时器采集变量，0为PA6，1为PA7
float PA6_Duty,PA6_D,PA6_F;
float PA7_Duty,PA7_D,PA7_F;
float PA1_D,PA1_F;
//adc变量
float adc_val;
uint8_t light_state,light_state_old;//0暗1亮
//逻辑变量
float a,b,f,ax,bx;
float a_old,b_old;
float pax=10,pbx=10,pf=1000;
uint8_t mode_state;
float a_store[5],a_rk[5];
float b_store[5],b_rk[5];
//按键变量
MyKeyworkTypeDef Key1_Work,Key2_Work,Key3_Work,Key4_Work;
MyKeystateTypeDef Key1_State,Key2_State,Key3_State,Key4_State;
//串口
uint8_t Rx_Buf[1000],Tx_Buf[1000];
uint16_t RxSize = 1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Sys_init(void)
{
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Init(&hadc2);
    LED_Close_All();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf, RxSize);
}
void shift_array(float arr[],float n)
{
    int i;
    for(i=4;i>0;i--)
    {
        arr[i]=arr[i-1];
    }
    arr[0] = n;
}

void Pluse_Get(void)//脉冲两路采集
{
    float temp;
	MX_TIM3_Init_PA6();
	Testting_flag=0;
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);		  //开启输入捕获通道
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);		  //开启输入捕获通道
	HAL_TIM_Base_Start(&htim3); //定时器开启	
	HAL_Delay(20); //留下时间采集
	HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);		  //关闭输入捕获通道
	HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2);		  //关闭输入捕获通道
	HAL_TIM_Base_Stop(&htim3);  //关闭定时器
    if(PA6_Duty < 0.1) a =0;
    else if(PA6_Duty > 0.9) a = 180;
    else a = 225*PA6_Duty - 22.5;
    //数组中所有数据往后移位
    shift_array(a_store,a);//将a传入数组的第一位
    shift_array(a_rk,a);//将a传入数组的第一位
    ax = a - a_old;
    a_old = a;
	
	MX_TIM3_Init_PA7();
	Testting_flag=1;
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);		
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);		  
	HAL_TIM_Base_Start(&htim3); 
	HAL_Delay(20);
	HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);		  
	HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2);		  
	HAL_TIM_Base_Stop(&htim3);  
    if(PA7_Duty < 0.1) b =0;
    else if(PA7_Duty > 0.9) b = 90;
    else b = 112.5*PA7_Duty - 11.25;
    shift_array(b_store,b);//将b传入数组的第一位
    shift_array(b_rk,b);//将b传入数组的第一位
    bx = b-b_old;
    b_old = b;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		if(Testting_flag == 0) //PA6采集
		{
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				PA6_F = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1)+1;
				PA6_Duty = (float)PA6_D/PA6_F;
			}
			else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				PA6_D = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2)+1; 
			}
		}
		
		if(Testting_flag == 1) //PA7采集
		{
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				PA7_F = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2)+1;
				PA7_Duty = (float)PA7_D/PA7_F;
			}
			else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				PA7_D = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1)+1; 
			}
		}
	}
	if(htim == &htim2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			PA1_D = __HAL_TIM_GET_COMPARE(&htim2,TIM_CHANNEL_1)+1;
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			PA1_F = __HAL_TIM_GET_COMPARE(&htim2,TIM_CHANNEL_2)+1; 
            f = (float)1000000./PA1_F;
		}		
	}
}
void ADC_Func(void)
{
	HAL_ADC_Start(&hadc2);
	adc_val = HAL_ADC_GetValue(&hadc2)*3.3/4096;
    if(adc_val < 2) light_state = 0;
    else light_state = 1;
    if(mode_state == 1)//模式b下
    {
        if(light_state != light_state_old)  
        {
            Pluse_Get();
            mode_state = 0;
        }
    }
    light_state_old = light_state;
}
void LCD_Show(void)
{
    if(LCD_State == 0)
    {
        LCD_DisplayStringLine(Line1,(uint8_t *)"        DATA      ");
        sprintf(str,"   a:%.1f      ",a);
        LCD_DisplayStringLine(Line2,str);
        sprintf(str,"   b:%.1f      ",b);
        LCD_DisplayStringLine(Line3,str);
        sprintf(str,"   f:%.0fHz      ",f);
        LCD_DisplayStringLine(Line4,str);
        sprintf(str,"   ax:%.0f      ",ax);
        LCD_DisplayStringLine(Line6,str);
        sprintf(str,"   bx:%.0f      ",bx);
        LCD_DisplayStringLine(Line7,str);
        if(mode_state == 0) 
            LCD_DisplayStringLine(Line8,(uint8_t *)"   mode:A");
        else if(mode_state == 1)
            LCD_DisplayStringLine(Line8,(uint8_t *)"   mode:B");
    }
    else if(LCD_State == 1)
    {
        LCD_DisplayStringLine(Line1,(uint8_t *)"        PARA      ");
        sprintf(str,"   pax:%.0f      ",pax);
        LCD_DisplayStringLine(Line2,str);
        sprintf(str,"   pbx:%.0f      ",pbx);
        LCD_DisplayStringLine(Line3,str);
        sprintf(str,"   pf:%.0f       ",pf);
        LCD_DisplayStringLine(Line4,str);
        LCD_DisplayStringLine(Line5,(uint8_t *)"                 ");
        LCD_DisplayStringLine(Line6,(uint8_t *)"                 ");
        LCD_DisplayStringLine(Line7,(uint8_t *)"                 ");
        LCD_DisplayStringLine(Line8,(uint8_t *)"                 ");
    }
}
void Key_Scan(void)
{
    //Key1
    switch (Key1_State)
    {
        case Key_Check:
        {
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == GPIO_PIN_RESET)
            {
                Key1_State = Key_Confirm;
                break;
            }
        }
        case Key_Confirm:
        {
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == GPIO_PIN_RESET)
            {
                Key1_State = Key_Release;
                Key1_Work = Short_Key;
                break;
            }
            else
            {
                Key1_State = Key_Check;
                break;
            }
        }
        case Key_Release:
        {
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == GPIO_PIN_SET)
                Key1_State = Key_Check;
        }
    }
    //Key2
    switch(Key2_State)
    {
        case Key_Check:
        {
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == GPIO_PIN_RESET)
            {
                Key2_State = Key_Confirm;
                break;
            }
        }
        case Key_Confirm:
        {
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == GPIO_PIN_RESET)
            {
                Key2_State = Key_Release;
                Key2_Work = Short_Key;
                break;
            }
            else
            {
                Key2_State = Key_Check;
                break;
            }
        }
        case Key_Release:
        {
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == GPIO_PIN_SET)
                Key2_State = Key_Check;
        }
    }
    //key3
    switch(Key3_State)
    {
        case Key_Check:
        {
            if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin) == GPIO_PIN_RESET)
            {
                Key3_State = Key_Confirm;
                break;
            }
        }
        case Key_Confirm:
        {
            if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin) == GPIO_PIN_RESET)
            {
                Key3_State = Key_Release;
                Key3_Work = Short_Key;
                break;
            }
            else
            {
                Key3_State = Key_Check;
                break;
            }
        }
        case Key_Release:
        {
            if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin) == GPIO_PIN_SET)
                Key3_State = Key_Check;
        }
    }
    //key4
    switch(Key4_State)
    {
        case Key_Check:
        {
            if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin) == GPIO_PIN_RESET)
            {
                Key4_State = Key_Confirm;
                break;
            }
        }
        case Key_Confirm:
        {
            if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin) == GPIO_PIN_RESET)
            {
                Key4_State = Key_Release;
                Key4_Work = Short_Key;
                break;
            }
            else
            {
                Key4_State = Key_Check;
                break;
            }
        }
        case Key_Release:
        {
            if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin) == GPIO_PIN_SET)
                Key4_State = Key_Check;
        }
    }
}
void Key_event(void)
{
    switch (Key1_Work)
    {
        case Short_Key:
        {
            LCD_State ^=1;
            Key1_Work = NULL_KEY;
            break;
        }
    }
    switch (Key2_Work)
    {
        case Short_Key:
        {
            if(LCD_State == 1)
            {
                pax+=10;
                if(pax >60) pax = 10;
                pbx+=10;
                if(pbx >60) pbx = 10;
                Key2_Work = NULL_KEY;
                break;
            }
        }
    }
    switch (Key3_Work)
    {
        case Short_Key:
        {
            if(LCD_State == 1)
            {
                pf+=1000;
                if(pf>10000) pf =1000;
                Key3_Work = NULL_KEY;
                break;
            }
        }
    }
    switch (Key4_Work)
    {
        case Short_Key:
        {
            if(LCD_State == 0 && mode_state == 0)
            {
                mode_state = 1;
                Pluse_Get();
                Key4_Work = NULL_KEY;
                break;
            }
        }
    }
}
void LED_Func(void)
{
    if(a > pax) LEDx_on(1);
    else LEDx_off(1);
    if(b > pbx) LEDx_on(2);
    else LEDx_off(2);
    if(f > pf) LEDx_on(3);
    else LEDx_off(3);
    if(mode_state == 0) LEDx_on(4);
    else LEDx_off(4);
    if(a-b>80) LEDx_on(5);
    else LEDx_off(5);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart1)
    {
        if(Size == 2)
        {
            if(Rx_Buf[0] == 'a' && Rx_Buf[1] == '?')
            {
                sprintf(Tx_Buf,"a:%.1f\r\n",a);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));
            }
            if(Rx_Buf[0] == 'b' && Rx_Buf[1] == '?')
            {
                sprintf(Tx_Buf,"b:%.1f\r\n",b);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));                
            }
        }
        else if(Size == 3)
        {
            if(Rx_Buf[0] == 'a' && Rx_Buf[1] == 'a' && Rx_Buf[2] == '?')
            {
                sprintf(Tx_Buf,"%.1f-%.1f-%.1f-%.1f-%.1f\r\n",a_store[0],a_store[1],a_store[2],a_store[3],a_store[4]);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));
            }
            if(Rx_Buf[0] == 'b' && Rx_Buf[1] == 'b' && Rx_Buf[2] == '?')
            {
                sprintf(Tx_Buf,"%.1f-%.1f-%.1f-%.1f-%.1f\r\n",b_store[0],b_store[1],b_store[2],b_store[3],b_store[4]);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));
            }
            if(Rx_Buf[0] == 'q' && Rx_Buf[1] == 'a' && Rx_Buf[2] == '?')
            {
                for(uint8_t i=0;i<5;i++)//冒泡法排序
                {
                    for(uint8_t j=0;j<5-i-1;j++)
                    {
                        if(a_rk[j]>a_rk[j+1])
                        {
                            float temp = a_rk[j];
                            a_rk[j] = a_rk[j+1];
                            a_rk[j+1] = temp;
                        }
                    }
                }
                sprintf(Tx_Buf,"%.1f-%.1f-%.1f-%.1f-%.1f\r\n",a_rk[0],a_rk[1],a_rk[2],a_rk[3],a_rk[4]);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));
            }
            if(Rx_Buf[0] == 'q' && Rx_Buf[1] == 'b' && Rx_Buf[2] == '?')
            {
                for(uint8_t i=0;i<5;i++)
                {
                    for(uint8_t j=0;j<5-i-1;j++)
                    {
                        if(b_rk[j]>b_rk[j+1])
                        {
                            float temp = b_rk[j];
                            b_rk[j] = b_rk[j+1];
                            b_rk[j+1] = temp;
                        }
                    }
                }
                sprintf(Tx_Buf,"%.1f-%.1f-%.1f-%.1f-%.1f\r\n",b_rk[0],b_rk[1],b_rk[2],b_rk[3],b_rk[4]);
                HAL_UART_Transmit_IT(&huart1,Tx_Buf, strlen(Tx_Buf));
            }
        }
        else {
            HAL_UART_Transmit_IT(&huart1,(uint8_t *)"error\r\n", 5); 
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf, RxSize);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  Sys_init();
  u32 LCD_uwTick;
  u32 ADC_uwTick;
  u32 Key_uwTick;
  u32 LED_uwTick;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(uwTick - LCD_uwTick > 150)
	  {
		  LCD_uwTick = uwTick;
		  LCD_Show();
	  }
	  if(uwTick - ADC_uwTick > 100)
	  {
		  ADC_uwTick = uwTick;
		  ADC_Func();
	  }
      if(uwTick - Key_uwTick > 10)
      {
          Key_uwTick = uwTick;
          Key_Scan();
          Key_event();
      }
      if(uwTick - LED_uwTick > 100)
      {
          LED_uwTick = uwTick;
          LED_Func();
      }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
