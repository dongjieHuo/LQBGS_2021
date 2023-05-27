#include "LED.h"

uint16_t LED_ALL = 0xffff;

void LEDx_on(uint16_t n)
{
	LED_ALL &= 0xfeff <<(n-1) | 0xfeff>>(16-(n-1));
	GPIOC->ODR = LED_ALL;
	HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);
}

void LEDx_off(uint16_t n)
{
	LED_ALL |= 0x0100 <<(n-1) | 0x0100>>(16-(n-1));
	GPIOC->ODR = LED_ALL;
	HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);
}

void LED_Close_All(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);
}
