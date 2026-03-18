//
// Created by 14717 on 2025/8/27.
//

#include "bsp_LED.h"

#include "main.h"
#include "stm32f4xx_hal_gpio.h"

void LED_RED_SET(void) {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}
void LED_RED_RESET(void) {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}
void LED_RED_Toggle(void) {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
}
void LED_BLUE_SET(void) {
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}
void LED_BLUE_RESET(void) {
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
}
void LED_BLUE_Toggle(void) {
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}
void LED_GREEN_SET(void) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
}
void LED_GREEN_RESET(void){
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
}
void LED_GREEN_Toggle(void){
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}