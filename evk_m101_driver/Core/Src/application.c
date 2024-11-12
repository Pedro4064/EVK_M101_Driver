#include "application.h"
#include "gpio.h"
#include "tim.h"

#define SAMPLING_TIM htim6

void ApplicationMain(void){

    HAL_TIM_Base_Start_IT(&SAMPLING_TIM);

    while (1)
    {
    }
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

}