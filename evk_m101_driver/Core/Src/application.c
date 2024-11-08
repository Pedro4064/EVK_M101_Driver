#include "application.h"
#include "gpio.h"

void ApplicationMain(void){

    while (1)
    {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        HAL_Delay(1000);
    }
    
}