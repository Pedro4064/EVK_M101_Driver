#include "m10gnss_driver.h"
#include "application.h"
#include "gpio.h"
// #include "tim.h"
#include "i2c.h"

#define SAMPLING_TIM htim6
m10_gnss gnss_module = {
                            .i2c_address = I2C_ADDRESS,
                            .i2c_handle = &hi2c1
                        };

int stat = 0;

void ApplicationMain(void){

    M10GnssDriverInit(&gnss_module);
    // HAL_TIM_Base_Start_IT(&SAMPLING_TIM);

    while (1)
    {
        stat = 0;
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        M10GnssDriverReadData();
        // HAL_Delay(100);
    }
    
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     stat = 1;
// }