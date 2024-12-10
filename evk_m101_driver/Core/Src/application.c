#include "m10gnss_driver.h"
#include "application.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"

#define SAMPLING_TIM htim14

#define IDLE_GNSS 0
#define SAMPLE_GNSS 1
#define PARSE_GNSS 2

m10_gnss gnss_module = {
                            .i2c_address = I2C_ADDRESS,
                            .i2c_handle = &hi2c1
                        };

char gnss_status = IDLE_GNSS;

void ApplicationMain(void){

    M10GnssDriverInit(&gnss_module);
    HAL_TIM_Base_Start_IT(&SAMPLING_TIM);

    while (1){
        if(gnss_status == SAMPLE_GNSS){
            M10GnssDriverReadStreamBuffer();
            gnss_status = IDLE_GNSS;
        }
        else if(gnss_status == PARSE_GNSS){
            M10GnssDriverHandleBufferStream();
            gnss_status = IDLE_GNSS;
        }
    }
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    // if(gnss_status != IDLE_GNSS)
        // return;
    gnss_status = SAMPLE_GNSS;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    // if(gnss_status != IDLE_GNSS)
        // return;
    gnss_status = PARSE_GNSS;
}