#ifndef INC_TASK_MANAGER_H_
#define INC_TASK_MANAGER_H_

#include "main.h"

void init_tasks(
    SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc);

void manage_tasks();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TASK_MANAGER_H_ */
