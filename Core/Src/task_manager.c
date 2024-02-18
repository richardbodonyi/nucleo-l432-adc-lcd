#include <task_manager.h>
#include "display.h"

void init_tasks(
    SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
    UART_HandleTypeDef* uart) {

  init_display(spi, timer, adc, uart);
  HAL_TIM_Base_Start_IT(timer);
}

void manage_tasks() {
  display_graph();
}
