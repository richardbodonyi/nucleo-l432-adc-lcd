#include "task_manager.h"
#include "display.h"

void init_tasks(
    SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
    UART_HandleTypeDef* uart,
    UART_HandleTypeDef* uart_bt) {

  init_display(spi, timer, adc, uart, uart_bt);
  HAL_TIM_Base_Start_IT(timer);
}

void manage_tasks() {
  display_graph();
}
