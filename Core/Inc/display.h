
#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
    UART_HandleTypeDef* uart);

void display_graph();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_DISPLAY_H_ */
