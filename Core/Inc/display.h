
#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac);

void display_graph();

void reset_values();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void increase_brightness();

void decrease_brightness();

#endif /* INC_DISPLAY_H_ */
