#include "main.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "display.h"
#include "ad_header.h"
#include "stm32l4xx_hal_dac.h"

#define BUFFER_SIZE 2000

DAC_HandleTypeDef* hdac_hal;

TIM_HandleTypeDef* timer_hal;

ili9341_t* ili9341_lcd;

uint16_t dma_values[0];

uint16_t raw_values[BUFFER_SIZE];

uint32_t time_buffer[BUFFER_SIZE];

uint16_t fill_index = 0;

uint16_t draw_index = 0;

uint16_t min_y = 3650;

uint16_t max_y = 3900;

uint8_t lcd_brightness = 130;

bool active = false;

const uint8_t MIN_BRIGHTNESS = 80, MAX_BRIGHTNESS = 250, BRIGHTNESS_STEP = 10;

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac) {
  hdac_hal = hdac;
  HAL_DAC_Start(hdac_hal, DAC_CHANNEL_1);
  HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
  timer_hal = timer;
  ili9341_lcd = ili9341_new(
          spi,
          TFT_RESET_GPIO_Port, TFT_RESET_Pin,
          TFT_CS_GPIO_Port,    TFT_CS_Pin,
          TFT_DC_GPIO_Port,    TFT_DC_Pin,
          isoLandscape,
          TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
          TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
          itsSupported,
          itnNormalized);
  ili9341_spi_tft_select(ili9341_lcd);
  ili9341_fill_screen(ili9341_lcd, ILI9341_BLACK);

  ili9341_text_attr_t attr;
  attr.bg_color = ILI9341_BLACK;
  attr.fg_color = ILI9341_LIGHTGREY;
  attr.font = &ili9341_font_16x26;
  attr.origin_x = 120;
  attr.origin_y = 100;
  ili9341_draw_string(ili9341_lcd, attr, "ECG");
  enableAD();

  HAL_ADC_Start_DMA(adc, (uint32_t*) dma_values, 1);
}

uint16_t translate_y(uint16_t value) {
  return ili9341_lcd->screen_size.height - 1 - (value - min_y) * (float) ili9341_lcd->screen_size.height / (max_y - min_y);
}

void display_graph() {
  if (fill_index > draw_index) {
    active = true;
    int x = draw_index % ili9341_lcd->screen_size.width;
    ili9341_draw_line(ili9341_lcd, ILI9341_BLACK, x, 0, x, ili9341_lcd->screen_size.height - 1);
    if (x == 0) {
      ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(raw_values[draw_index]));
    }
    else {
      ili9341_draw_line(ili9341_lcd, ILI9341_LIGHTGREY, x - 1, translate_y(raw_values[draw_index - 1]), x, translate_y(raw_values[draw_index]));
    }
    draw_index++;
  }
  else if (fill_index == BUFFER_SIZE && active) {
	  // shutdown();
  }
}

void reset_values() {
	disableAD();
	fill_index = 0;
	draw_index = 0;
	active = false;
}

void increase_brightness() {
	lcd_brightness += BRIGHTNESS_STEP;
	if (lcd_brightness > MAX_BRIGHTNESS) {
		lcd_brightness = MAX_BRIGHTNESS;
	}
	HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
}

void decrease_brightness() {
	lcd_brightness -= BRIGHTNESS_STEP;
	if (lcd_brightness < MIN_BRIGHTNESS) {
		lcd_brightness = MIN_BRIGHTNESS;
	}
	HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (fill_index >= BUFFER_SIZE) {
    return;
  }
  raw_values[fill_index] = dma_values[0];
  time_buffer[fill_index] = HAL_GetTick();
  fill_index++;
}


