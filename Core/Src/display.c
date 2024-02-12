#include "main.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>
#include "display.h"

#define BUFFER_SIZE 2500

TIM_HandleTypeDef* timer_hal;

UART_HandleTypeDef* uart_hal;

ili9341_t* ili9341_lcd;

uint16_t dma_values[0];

uint16_t raw_values[BUFFER_SIZE];

uint16_t fill_index = 0;

uint16_t draw_index = 0;

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
    UART_HandleTypeDef* uart) {
  timer_hal = timer;
  uart_hal = uart;
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

  char message[40];
  sprintf(message, "Screen: %d x %d\r\n", ili9341_lcd->screen_size.width, ili9341_lcd->screen_size.height);
  HAL_UART_Transmit(uart_hal, (uint8_t*) message, strlen(message), 100);
  sprintf(message, "Orientation: %d\r\n", ili9341_lcd->orientation);
  HAL_UART_Transmit(uart_hal, (uint8_t*) message, strlen(message), 100);

  ili9341_fill_screen(ili9341_lcd, ILI9341_BLACK);

  ili9341_text_attr_t attr;
  attr.bg_color = ILI9341_BLUE;
  attr.fg_color = ILI9341_WHITE;
  attr.font = &ili9341_font_11x18;
  attr.origin_x = 50;
  attr.origin_y = 50;
  ili9341_draw_string(ili9341_lcd, attr, "Hello");

  ili9341_draw_line(ili9341_lcd, ILI9341_RED, 0, 239, 319, 0);

  HAL_ADC_Start_DMA(adc, (uint32_t*) dma_values, 1);
}


void printToUart(UART_HandleTypeDef *huart, char *msg) {
  HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}

uint16_t translate_y(uint16_t value) {
//  return 239 - value * 0.0589;
  return 239 - (value - 2000) * 0.0589;
}

void display_graph() {
  if (fill_index > draw_index) {
    int x = draw_index % ili9341_lcd->screen_size.width;
    ili9341_draw_line(ili9341_lcd, ILI9341_BLACK, x, 0, x, 239);
    if (x == 0) {
      ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(raw_values[draw_index]));
    }
    else {
      ili9341_draw_line(ili9341_lcd, ILI9341_LIGHTGREY, x - 1, translate_y(raw_values[draw_index - 1]), x, translate_y(raw_values[draw_index]));
    }
    draw_index++;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (fill_index >= BUFFER_SIZE) {
    return;
  }
  raw_values[fill_index] = dma_values[0];
  char message[40];
  sprintf(message, "%d -> %d\r\n", fill_index, translate_y(raw_values[fill_index]));
  printToUart(uart_hal, message);
  fill_index++;
}


