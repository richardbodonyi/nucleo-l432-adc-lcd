#include "main.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "display.h"
#include "bt.h"

#define BUFFER_SIZE 2500

TIM_HandleTypeDef* timer_hal;

UART_HandleTypeDef* uart_hal;

ili9341_t* ili9341_lcd;

uint16_t dma_values[0];

uint16_t raw_values[BUFFER_SIZE];

uint32_t time_buffer[BUFFER_SIZE];

uint16_t fill_index = 0;

uint16_t draw_index = 0;

uint16_t min_y = 3650;

uint16_t max_y = 4095;

bool data_sent = false;

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
    UART_HandleTypeDef* uart,
    UART_HandleTypeDef* uart_bt) {
  timer_hal = timer;
  uart_hal = uart;

  init_bt(uart_bt);
  bt_send("max voltage: 3300\r\n");
  bt_send("data rate: 120\r\n");

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

  HAL_ADC_Start_DMA(adc, (uint32_t*) dma_values, 1);
}


void printToUart(UART_HandleTypeDef *huart, char *msg) {
  HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}

uint16_t translate_y(uint16_t value) {
  return ili9341_lcd->screen_size.height - 1 - (value - min_y) * (float) ili9341_lcd->screen_size.height / (max_y - min_y);
}

void display_graph() {
  if (fill_index > draw_index) {
    int x = draw_index % ili9341_lcd->screen_size.width;
    ili9341_draw_line(ili9341_lcd, ILI9341_BLACK, x, 0, x, ili9341_lcd->screen_size.height - 1);
    if (x == 0) {
      ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(raw_values[draw_index]));
    }
    else {
      ili9341_draw_line(ili9341_lcd, ILI9341_LIGHTGREY, x - 1, translate_y(raw_values[draw_index - 1]), x, translate_y(raw_values[draw_index]));
    }
//    char message[100];
//    sprintf(message, "%lu: %d\r\n", time_buffer[draw_index], raw_values[draw_index]);
//    bt_send(message);
    draw_index++;
  }
  else if (fill_index == BUFFER_SIZE && !data_sent) {
    char message[100];
    for (int i = 0; i < BUFFER_SIZE; i++) {
      sprintf(message, "%lu: %d\r\n", time_buffer[i], raw_values[i]);
      bt_send(message);
    }
    bt_send("setting state to 0\r\n");
    data_sent = true;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (fill_index >= BUFFER_SIZE) {
    return;
  }
  raw_values[fill_index] = dma_values[0];
  time_buffer[fill_index] = HAL_GetTick();
//  char message[40];
//  sprintf(message, "%d\r\n", raw_values[fill_index]);
//  printToUart(uart_hal, message);
  fill_index++;
}


