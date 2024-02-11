#include "main.h"
#include "plot.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>

ili9341_t* ili9341_lcd_;

void init_ili9341_lcd(SPI_HandleTypeDef *spi_hal) {
  ili9341_lcd_ = ili9341_new(
        spi_hal,
        TFT_RESET_GPIO_Port, TFT_RESET_Pin,
        TFT_CS_GPIO_Port,    TFT_CS_Pin,
        TFT_DC_GPIO_Port,    TFT_DC_Pin,
        isoLandscape,
        TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
        TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
        itsSupported,
        itnNormalized);
  ili9341_spi_tft_select(ili9341_lcd_);
}

void wait() {
  while (HAL_DMA_STATE_BUSY == HAL_DMA_GetState(ili9341_lcd_->spi_hal->hdmatx))
      { continue; }
}

void print_screen_details(UART_HandleTypeDef *huart) {
  char message[40];
  sprintf(message, "Screen: %d x %d\r\n", ili9341_lcd_->screen_size.width, ili9341_lcd_->screen_size.height);
  HAL_UART_Transmit(huart, (uint8_t*) message, strlen(message), 100);
  sprintf(message, "Orientation: %d\r\n", ili9341_lcd_->orientation);
  HAL_UART_Transmit(huart, (uint8_t*) message, strlen(message), 100);
}

void clear_screen() {
  ili9341_fill_screen(ili9341_lcd_, ILI9341_BLACK);
  ili9341_text_attr_t attr;
  attr.bg_color = ILI9341_BLUE;
  attr.fg_color = ILI9341_WHITE;
  attr.font = &ili9341_font_16x26;
  attr.origin_x = 100;
  attr.origin_y = 50;
  ili9341_draw_string(ili9341_lcd_, attr, "Hello");
}

uint16_t translate_y_(uint16_t value) {
  return value * 0.05896;
}

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  ili9341_draw_line(ili9341_lcd_, ILI9341_LIGHTGREY, x0, y0, x1, y1);
}

