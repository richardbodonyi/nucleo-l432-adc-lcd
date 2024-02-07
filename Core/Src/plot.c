#include "main.h"
#include "plot.h"
#include "ili9341_gfx.h"

ili9341_t* ili9341_lcd;

void init_ili9341_lcd(SPI_HandleTypeDef *spi_hal) {
  ili9341_lcd = ili9341_new(
        spi_hal,
        TFT_RESET_GPIO_Port, TFT_RESET_Pin,
        TFT_CS_GPIO_Port,    TFT_CS_Pin,
        TFT_DC_GPIO_Port,    TFT_DC_Pin,
        isoLandscape,
        NULL,  0,
        NULL, 0,
        itsSupported,
        itnNormalized);
}

void fill_screen() {
  ili9341_fill_screen(ili9341_lcd, ILI9341_BLACK);
}

uint16_t translate_y(uint16_t value, uint16_t max_value) {
  return 240 - 240 * value / max_value;
}

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  ili9341_draw_line(ili9341_lcd, ILI9341_LIGHTGREY, x0, y0, x1, y1);
}

