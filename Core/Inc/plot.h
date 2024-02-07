
#include "ili9341_gfx.h"

void init_ili9341_lcd(SPI_HandleTypeDef *spi_hal);

void fill_screen();

uint16_t translate_y(uint16_t value, uint16_t max_value);

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
