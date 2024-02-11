#include "ili9341_gfx.h"

void init_ili9341_lcd(SPI_HandleTypeDef *spi_hal);

void print_screen_details(UART_HandleTypeDef *huart);

void clear_screen();

uint16_t translate_y_(uint16_t value);

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
