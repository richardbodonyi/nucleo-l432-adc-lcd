#include "main.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "display.h"
#include "ad_header.h"
#include "stm32l4xx_hal_dac.h"

#define VERSION "1.0"

#define BUFFER_SIZE 2000

#define MENU_SIZE 3

char* MENU_TEXTS[] = {"Szunet", "Hang", "Vissza"};

typedef enum {
  MEASURE = 0,
  MENU
} T_Mode;

typedef struct {
  uint8_t selected;
} T_Menu;

DAC_HandleTypeDef* hdac_hal;

TIM_HandleTypeDef* timer_hal;

ili9341_t* ili9341_lcd;

uint16_t dma_values[0];

uint16_t raw_values[BUFFER_SIZE];

uint32_t time_buffer[BUFFER_SIZE];

uint16_t fill_index = 0;

uint16_t draw_index = 0;

uint16_t min_y = 0;

uint16_t max_y = 5000;

uint8_t lcd_brightness = 130;

bool active = false, enabled = true;

T_Mode mode = MEASURE;

T_Menu menu;

const uint8_t MIN_BRIGHTNESS = 80, MAX_BRIGHTNESS = 250, BRIGHTNESS_STEP = 10;

uint16_t debug_line_y = 100;

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
  attr.origin_x = 60;
  attr.origin_y = 100;
  ili9341_draw_string(ili9341_lcd, attr, "EKG MONITOR");
  attr.font = &ili9341_font_11x18;
  attr.origin_x = 120;
  attr.origin_y = 150;
  ili9341_draw_string(ili9341_lcd, attr, VERSION);
  enableAD();

  HAL_ADC_Start_DMA(adc, (uint32_t*) dma_values, 1);
}

uint16_t translate_y(uint16_t value) {
  return ili9341_lcd->screen_size.height - 1 - (value - min_y) * (float) ili9341_lcd->screen_size.height / (max_y - min_y);
}

void draw_menu() {
  // return;
  uint8_t x = 10, y = 10;
  for (uint8_t i = 0; i < MENU_SIZE; i++) {
    ili9341_text_attr_t attr;
    attr.bg_color = menu.selected == i ? ILI9341_BLUE : ILI9341_DARKGREY;
    attr.fg_color = ILI9341_LIGHTGREY;
    attr.font = &ili9341_font_11x18;
    attr.origin_x = x;
    attr.origin_y = y + i * 18;
    ili9341_draw_string(ili9341_lcd, attr, MENU_TEXTS[i]);
  }
}

void display_graph() {
  if (enabled) {
    while (fill_index > draw_index) {
      active = true;
      int x = draw_index % ili9341_lcd->screen_size.width;
      ili9341_draw_line(ili9341_lcd, ILI9341_BLACK, x, 0, x, ili9341_lcd->screen_size.height - 1);
      if (x == 0) {
        ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(raw_values[draw_index]));
        ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(raw_values[draw_index]));
      }
      else {
        ili9341_draw_line(ili9341_lcd, ILI9341_LIGHTGREY, x - 1, translate_y(raw_values[draw_index - 1]), x, translate_y(raw_values[draw_index]));
      }
      draw_index++;
    }
    ili9341_draw_line(ili9341_lcd, ILI9341_CYAN, 0, debug_line_y, 319, debug_line_y);
    if (fill_index == BUFFER_SIZE && active) {
      fill_index = 0;
      draw_index = 0;
      // shutdown();
    }
    if (mode == MENU) {
      draw_menu();
    }
  }
  else {
    disableAD();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    ili9341_text_attr_t attr;
    attr.bg_color = ILI9341_BLACK;
    attr.fg_color = ILI9341_LIGHTGREY;
    attr.font = &ili9341_font_16x26;
    attr.origin_x = 100;
    attr.origin_y = 100;
    ili9341_draw_string(ili9341_lcd, attr, "SHUTDOWN");
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

void button_press() {
  if (mode == MEASURE) {
    mode = MENU;
  }
  else {
    if (menu.selected == MENU_SIZE - 1) {
      mode = MEASURE;
    }
  }
}

void button_turned_right() {
//  enabled = false;
  if (mode == MENU) {
    menu.selected++;
    if (menu.selected >= MENU_SIZE) {
      menu.selected = MENU_SIZE - 1;
    }
  }
  else {
    debug_line_y++;
  }
}

void button_turned_left() {
  if (mode == MENU) {
    if (menu.selected > 0) {
      menu.selected--;
    }
    else {
      menu.selected = 0;
    }
  }
  else {
    debug_line_y--;
  }
}

void display_handle_interrupt(uint8_t interrupt) {
  switch (interrupt) {
    case BUTTON_PRESS: button_press();
      break;
    case RIGHT_TURN: button_turned_right();
      break;
    case LEFT_TURN: button_turned_left();
      break;
  }
}

void display_shutdown() {
  enabled = false;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM16) {
    if (enabled) {
      if (fill_index >= BUFFER_SIZE) {
        return;
      }
      raw_values[fill_index] = dma_values[0];
      time_buffer[fill_index] = HAL_GetTick();
      fill_index++;
    }
    else {
      HAL_TIM_Base_Stop_IT(htim);
    }
  }
  else if (htim->Instance == TIM7) {
    HAL_TIM_Base_Stop_IT(htim);
    display_shutdown();
  }
}
