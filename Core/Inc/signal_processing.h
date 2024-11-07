
#ifndef SIGNAL_PROCESSING_H_
#define SIGNAL_PROCESSING_H_

#include "stm32l4xx_hal.h"

#define BUFFER_SIZE 500l

typedef struct {
  int32_t thf1;
  int32_t thi1;
  bool is_qrs;
  uint16_t pulse;
  bool is_regular;
} pt_result_t;

void process_pan_tompkins(uint16_t* signal, int32_t* filtered, uint32_t current_index, pt_result_t* result);

#endif /* SIGNAL_PROCESSING_H_ */
