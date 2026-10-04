#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);

void encoder_get_counts(int32_t *left_count, int32_t *right_count);

void encoder_reset(void);

#endif
