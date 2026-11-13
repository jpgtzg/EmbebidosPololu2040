#pragma once
#define JSMN_STATIC
#include "jsmn.h"
#include <stdbool.h>
#include <stdint.h>

#define MAX_MARKERS 8

typedef struct {
  int id;
  float x;
  float y;
  float angle;
} marker_t;

typedef struct {
  double timestamp;
  bool board_detected;
  uint8_t marker_count;
  marker_t markers[MAX_MARKERS];
} vision_packet_t;

bool parse_packet(const char *json, const jsmntok_t *tokens,
                  vision_packet_t *packet);
