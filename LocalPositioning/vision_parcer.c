#include "vision_parcer.h"
#include <stdlib.h>
#include <string.h>

static bool is_key(const char *json, const jsmntok_t *token, const char *key) {
  int token_length = token->end - token->start;

  if (token->type != JSMN_STRING)
    return false;
  if (token_length != (int)strlen(key))
    return false;

  return strncmp(json + token->start, key, token_length) == 0;
}

static double token_to_double(const char *json, const jsmntok_t *token) {
  char text[32];
  int token_length = token->end - token->start;

  if (token_length > 31)
    return 0;

  memcpy(text, json + token->start, token_length);
  text[token_length] = '\0';
  return strtod(text, NULL);
}

static int token_to_int(const char *json, const jsmntok_t *token) {
  char text[32];
  int token_length = token->end - token->start;

  if (token_length > 31)
    return 0;

  memcpy(text, json + token->start, token_length);
  text[token_length] = '\0';
  return (int)strtol(text, NULL, 10);
}

static bool token_to_bool(const char *json, const jsmntok_t *token) {
  return *(json + token->start) == 't';
}

static int index_after_value(const jsmntok_t *tokens, int index) {
  int children_left = tokens[index].size;
  index++;

  while (children_left > 0) {
    index = index_after_value(tokens, index);
    children_left--;
  }

  return index;
}

static bool parse_marker(const char *json, const jsmntok_t *tokens,
                         int object_index, marker_t *marker) {
  if (tokens[object_index].type != JSMN_OBJECT)
    return false;
  int key_count = tokens[object_index].size;
  int index = object_index + 1;

  for (int key_number = 0; key_number < key_count; key_number++) {
    const jsmntok_t *key = &tokens[index];
    const jsmntok_t *value = &tokens[index + 1];

    if (is_key(json, key, "id")) {
      marker->id = token_to_int(json, value);
    } else if (is_key(json, key, "x")) {
      marker->x = token_to_double(json, value);
    } else if (is_key(json, key, "y")) {
      marker->y = token_to_double(json, value);
    } else if (is_key(json, key, "angle")) {
      marker->angle = token_to_double(json, value);
    }

    index = index_after_value(tokens, index);
  }

  return true;
}

static bool parse_markers(const char *json, const jsmntok_t *tokens,
                          int array_index, vision_packet_t *packet) {
  if (tokens[array_index].type != JSMN_ARRAY)
    return false;
  int element_count = tokens[array_index].size;
  int index = array_index + 1;
  int count = 0;

  for (int element_number = 0; element_number < element_count;
       element_number++) {
    if (element_number < MAX_MARKERS) {
      if (!parse_marker(json, tokens, index, &packet->markers[count])) {
        return false;
      }
      count++;
    }

    index = index_after_value(tokens, index);
  }

  packet->marker_count = count;
  return true;
}

bool parse_packet(const char *json, const jsmntok_t *tokens,
                  vision_packet_t *packet) {
  if (tokens[0].type != JSMN_OBJECT)
    return false;
  int key_count = tokens[0].size;
  int index = 1;

  for (int key_number = 0; key_number < key_count; key_number++) {
    const jsmntok_t *key = &tokens[index];
    const jsmntok_t *value = &tokens[index + 1];

    if (is_key(json, key, "timestamp")) {
      packet->timestamp = token_to_double(json, value);
    } else if (is_key(json, key, "board_detected")) {
      packet->board_detected = token_to_bool(json, value);
    } else if (is_key(json, key, "markers")) {
      if (!parse_markers(json, tokens, index + 1, packet))
        return false;
    }

    index = index_after_value(tokens, index);
  }

  return true;
}
