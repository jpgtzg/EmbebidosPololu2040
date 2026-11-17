#ifndef ZUMO_FOLLOWER_H
#define ZUMO_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

// Latest decoded pose of the robot, as received from LocalPositioning over CAN.
typedef struct {
  bool valid;             // at least one complete pose/time pair received
  bool board_detected;
  bool marker_found;      // false => x/y/angle are not meaningful
  float x_m;
  float y_m;
  float angle_deg;
  uint32_t timestamp_ms;  // epoch ms, low 32 bits
  uint8_t seq;
} robot_state_t;

// Halves of a pose/time pair, matched by sequence number.
typedef struct {
  bool have_pose;
  uint8_t pose_seq;
  uint8_t flags;
  int16_t x, y, angle;
  bool have_time;
  uint8_t time_seq;
  uint32_t time_ms;
} pending_pair_t;

#endif // ZUMO_FOLLOWER_H
