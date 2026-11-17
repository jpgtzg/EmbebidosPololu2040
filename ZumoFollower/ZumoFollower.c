// Receives the robot pose over CAN from the LocalPositioning Pico W.
// Pipeline: vision PC -> UDP -> LocalPositioning (Pico W) -> CAN -> this Zumo.
//
// CAN frame layout (must match LocalPositioning.c; little-endian, standard
// 11-bit IDs, 500 kbps):
//   CAN_ID_POSE: [0]=seq [1]=flags [2..3]=x mm [4..5]=y mm [6..7]=angle 0.1 deg
//   CAN_ID_TIME: [0]=seq [1..4]=epoch ms (low 32 bits) [5..7]=unused

#include "ZumoFollower.h"
#include "mcp2515.h"
#include "motors.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define CAN_ID_POSE 0x300
#define CAN_ID_TIME 0x301
#define FLAG_BOARD_DETECTED 0x01
#define FLAG_MARKER_FOUND 0x02

#define POSE_STALE_MS 500

static robot_state_t state;
static pending_pair_t pending;

static absolute_time_t last_pose_rx;
static bool stale_reported = false;

static robot_state_t first_state;
static bool is_first_state = true;

static int16_t get_i16(const uint8_t *src) {
  return (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static uint32_t get_u32(const uint8_t *src) {
  return (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

// as the messages come separately, this function joins them into a robot_sate_t
// once they share the same seq
static bool try_commit(void) {
  if (!pending.have_pose || !pending.have_time ||
      pending.pose_seq != pending.time_seq)
    return false;

  state.valid = true;
  state.seq = pending.pose_seq;
  state.board_detected = pending.flags & FLAG_BOARD_DETECTED;
  state.marker_found = pending.flags & FLAG_MARKER_FOUND;
  state.x_m = (float)pending.x / 1000.0f;
  state.y_m = (float)pending.y / 1000.0f;
  state.angle_deg = (float)pending.angle / 10.0f;
  state.timestamp_ms = pending.time_ms;

  pending.have_pose = false;
  pending.have_time = false;
  return true;
}

// deconstructs a can frame into either of two messages following its id
// once pending has both time and pose, try_commit joins them into a single
// robot_state_t
static void handle_frame(const can_frame_t *f) {
  bool committed = false;

  if (f->id == CAN_ID_POSE && f->dlc >= 8) {
    pending.pose_seq = f->data[0];
    pending.flags = f->data[1];
    pending.x = get_i16(&f->data[2]);
    pending.y = get_i16(&f->data[4]);
    pending.angle = get_i16(&f->data[6]);
    pending.have_pose = true;

    last_pose_rx = get_absolute_time();
    stale_reported = false;
    committed = try_commit();
  } else if (f->id == CAN_ID_TIME && f->dlc >= 5) {
    pending.time_seq = f->data[0];
    pending.time_ms = get_u32(&f->data[1]);
    pending.have_time = true;
    committed = try_commit();
  } else {
    return;
  }

  if (committed) {
    if (state.marker_found) {
      printf("RX seq=%3u t=%u ms  x=%.3f m  y=%.3f m  angle=%.1f deg%s\n",
             state.seq, (unsigned)state.timestamp_ms, state.x_m, state.y_m,
             state.angle_deg, state.board_detected ? "" : "  (no board)");
    } else {
      printf("RX seq=%3u t=%u ms  robot marker NOT found%s\n", state.seq,
             (unsigned)state.timestamp_ms,
             state.board_detected ? "" : "  (no board)");
    }
  }
}

int main(void) {
  stdio_init_all();
  sleep_ms(3000); // give the USB serial connection time to enumerate

  motors_init();

  printf("\n--- ZumoFollower: CAN pose receiver ---\n");
  mcp2515_init();
  if (!mcp2515_set_bitrate_500kbps_8mhz())
    printf("CAN bitrate setup failed\n");
  if (!mcp2515_set_mode(MCP2515_MODE_NORMAL))
    printf("CAN could not enter NORMAL mode\n");

  last_pose_rx = get_absolute_time();

  while (true) {
    while (mcp2515_message_available()) {
      can_frame_t rx;
      if (!mcp2515_read_message(&rx))
        break;
      handle_frame(&rx);
    }

    if (!stale_reported &&
        absolute_time_diff_us(last_pose_rx, get_absolute_time()) >
            POSE_STALE_MS * 1000) {
      printf("WARNING: no pose received for %d ms\n", POSE_STALE_MS);
      state.valid = false;
      stale_reported = true;
    }

    if (state.valid && state.marker_found) {
      if (is_first_state) {
        is_first_state = false;
        first_state = state;
      }

      // this is only for the wireless positioning part
      double dif_cm = (state.y_m - first_state.y_m) * 100;
      if (dif_cm < 50) {
        motors_set_speeds(2500, 2500);
      } else {
        motors_set_speeds(0, 0);
      }
    } else {
      motors_set_speeds(0, 0);
    }
    sleep_ms(5);
  }
}
