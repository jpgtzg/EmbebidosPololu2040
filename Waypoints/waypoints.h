// waypoints.h - Part B: waypoint table (x, y, theta) in the camera frame
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct { float x_cm, y_cm, theta_deg; } pose_t;   // camera measurement

typedef enum {
    WP_STRAIGHT   = 0,  // not used by the current table, reserved
    WP_U_ENTRY    = 1,  // start of a 180 deg semicircle
    WP_U_EXIT     = 2,  // end of a 180 deg semicircle
    WP_C90_ENTRY  = 3,  // start of a 90 deg corner
    WP_C90_EXIT   = 4   // end of a 90 deg corner
} wp_kind_t;

typedef struct {
    uint8_t   id;          // P1..P14
    float     x_cm, y_cm;  // position over the track line
    float     theta_deg;   // desired heading, (-180, 180], 0 = +x, 90 = +y
    wp_kind_t kind;
} waypoint_t;

#define WP_COUNT 14
extern const waypoint_t WAYPOINTS[WP_COUNT];

// Route order (indices into WAYPOINTS): P1 -> P2 -> ... -> P14 -> back to P1
#define WP_START_IDX  0    // P1, heading +y
#define WP_FINAL_IDX  0    // P1 again after closing the lap (P13 -> P14 corner, then up the left straight)

static inline bool wp_is_turn90(const waypoint_t *w) {
    return w->kind == WP_C90_ENTRY || w->kind == WP_C90_EXIT;
}

// Next waypoint in route order (P14 -> P1 -> P2 ... -> P13 -> P14)
static inline int wp_next(int i) { return (i + 1) % WP_COUNT; }
