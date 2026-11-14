// waypoints.c - Part B: waypoints read from the ceiling camera (marker id 13).
// Values are the camera readings in the capture screenshots, converted m -> cm.
// theta: 0 = +x, 90 = +y.
//
// P2, P3, P6, P7, P10, P11 were captured with the Zumo facing +y (reading ~90..96)
// although the route goes -y there; theta below is the reading minus 180.
// TODO: re-capture these facing the real route direction, and average x per straight
//       (P2/P3, P4/P5, P6/P7, P8/P9 should share the same x).
#include "waypoints.h"

const waypoint_t WAYPOINTS[WP_COUNT] = {
    // id    x        y        theta    kind
    { 1,  -40.8f,  101.6f,   91.0f, WP_U_ENTRY   },  // top-left,  exit of left straight (up)
    { 2,  -18.2f,  100.7f,  -84.0f, WP_U_EXIT    },  // end of top U, going down
    { 3,  -22.9f,   30.2f,  -86.0f, WP_U_ENTRY   },  // bottom U entry
    { 4,   -5.6f,   30.5f,   92.0f, WP_U_EXIT    },  // bottom U exit, going up
    { 5,   -7.6f,  100.9f,   95.0f, WP_U_ENTRY   },
    { 6,    9.7f,  100.5f,  -90.0f, WP_U_EXIT    },
    { 7,    5.9f,   30.4f,  -84.0f, WP_U_ENTRY   },
    { 8,   25.6f,   31.1f,   90.0f, WP_U_EXIT    },
    { 9,   22.1f,  100.5f,   95.0f, WP_U_ENTRY   },
    {10,   39.1f,  100.6f,  -89.0f, WP_U_EXIT    },
    {11,   36.9f,   11.4f,  -89.0f, WP_C90_ENTRY },  // bottom-right corner (-90 -> 180)
    {12,   35.7f,    9.5f,  179.0f, WP_C90_EXIT  },
    {13,  -23.0f,    7.9f,  178.0f, WP_C90_ENTRY },  // bottom-left corner (180 -> 90)
    {14,  -39.6f,   12.1f,   90.0f, WP_C90_EXIT  },  // START / FINAL
};
