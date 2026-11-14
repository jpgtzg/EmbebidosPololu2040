// can_pose.h - protocolo CAN de la pose de la camara (compartido Pico 2 W <-> Zumo)
//
// Una trama CAN estandar de 8 bytes (ID = CAN_TX_ID del gateway = CAN_PEER_ID del Zumo):
//   bytes 0-1  x     int16, cm * 10   (big-endian)   rango +-3276.7 cm
//   bytes 2-3  y     int16, cm * 10
//   bytes 4-5  theta int16, grados * 10, (-180, 180]
//   byte  6    age   uint8, frame_age_ms de la camara (saturado a 255)
//   byte  7    seq   uint8, contador que sube con cada pose nueva (para detectar datos viejos)
#ifndef CAN_POSE_H
#define CAN_POSE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mcp2515.h"

typedef struct {
    float   x_cm, y_cm, theta_deg;
    uint8_t age_ms;
    uint8_t seq;
} can_pose_t;

static inline int16_t can_pose_clamp16(float v) {
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}

static inline void can_pose_pack(const can_pose_t *p, can_frame_t *f) {
    int16_t x = can_pose_clamp16(p->x_cm * 10.0f);
    int16_t y = can_pose_clamp16(p->y_cm * 10.0f);
    int16_t t = can_pose_clamp16(p->theta_deg * 10.0f);
    f->extended = false;
    f->dlc = 8;
    f->data[0] = (uint8_t)((uint16_t)x >> 8); f->data[1] = (uint8_t)x;
    f->data[2] = (uint8_t)((uint16_t)y >> 8); f->data[3] = (uint8_t)y;
    f->data[4] = (uint8_t)((uint16_t)t >> 8); f->data[5] = (uint8_t)t;
    f->data[6] = p->age_ms;
    f->data[7] = p->seq;
}

static inline bool can_pose_unpack(const can_frame_t *f, can_pose_t *p) {
    if (f->dlc != 8) return false;
    int16_t x = (int16_t)(((uint16_t)f->data[0] << 8) | f->data[1]);
    int16_t y = (int16_t)(((uint16_t)f->data[2] << 8) | f->data[3]);
    int16_t t = (int16_t)(((uint16_t)f->data[4] << 8) | f->data[5]);
    p->x_cm = x / 10.0f;
    p->y_cm = y / 10.0f;
    p->theta_deg = t / 10.0f;
    p->age_ms = f->data[6];
    p->seq = f->data[7];
    return true;
}

#endif // CAN_POSE_H
