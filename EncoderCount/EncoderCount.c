#include "encoder.h"
#include "motors.h"
#include "pico/stdlib.h"
#include "stdlib.h"
#include <pico/stdio.h>
#include <pico/time.h>
#include <stdio.h>

int32_t clamp(int32_t value, int32_t min, int32_t max);
void drive_straight(int32_t target_counts);
void turn_right_90(int32_t target_counts);
void turn_right_90_precise(int32_t target_counts);
void drive_with_correction(int32_t target_counts);
void move_forward_correction(int32_t counts);
void move_backward_correction(int32_t counts);
int32_t angle_to_counts(int32_t degrees);
void turn_right_correction(int32_t counts);
void turn_left_correction(int32_t counts);

#define POSITION_TOLERANCE 10
#define ANGULAR_CORRECTION_SPEED 1200
#define TURN_CORRECTION_DEGREES 3
#define COUNTS_PER_METER 8535
#define COUNTS_PER_90_DEGREES 631
#define BASE_SPEED 2500
#define TURN_SPEED 1500
#define KP 8

int main(void) {
  stdio_init_all();

  motors_init();

  encoder_init();
  encoder_reset();

  sleep_ms(10000);
  // drive_straight(COUNTS_PER_METER);
  // turn_right_90(630);
  // turn_right_90_precise(COUNTS_PER_90_DEGREES);
  // drive_with_correction(COUNTS_PER_METER);

  for (int side = 0; side < 4; side++) {
    printf("Side %d\n", side + 1);
    drive_with_correction(COUNTS_PER_METER);
    printf("Straight completed\n");
    turn_right_90(COUNTS_PER_90_DEGREES);
    if (TURN_CORRECTION_DEGREES > 0) {
      turn_right_correction(angle_to_counts(TURN_CORRECTION_DEGREES));
      sleep_ms(300);
    }
    printf("Turn completed\n");
  }
  motors_set_speeds(0, 0);
  printf("Square completed.\n");
  while (true) {
    tight_loop_contents();
  }
}

void drive_straight(int32_t target_counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    printf("Left = %ld    Right = %ld\n", (long)left, (long)right);

    int32_t distance = (left + right) / 2;
    if (abs(distance) >= target_counts) {
      break;
    }

    int error = left - right;
    double correction = KP * error;

    int32_t left_speed = BASE_SPEED - correction;
    int32_t right_speed = BASE_SPEED + correction;

    left_speed = clamp(left_speed, 1000, 3000);
    right_speed = clamp(right_speed, 1000, 3000);

    motors_set_speeds(left_speed, right_speed);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
  sleep_ms(300);
}

void drive_with_correction(int32_t target_counts) {
  drive_straight(target_counts);

  int32_t left, right;
  encoder_get_counts(&left, &right);
  int32_t position = (left + right) / 2;

  while (true) {
    int32_t difference = target_counts - position;

    if (abs(difference) <= POSITION_TOLERANCE) {
      break;
    }

    if (difference > 0) {
      move_forward_correction(difference);
    } else {
      move_backward_correction(-difference);
    }
    sleep_ms(300);
    encoder_get_counts(&left, &right);
    position += (left + right) / 2;
  }
  motors_set_speeds(0, 0);
  sleep_ms(300);
}

void turn_right_90(int32_t target_counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    printf("Left = %ld    Right = %ld\n", (long)left, (long)right);

    int32_t rotation = (abs(left) + abs(right)) / 2;
    if (rotation >= target_counts) {
      break;
    }
    motors_set_speeds(TURN_SPEED, -TURN_SPEED);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
  sleep_ms(300);
}

void turn_right_90_precise(int32_t target) {
  encoder_reset();

  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    printf("Left = %ld    Right = %ld\n", (long)left, (long)right);

    int32_t rotation = (abs(left) + abs(right)) / 2;
    int32_t remaining = target - rotation;

    if (remaining <= 0) {
      break;
    }
    int32_t speed;
    if (remaining > 300)
      speed = 1800;
    else if (remaining > 100)
      speed = 1200;
    else
      speed = 700;
    motors_set_speeds(speed, -speed);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
  sleep_ms(300);
}

int32_t clamp(int32_t value, int32_t min, int32_t max) {
  if (value < min)
    return min;
  else if (value > max)
    return max;
  return value;
}

void move_forward_correction(int32_t counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    int32_t average = (left + right) / 2;
    if (average >= counts)
      break;
    motors_set_speeds(700, 700);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
}

void move_backward_correction(int32_t counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    int32_t average = (left + right) / 2;
    if (average <= -counts)
      break;
    motors_set_speeds(-700, -700);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
}

int32_t angle_to_counts(int32_t degrees) {
  return (COUNTS_PER_90_DEGREES * degrees + 45) / 90;
}

void turn_right_correction(int32_t counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    int32_t rotation = (abs(left) + abs(right)) / 2;
    if (rotation >= counts)
      break;
    motors_set_speeds(ANGULAR_CORRECTION_SPEED, -ANGULAR_CORRECTION_SPEED);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
}

void turn_left_correction(int32_t counts) {
  encoder_reset();
  while (true) {
    int32_t left, right;
    encoder_get_counts(&left, &right);
    int32_t rotation = (abs(left) + abs(right)) / 2;
    if (rotation >= counts)
      break;
    motors_set_speeds(-ANGULAR_CORRECTION_SPEED, ANGULAR_CORRECTION_SPEED);
    sleep_ms(5);
  }
  motors_set_speeds(0, 0);
}
