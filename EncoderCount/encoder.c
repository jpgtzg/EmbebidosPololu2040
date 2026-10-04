#include "encoder.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define RIGHT_ENCODER_A_PIN 8
#define RIGHT_ENCODER_B_PIN 9

#define LEFT_ENCODER_A_PIN 12
#define LEFT_ENCODER_B_PIN 13

static volatile int32_t left_encoder_count = 0;
static volatile int32_t right_encoder_count = 0;

static volatile uint8_t left_previous_state = 0;
static volatile uint8_t right_previous_state = 0;

/*
 * Quadrature transition table.
 *
 * Index:
 *
 *     previous_AB << 2 | current_AB
 *
 * AB is encoded as:
 *
 *     bit 1 = channel A
 *     bit 0 = channel B
 *
 * On the Zumo, B leads A during forward motion. This table therefore
 * assigns positive counts to:
 *
 *     00 -> 01 -> 11 -> 10 -> 00
 */
static const int8_t transition_table[16] = {0,  +1, -1, 0,  -1, 0,  0,  +1,
                                            +1, 0,  0,  -1, 0,  -1, +1, 0};

static inline uint8_t read_encoder_state(uint pin_a, uint pin_b) {
  uint8_t a = gpio_get(pin_a) ? 1u : 0u;
  uint8_t b = gpio_get(pin_b) ? 1u : 0u;

  return (uint8_t)((a << 1) | b);
}

static void encoder_gpio_callback(uint gpio, uint32_t events) {
  (void)events;

  if ((gpio == RIGHT_ENCODER_A_PIN) || (gpio == RIGHT_ENCODER_B_PIN)) {
    uint8_t current_state =
        read_encoder_state(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

    uint8_t table_index =
        (uint8_t)((right_previous_state << 2) | current_state);

    right_encoder_count += -transition_table[table_index];
    right_previous_state = current_state;
  }

  if ((gpio == LEFT_ENCODER_A_PIN) || (gpio == LEFT_ENCODER_B_PIN)) {
    uint8_t current_state =
        read_encoder_state(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);

    uint8_t table_index = (uint8_t)((left_previous_state << 2) | current_state);

    left_encoder_count += -transition_table[table_index];
    left_previous_state = current_state;
  }
}

static void initialize_encoder_pin(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);

  /*
   * The encoder outputs are already conditioned by the Zumo hardware.
   * Internal pull-ups are normally unnecessary.
   */
  gpio_disable_pulls(pin);
}

void encoder_init(void) {
  initialize_encoder_pin(RIGHT_ENCODER_A_PIN);
  initialize_encoder_pin(RIGHT_ENCODER_B_PIN);
  initialize_encoder_pin(LEFT_ENCODER_A_PIN);
  initialize_encoder_pin(LEFT_ENCODER_B_PIN);

  right_previous_state =
      read_encoder_state(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

  left_previous_state =
      read_encoder_state(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);

  /*
   * Install the callback with the first pin.
   */
  gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_A_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &encoder_gpio_callback);

  /*
   * The remaining GPIOs use the same callback.
   */
  gpio_set_irq_enabled(RIGHT_ENCODER_B_PIN,
                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_set_irq_enabled(LEFT_ENCODER_A_PIN,
                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_set_irq_enabled(LEFT_ENCODER_B_PIN,
                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void encoder_get_counts(int32_t *left_count, int32_t *right_count) {
  if ((left_count == NULL) || (right_count == NULL)) {
    return;
  }

  /*
   * Prevent an encoder interrupt from changing a 32-bit value while
   * the main program is copying it.
   */
  uint32_t interrupt_state = save_and_disable_interrupts();

  *left_count = left_encoder_count;
  *right_count = right_encoder_count;

  restore_interrupts(interrupt_state);
}

void encoder_reset(void) {
  uint32_t interrupt_state = save_and_disable_interrupts();

  left_encoder_count = 0;
  right_encoder_count = 0;

  restore_interrupts(interrupt_state);
}
