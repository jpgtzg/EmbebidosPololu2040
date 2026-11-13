#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#define SENSOR_COUNT       5u
#define SENSOR_GPIO_MASK   (0x1Fu << 18)  /* GP18 through GP22 */
#define EMITTER_GPIO       26u
#define CHARGE_US          12u
#define LOW_CONFIRM_US     10u
#define TIMEOUT_US         5000u
#define DISPLAY_PAUSE_MS   250u

/* Sensor order, from the robot's left to its right. */
static const unsigned sensor_pins[SENSOR_COUNT] = {22, 21, 20, 19, 18};
static void sensors_init(void)
{
    for (unsigned i = 0; i < SENSOR_COUNT; ++i) {
        gpio_init(sensor_pins[i]);
        gpio_disable_pulls(sensor_pins[i]);
        gpio_set_dir(sensor_pins[i], GPIO_IN);
    }

    /* Hold motor PWM (GP14/GP15) and proximity emitters (GP16/GP17) low. */
    for (unsigned pin = 14; pin <= 17; ++pin) {
        gpio_init(pin);
        gpio_put(pin, 0);
        gpio_set_dir(pin, GPIO_OUT);
    }

    gpio_init(EMITTER_GPIO);
    gpio_put(EMITTER_GPIO, 1);
    gpio_set_dir(EMITTER_GPIO, GPIO_OUT);
    sleep_ms(2);
}

/* Returns a timeout mask. Bit 0 belongs to S1, bit 4 to S5. */
static uint32_t read_reflectance_us(uint32_t readings[SENSOR_COUNT])
{
    uint32_t first_low_us[SENSOR_COUNT] = {0};
    uint32_t low_candidates = 0;
    uint32_t pending = (1u << SENSOR_COUNT) - 1u;
    for (unsigned i = 0; i < SENSOR_COUNT; ++i)
        readings[i] = TIMEOUT_US;
    /* Charge the five RC outputs in parallel. */
    gpio_put_masked(SENSOR_GPIO_MASK, SENSOR_GPIO_MASK);
    gpio_set_dir_out_masked(SENSOR_GPIO_MASK);
    busy_wait_us_32(CHARGE_US);
    /* Keep USB interrupt handlers out of the timing window. */
    const uint32_t irq_state = save_and_disable_interrupts();
    const uint32_t start_us = time_us_32();
    gpio_set_dir_in_masked(SENSOR_GPIO_MASK);
    while (pending != 0u) {
        const uint32_t levels = gpio_get_all();
        const uint32_t elapsed_us = time_us_32() - start_us;
        if (elapsed_us >= TIMEOUT_US)
            break;
        for (unsigned i = 0; i < SENSOR_COUNT; ++i) {
            const uint32_t sensor_bit = 1u << i;
            if (!(pending & sensor_bit))
                continue;
            if (levels & (1u << sensor_pins[i])) {
                low_candidates &= ~sensor_bit;
            } else if (!(low_candidates & sensor_bit)) {
                first_low_us[i] = elapsed_us;
                low_candidates |= sensor_bit;
            } else if (elapsed_us - first_low_us[i] >= LOW_CONFIRM_US) {
                readings[i] = first_low_us[i];
                pending &= ~sensor_bit;
            }
        }
    }
    restore_interrupts(irq_state);
    return pending;
}



int main(void)
{
    stdio_init_all();
    sensors_init();
    while (true) {
        if (!stdio_usb_connected()) {
            sleep_ms(50);
            continue;
        }
        uint32_t readings[SENSOR_COUNT];
        const uint32_t timeout_mask = read_reflectance_us(readings);
        printf("S1=%4u us | S2=%4u us | S3=%4u us | "
               "S4=%4u us | S5=%4u us | timeout=0x%02X\n",
               (unsigned)readings[0], (unsigned)readings[1],
               (unsigned)readings[2], (unsigned)readings[3],
               (unsigned)readings[4], (unsigned)timeout_mask);
        sleep_ms(DISPLAY_PAUSE_MS);
    }
}
