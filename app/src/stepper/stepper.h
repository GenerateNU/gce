#ifndef STEPPER_H
#define STEPPER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

/* Pin definitions - adjust to your device tree */
#define STEPPER_STEP_NODE    DT_ALIAS(stepper_step)
#define STEPPER_DIR_NODE     DT_ALIAS(stepper_dir)
#define STEPPER_ENABLE_NODE  DT_ALIAS(stepper_enable)

/* Direction constants */
typedef enum {
    STEPPER_DIR_CW = 0,   /* Clockwise */
    STEPPER_DIR_CCW = 1   /* Counter-clockwise */
} stepper_dir_t;

/**
 * Initialize stepper motor driver
 * Sets up GPIO pins and default state (disabled)
 * 
 * @return 0 on success, negative errno on failure
 */
int stepper_init(void);

/**
 * Stepper thread entry point
 */
void stepper_thread_entry(void *p1, void *p2, void *p3);

/**
 * Enable or disable the stepper motor driver
 * When disabled, motor is de-energized and can be moved manually
 * 
 * @param enable true to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int stepper_enable(bool enable);

/**
 * Start a stepper motor move (non-blocking)
 * Returns immediately, movement happens in background thread
 * Only one move can be active at a time - calling this while moving
 * will update to the new move parameters
 * 
 * @param steps Number of steps to move
 * @param direction Direction to move (CW or CCW)
 * @param step_delay_us Delay between steps in microseconds (controls speed)
 *                      Typical: 500us (2kHz) to 5000us (200Hz)
 *                      Min: 200us (5kHz) for DRV8434
 * @return 0 on success, negative errno on failure
 */
int stepper_start_move(uint32_t steps, stepper_dir_t direction, 
                       uint32_t step_delay_us);

/**
 * Stop the stepper motor immediately
 * 
 * @return 0 on success
 */
int stepper_stop(void);

/**
 * Check if stepper is currently moving
 * 
 * @return true if moving, false if idle
 */
bool stepper_is_moving(void);

#endif /* STEPPER_H */