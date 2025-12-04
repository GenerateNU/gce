#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Motor driver error codes
 */
typedef enum {
    MOTOR_OK = 0,           /**< Success */
    MOTOR_ERR_INIT = -1,    /**< Initialization error */
    MOTOR_ERR_PWM = -2,     /**< PWM configuration error */
    MOTOR_ERR_INVALID = -3  /**< Invalid parameter */
} motor_error_t;

/**
 * @brief Initialize the motor driver (software PWM)
 * 
 * Configures GPIO and timer for software PWM generation.
 * PWM starts at 0% duty cycle.
 * 
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_init(void);

/**
 * @brief Set motor speed
 * 
 * Controls motor speed by adjusting PWM duty cycle.
 * 
 * @param speed_percent Speed as percentage (0-100)
 *                      0 = stopped
 *                      100 = full speed
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_set_speed(uint8_t speed_percent);

/**
 * @brief Get current motor speed setting
 * 
 * @return Current speed as percentage (0-100)
 */
uint8_t motor_get_speed(void);

/**
 * @brief Software PWM thread entry point (internal use)
 */
void motor_pwm_thread_entry(void *p1, void *p2, void *p3);

#endif /* MOTOR_DRIVER_H */