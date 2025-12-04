/**
 * @file bldc.c
 * @brief Software PWM driver for motor speed control
 */

#include "bldc.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* GPIO pin for PWM output - using P2_6 (J3-15) */
#define PWM_GPIO_NODE DT_ALIAS(motor_pwm_pin)

/* PWM configuration */
#define PWM_FREQ_HZ     25000  /* 25 kHz */
#define PWM_PERIOD_US   (1000000 / PWM_FREQ_HZ)  /* 40 us for 25kHz */

/* GPIO device spec */
static const struct gpio_dt_spec pwm_pin = GPIO_DT_SPEC_GET(PWM_GPIO_NODE, gpios);

/* Driver state */
static struct {
    uint8_t current_speed;
    uint32_t duty_cycle_us;  /* High time in microseconds */
    bool initialized;
    bool running;
} motor_state = {
    .current_speed = 0,
    .duty_cycle_us = 0,
    .initialized = false,
    .running = false
};

/* Synchronization */
static K_MUTEX_DEFINE(motor_mutex);

motor_error_t motor_init(void)
{
    int ret;

    printk("=== MOTOR_INIT: Starting (Software PWM) ===\n");

    if (motor_state.initialized) {
        printk("MOTOR_INIT: Already initialized\n");
        return MOTOR_OK;
    }

    /* Verify GPIO device is ready */
    if (!gpio_is_ready_dt(&pwm_pin)) {
        printk("MOTOR_INIT: ERROR - PWM GPIO device not ready\n");
        return MOTOR_ERR_INIT;
    }

    /* Configure pin as output, initial low */
    ret = gpio_pin_configure_dt(&pwm_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("MOTOR_INIT: ERROR - Failed to configure PWM pin: %d\n", ret);
        return MOTOR_ERR_INIT;
    }

    k_mutex_lock(&motor_mutex, K_FOREVER);
    motor_state.initialized = true;
    motor_state.running = true;
    motor_state.current_speed = 0;
    motor_state.duty_cycle_us = 0;
    k_mutex_unlock(&motor_mutex);

    printk("MOTOR_INIT: Success!\n");
    printk("  Frequency: %d Hz (software PWM)\n", PWM_FREQ_HZ);
    printk("  Period: %d us\n", PWM_PERIOD_US);
    printk("  Output pin: P2_6 (J3-15)\n");

    return MOTOR_OK;
}

motor_error_t motor_set_speed(uint8_t speed_percent)
{
    if (!motor_state.initialized) {
        printk("Motor driver not initialized\n");
        return MOTOR_ERR_INIT;
    }

    if (speed_percent > 100) {
        printk("Invalid speed: %d (must be 0-100)\n", speed_percent);
        return MOTOR_ERR_INVALID;
    }

    /* Calculate duty cycle in microseconds */
    uint32_t duty_us = (PWM_PERIOD_US * speed_percent) / 100;

    k_mutex_lock(&motor_mutex, K_FOREVER);
    motor_state.current_speed = speed_percent;
    motor_state.duty_cycle_us = duty_us;
    k_mutex_unlock(&motor_mutex);

    printk("Motor speed set to %d%% (duty: %d us / %d us)\n", 
           speed_percent, duty_us, PWM_PERIOD_US);

    return MOTOR_OK;
}

uint8_t motor_get_speed(void)
{
    uint8_t speed;
    
    k_mutex_lock(&motor_mutex, K_FOREVER);
    speed = motor_state.current_speed;
    k_mutex_unlock(&motor_mutex);
    
    return speed;
}

/**
 * Software PWM generation thread
 * Runs at high priority to maintain accurate timing
 */
void motor_pwm_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("Motor PWM thread started\n");

    uint32_t duty_us;
    uint32_t off_time_us;

    while (1) {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        
        if (!motor_state.running) {
            k_mutex_unlock(&motor_mutex);
            k_sleep(K_MSEC(10));
            continue;
        }

        duty_us = motor_state.duty_cycle_us;
        k_mutex_unlock(&motor_mutex);

        /* Calculate off time */
        off_time_us = PWM_PERIOD_US - duty_us;

        /* Generate PWM pulse */
        if (duty_us > 0) {
            /* Set pin high */
            gpio_pin_set_dt(&pwm_pin, 1);
            
            /* Wait for high time */
            if (duty_us >= 10) {
                k_usleep(duty_us);
            } else if (duty_us > 0) {
                k_busy_wait(duty_us);
            }
        }

        if (off_time_us > 0) {
            /* Set pin low */
            gpio_pin_set_dt(&pwm_pin, 0);
            
            /* Wait for low time */
            if (off_time_us >= 10) {
                k_usleep(off_time_us);
            } else {
                k_busy_wait(off_time_us);
            }
        }

        /* Handle 0% (always off) and 100% (always on) duty cycles */
        if (duty_us == 0) {
            gpio_pin_set_dt(&pwm_pin, 0);
            k_usleep(PWM_PERIOD_US);
        } else if (duty_us >= PWM_PERIOD_US) {
            gpio_pin_set_dt(&pwm_pin, 1);
            k_usleep(PWM_PERIOD_US);
        }
    }
}