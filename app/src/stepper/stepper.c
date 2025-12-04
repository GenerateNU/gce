#include "stepper.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(stepper, LOG_LEVEL_INF);

/* GPIO device specifications */
static const struct gpio_dt_spec step_pin = GPIO_DT_SPEC_GET(STEPPER_STEP_NODE, gpios);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(STEPPER_DIR_NODE, gpios);
static const struct gpio_dt_spec enable_pin = GPIO_DT_SPEC_GET(STEPPER_ENABLE_NODE, gpios);

/* Move command structure */
struct stepper_move_params {
    uint32_t steps;
    stepper_dir_t direction;
    uint32_t step_delay_us;
};

/* State tracking */
static bool is_enabled = false;
static bool is_moving = false;
static bool stop_requested = false;
static struct stepper_move_params current_move;

/* Synchronization primitives */
static K_MUTEX_DEFINE(stepper_mutex);
static K_SEM_DEFINE(move_sem, 0, 1);

void stepper_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("Stepper thread started\n");

    while (1) {
        /* Wait for move command */
        k_sem_take(&move_sem, K_FOREVER);

        /* Copy parameters under mutex protection */
        k_mutex_lock(&stepper_mutex, K_FOREVER);
        struct stepper_move_params move = current_move;
        is_moving = true;
        stop_requested = false;
        k_mutex_unlock(&stepper_mutex);

        if (!is_enabled) {
            printk("Cannot move: stepper not enabled");
            k_mutex_lock(&stepper_mutex, K_FOREVER);
            is_moving = false;
            k_mutex_unlock(&stepper_mutex);
            continue;
        }

        /* Set direction */
        gpio_pin_set_dt(&dir_pin, move.direction);
        k_busy_wait(1);  /* 650ns setup time */

        /* Generate step pulses */
        for (uint32_t i = 0; i < move.steps; i++) {
            /* Check for stop request */
            k_mutex_lock(&stepper_mutex, K_FOREVER);
            bool should_stop = stop_requested;
            k_mutex_unlock(&stepper_mutex);
            
            if (should_stop) {
                printk("Move stopped at step %u/%u", i, move.steps);
                break;
            }

            /* Pulse high */
            gpio_pin_set_dt(&step_pin, 1);
            k_busy_wait(2);  /* 1.9us min pulse width */
            
            /* Pulse low */
            gpio_pin_set_dt(&step_pin, 0);
            
            /* Wait between steps */
            k_usleep(move.step_delay_us);
        }

        k_mutex_lock(&stepper_mutex, K_FOREVER);
        is_moving = false;
        k_mutex_unlock(&stepper_mutex);

        printk("Move complete: %u steps %s", move.steps,
                move.direction == STEPPER_DIR_CW ? "CW" : "CCW");
    }
}

int stepper_init(void)
{
    int ret;

    /* Verify GPIO devices are ready */
    if (!gpio_is_ready_dt(&step_pin)) {
        printk("STEP pin GPIO device not ready");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&dir_pin)) {
        printk("DIR pin GPIO device not ready");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&enable_pin)) {
        printk("ENABLE pin GPIO device not ready");
        return -ENODEV;
    }

    /* Configure STEP pin as output, initial low */
    ret = gpio_pin_configure_dt(&step_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure STEP pin: %d", ret);
        return ret;
    }

    /* Configure DIR pin as output, initial CW */
    ret = gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure DIR pin: %d", ret);
        return ret;
    }

    /* Configure ENABLE pin as output, initial disabled (active low on DRV8434) */
    ret = gpio_pin_configure_dt(&enable_pin, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure ENABLE pin: %d", ret);
        return ret;
    }

    printk("Stepper motor initialized (disabled)\n");
    return 0;
}

int stepper_enable(bool enable)
{
    int ret;

    /* DRV8434 ENABLE is active LOW */
    ret = gpio_pin_set_dt(&enable_pin, !enable);
    if (ret < 0) {
        printk("Failed to set ENABLE pin: %d", ret);
        return ret;
    }

    k_mutex_lock(&stepper_mutex, K_FOREVER);
    is_enabled = enable;
    k_mutex_unlock(&stepper_mutex);

    printk("Stepper motor %s", enable ? "enabled" : "disabled");
    return 0;
}

int stepper_start_move(uint32_t steps, stepper_dir_t direction,
                       uint32_t step_delay_us)
{
    if (steps == 0) {
        return 0;
    }

    if (step_delay_us < 200) {
        printk("Step delay %u us too fast, clamping to 200us", step_delay_us);
        step_delay_us = 200;
    }

    k_mutex_lock(&stepper_mutex, K_FOREVER);
    
    /* Set new move parameters */
    current_move.steps = steps;
    current_move.direction = direction;
    current_move.step_delay_us = step_delay_us;
    
    k_mutex_unlock(&stepper_mutex);

    /* Wake up thread to start move */
    k_sem_give(&move_sem);

    printk("Started move: %u steps %s at %u us/step", steps,
            direction == STEPPER_DIR_CW ? "CW" : "CCW", step_delay_us);

    return 0;
}

int stepper_stop(void)
{
    k_mutex_lock(&stepper_mutex, K_FOREVER);
    stop_requested = true;
    k_mutex_unlock(&stepper_mutex);

    printk("Stop requested");
    return 0;
}

bool stepper_is_moving(void)
{
    bool moving;
    
    k_mutex_lock(&stepper_mutex, K_FOREVER);
    moving = is_moving;
    k_mutex_unlock(&stepper_mutex);
    
    return moving;
}