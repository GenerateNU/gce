// ui.c
#include "ui.h"
#include "../system_states/system_states.h"
#include "../can_protocol/can_protocol.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// UI state variable
volatile ui_state_t ui_state = IDLE;

// LED device tree specs
static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_red), gpios, {0}),      // 0 = RED
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_yellow), gpios, {0}),   // 1 = YELLOW
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_green), gpios, {0}),    // 2 = GREEN
};

// Button device tree specs
static const struct gpio_dt_spec buttons[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(btn_start_left), gpios, {0}),   // 0 = LEFT
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(btn_start_right), gpios, {0}),  // 1 = RIGHT
};

// Button interrupt callback data
static struct gpio_callback button_left_cb_data;
static struct gpio_callback button_right_cb_data;

// Button press tracking
static volatile bool left_pressed = false;
static volatile bool right_pressed = false;
static volatile bool dual_press_handled = false;

// Debounce timing
#define DEBOUNCE_MS 50
static uint32_t last_left_press = 0;
static uint32_t last_right_press = 0;

// LED indices
#define LED_RED    0
#define LED_YELLOW 1
#define LED_GREEN  2

// Button indices
#define BTN_LEFT   0
#define BTN_RIGHT  1

// Forward declarations
static int leds_init(void);
static int buttons_init(void);
static void update_leds(void);
static void button_left_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button_right_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

int ui_init(void)
{
    int ret;
    
    printk("Initializing UI subsystem");
    
    // Initialize LEDs
    ret = leds_init();
    if (ret != 0) {
        printk("LED initialization failed: %d", ret);
        return ret;
    }
    
    // Initialize buttons
    ret = buttons_init();
    if (ret != 0) {
        printk("Button initialization failed: %d", ret);
        return ret;
    }
    
    // Set initial UI state
    ui_state = IDLE;
    update_leds();
    
    printk("UI subsystem initialized - State: IDLE (Yellow LED)");
    return 0;
}

ui_state_t ui_get_state(void)
{
    return ui_state;
}

void ui_set_state(ui_state_t new_ui_state)
{
    if (ui_state != new_ui_state) {
        ui_state = new_ui_state;
        
        const char *state_names[] = {"IDLE", "RUNNING", "ERROR"};
        printk("UI state changed to: %s", state_names[new_ui_state]);
        
        update_leds();
    }
}

// LED Functions
static int leds_init(void)
{
    int ret;
    
    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            printk("LED %d GPIO not ready", i);
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("Failed to configure LED %d: %d", i, ret);
            return ret;
        }
    }
    
    printk("LEDs initialized");
    return 0;
}

static void update_leds(void)
{
    // Turn all LEDs off first
    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        gpio_pin_set_dt(&leds[i], 0);
    }
    
    // Turn on appropriate LED based on state
    switch (ui_state) {
        case IDLE:
            gpio_pin_set_dt(&leds[LED_YELLOW], 1);
            break;
            
        case RUNNING:
            gpio_pin_set_dt(&leds[LED_GREEN], 1);
            break;
            
        case ERROR:
            gpio_pin_set_dt(&leds[LED_RED], 1);
            break;
    }
}

// Button Functions
static int buttons_init(void)
{
    int ret;
    
    // Configure left button
    if (!gpio_is_ready_dt(&buttons[BTN_LEFT])) {
        printk("Left button GPIO not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&buttons[BTN_LEFT], GPIO_INPUT);
    if (ret != 0) {
        printk("Failed to configure left button: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&buttons[BTN_LEFT], GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Failed to configure left button interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&button_left_cb_data, button_left_isr, 
                      BIT(buttons[BTN_LEFT].pin));
    gpio_add_callback(buttons[BTN_LEFT].port, &button_left_cb_data);
    
    // Configure right button
    if (!gpio_is_ready_dt(&buttons[BTN_RIGHT])) {
        printk("Right button GPIO not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&buttons[BTN_RIGHT], GPIO_INPUT);
    if (ret != 0) {
        printk("Failed to configure right button: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&buttons[BTN_RIGHT], GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Failed to configure right button interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&button_right_cb_data, button_right_isr, 
                      BIT(buttons[BTN_RIGHT].pin));
    gpio_add_callback(buttons[BTN_RIGHT].port, &button_right_cb_data);
    
    printk("Safety buttons initialized (dual press required)");
    return 0;
}

static void check_dual_press(void)
{
    // Check if both buttons are currently pressed
    if (left_pressed && right_pressed && !dual_press_handled) {
        printk("DUAL BUTTON PRESS DETECTED");
        
        // Only trigger state change if in IDLE state
        if (ui_state == IDLE) {
            ui_set_state(RUNNING);
            broadcast_state_change(STATE_COMPRESSION);

            
            // Trigger system state change
            // broadcast a "start cycle" CAN message here
            // if (g_system_state == STATE_IDLE) {
            //     change_system_state(STATE_HOMING);
            // }
        }
        
        dual_press_handled = true;
    }
    
    // Reset trigger when either button is released
    if (!left_pressed || !right_pressed) {
        dual_press_handled = false;
    }
}

static void button_left_isr(const struct device *dev, 
                           struct gpio_callback *cb, 
                           uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    uint32_t now = k_uptime_get_32();
    
    // Debounce
    if ((now - last_left_press) < DEBOUNCE_MS) {
        return;
    }
    last_left_press = now;
    
    // Check if button is pressed (active low)
    int val = gpio_pin_get_dt(&buttons[BTN_LEFT]);
    left_pressed = (val == 0);
    
    printk("Left button: %s", left_pressed ? "PRESSED" : "RELEASED");
    
    check_dual_press();
}

static void button_right_isr(const struct device *dev, 
                            struct gpio_callback *cb, 
                            uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    uint32_t now = k_uptime_get_32();
    
    // Debounce
    if ((now - last_right_press) < DEBOUNCE_MS) {
        return;
    }
    last_right_press = now;
    
    // Check if button is pressed (active low)
    int val = gpio_pin_get_dt(&buttons[BTN_RIGHT]);
    right_pressed = (val == 0);
    
    printk("Right button: %s", right_pressed ? "PRESSED" : "RELEASED");
    
    check_dual_press();
}