#include <zephyr/kernel.h>
#include <string.h>
#include "can_protocol/can_protocol.h"
#include "board_roles/board_logic.h"
#include "system_states/system_states.h"
#include "ui/ui.h"
#include "stepper/stepper.h"
#include "bldc/bldc.h"

/**
 * Thread definitions
 */

// Thread stack size / priority 
#define CAN_THREAD_STACK_SIZE 2048
#define CAN_THREAD_PRIORITY 5

#define STATE_MACHINE_THREAD_STACK_SIZE 2048
#define STATE_MACHINE_THREAD_PRIORITY 6  

#define UI_THREAD_STACK_SIZE 1024
#define UI_THREAD_PRIORITY 7

#define STEPPER_THREAD_STACK_SIZE 2048
#define STEPPER_THREAD_PRIORITY 5

#define MOTOR_PWM_THREAD_STACK_SIZE 1024
#define MOTOR_PWM_THREAD_PRIORITY 4  /* High priority for accurate PWM */

// Thread stack allocation
K_THREAD_STACK_DEFINE(can_thread_stack, CAN_THREAD_STACK_SIZE);
static struct k_thread can_thread_data;

K_THREAD_STACK_DEFINE(state_machine_thread_stack, STATE_MACHINE_THREAD_STACK_SIZE);
static struct k_thread state_machine_thread_data;

K_THREAD_STACK_DEFINE(ui_thread_stack, UI_THREAD_STACK_SIZE);
static struct k_thread ui_thread_data;

K_THREAD_STACK_DEFINE(stepper_thread_stack, STEPPER_THREAD_STACK_SIZE);
static struct k_thread stepper_thread_data;

K_THREAD_STACK_DEFINE(motor_pwm_thread_stack, MOTOR_PWM_THREAD_STACK_SIZE);
static struct k_thread motor_pwm_thread_data;

// Determine node role from build-time configuration
//can_node_id_t node_role = NODE_INPUT;
can_node_id_t node_role = NODE_MAIN;

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/**
 * CAN message handler thread
 * Blocks waiting for CAN messages and processes them
 */
void can_handler_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    can_packet_t packet;

    printk("CAN handler thread started\n");

    while (1)
    {   
        // Block waiting for CAN message (forever timeout)
        int ret = can_protocol_receive(&packet, K_FOREVER);

        if (ret != 0)
        {
            printk("CAN receive error: %d\n", ret);
            continue;
        }

        // Message received - process based on type
        printk("Received CAN message: type=0x%02x, sub_id=0x%02x\n",
               packet.msg_type, packet.sub_id);

        switch (packet.msg_type)
        {
        case MSG_MOTOR_CMD:
            printk("Motor command received\n");
            // TODO: Handle motor command
            break;

        case MSG_MOTOR_STATUS:
            printk("Motor status received\n");
            // TODO: Handle motor status
            break;

        case MSG_EMERGENCY_STOP:
            printk("EMERGENCY STOP received!\n");
            // TODO: Handle emergency stop
            break;

        case MSG_HEARTBEAT:
            printk("Heartbeat received\n");
            // TODO: Handle heartbeat
            break;

        case MSG_ERROR:
            printk("Error message received\n");
            // TODO: Handle error
            break;

        case MSG_SYSTEM_STATUS:
            printk("System status received\n");
            // TODO: Handle system status
            break;
        case MSG_STATE_CHANGE:
            handle_state_change_message(&packet);
            break;
        default:
            printk("Unknown message type: 0x%02x\n", packet.msg_type);
            break;
        }
    }
}

/**
 * State Machine Thread
 * Executes board roles based on respective board
 */
void state_machine_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("State machine thread started (Node: %d)\n", node_role);

    while (1) {
        // Each board runs its own state machine logic
        switch (node_role) {
            case NODE_MAIN:
                handle_main_board_logic();
                break;
                
            case NODE_INPUT:
                handle_input_board_logic();
                break;
                
            case NODE_OUTPUT:
                handle_output_board_logic();
                break;
            case NODE_BROADCAST:
                // Do nothing, should never reach this case
                break;
        }
        
        k_sleep(K_MSEC(100));  // 10Hz update rate
    }
}

static void ui_monitor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        // Monitor system state for errors
        if (g_system_state == STATE_ERROR) {
            if (ui_get_state() != ERROR) {
                ui_set_state(ERROR);
            }
        }
        
        k_sleep(K_MSEC(100));
    }
}

/**
 * Send a heartbeat message (example function)
 */
void send_heartbeat(void)
{
    can_packet_t packet;
    packet.msg_type = MSG_HEARTBEAT;
    packet.sub_id = 0;
    memset(packet.payload, 0, sizeof(packet.payload));

    int ret = can_protocol_send(7, NODE_BROADCAST, &packet);

    if (ret == 0)
    {
        printk("Heartbeat sent OK\n");
    }
    else
    {
        printk("Heartbeat send FAILED: %d\n", ret);
    }
}



// # Build for board
// west build -b frdm_mcxn947/mcxn947/cpu0 -p always
// west flash

int main(void)
{
    printk("\n=== Entered Main ===\n");
    printk("Initializing as node: %d\n", node_role);

    // Initialize CAN protocol
    int ret = can_protocol_init(node_role);
    if (ret != 0) {
        printk("Failed to initialize CAN protocol: %d\n", ret);
        return ret;
    }
    printk("CAN protocol initialized successfully\n");

    // Initialize stepper motor
    ret = stepper_init();
    if (ret != 0) {
        printk("Failed to initialize stepper: %d\n", ret);
        return ret;
    }
    printk("Stepper motor initialized successfully\n");

    // Initialize UI subsystem (IMPORTANT - was missing!)
    ret = ui_init();
    if (ret != 0) {
        printk("Failed to initialize UI: %d\n", ret);
        return ret;
    }
    printk("UI initialized successfully\n");

    // Initialize BLDC motor
    ret = motor_init();
    if (ret != 0) {
        printk("Failed to initialize motor: %d\n", ret);
        return ret;
    }
    printk("BLDC motor initialized successfully\n");

    ret = motor_init();
    if (ret != 0) {
        printk("Failed to initialize motor: %d\n", ret);
        return ret;
    }
    printk("BLDC motor initialized successfully\n");
    
    // Create BLDC handler thread
    k_tid_t motor_pwm_tid = k_thread_create(
        &motor_pwm_thread_data,
        motor_pwm_thread_stack,
        K_THREAD_STACK_SIZEOF(motor_pwm_thread_stack),
        motor_pwm_thread_entry,
        NULL, NULL, NULL,
        MOTOR_PWM_THREAD_PRIORITY,
        0,
        K_NO_WAIT);
    
    k_thread_name_set(motor_pwm_tid, "motor_pwm");
    printk("Motor PWM thread created\n");

    // Create CAN handler thread
    k_tid_t can_tid = k_thread_create(
        &can_thread_data, can_thread_stack,
        K_THREAD_STACK_SIZEOF(can_thread_stack),
        can_handler_thread, NULL, NULL, NULL,
        CAN_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(can_tid, "can_handler");
    printk("CAN handler thread created\n");

    // Spawn state machine thread
    k_tid_t state_machine_tid = k_thread_create(
        &state_machine_thread_data, state_machine_thread_stack,
        K_THREAD_STACK_SIZEOF(state_machine_thread_stack),
        state_machine_thread_entry, NULL, NULL, NULL,
        STATE_MACHINE_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(state_machine_tid, "state_machine");
    printk("State machine thread created\n");

    // Spawn UI monitoring thread
    k_tid_t ui_tid = k_thread_create(
        &ui_thread_data, ui_thread_stack,
        K_THREAD_STACK_SIZEOF(ui_thread_stack),
        ui_monitor_thread, NULL, NULL, NULL,
        UI_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(ui_tid, "ui_monitor");
    printk("UI thread created\n");

    // Spawn stepper motor thread
    k_tid_t stepper_tid = k_thread_create(
        &stepper_thread_data, stepper_thread_stack,
        K_THREAD_STACK_SIZEOF(stepper_thread_stack),
        stepper_thread_entry, NULL, NULL, NULL,
        STEPPER_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(stepper_tid, "stepper");
    printk("Stepper thread created\n");

    // Setup LED for testing
    int ret1;
    bool led_state = true;

    if (!gpio_is_ready_dt(&led)) {
        printk("LED GPIO not ready\n");
        return 0;
    }

    ret1 = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret1 < 0) {
        printk("Failed to configure LED\n");
        return 0;
    }

    // Test motor at 50% speed
    printk("Starting motor test at 50%% speed\n");
    motor_set_speed(50);

    // Main loop - blink LED
    while (1) {
        ret1 = gpio_pin_toggle_dt(&led);
        if (ret1 < 0) {
            return 0;
        }
        led_state = !led_state;
        printk("LED state: %s\n", led_state ? "ON" : "OFF");
        k_msleep(2000);
    }

    return 0;
}