#include <zephyr/kernel.h>
#include <string.h>
#include "can_protocol/can_protocol.h"

/**
 * Thread definitions
 */

// Thread stack size
#define CAN_THREAD_STACK_SIZE 2048
#define CAN_THREAD_PRIORITY 5

// Thread stack allocation
K_THREAD_STACK_DEFINE(can_thread_stack, CAN_THREAD_STACK_SIZE);
static struct k_thread can_thread_data;

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
    
    while (1) {
        // Block waiting for CAN message (forever timeout)
        int ret = can_protocol_receive(&packet, K_FOREVER);
        
        if (ret != 0) {
            printk("CAN receive error: %d\n", ret);
            continue;
        }
        
        // Message received - process based on type
        printk("Received CAN message: type=0x%02x, sub_id=0x%02x\n", 
               packet.msg_type, packet.sub_id);
        
        switch (packet.msg_type) {
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
                
            default:
                printk("Unknown message type: 0x%02x\n", packet.msg_type);
                break;
        }
    }
}

/**
 * Send a motor command (example function - can be called from any thread)
 */
void send_motor_command_example(can_node_id_t target_node, uint8_t motor_id, int16_t speed)
{
    can_packet_t packet;
    packet.msg_type = MSG_MOTOR_CMD;
    packet.sub_id = 0;
    
    // Fill motor command payload
    motor_cmd_payload_t *cmd = (motor_cmd_payload_t *)packet.payload;
    cmd->motor_id = motor_id;
    cmd->speed = speed;
    cmd->direction = (speed >= 0) ? 0 : 1;
    cmd->brake = 0;
    cmd->reserved = 0;
    
    // Send it (non-blocking, returns immediately)
    int ret = can_protocol_send(1, target_node, &packet);
    if (ret == 0) {
        printk("Sent motor command to node %d: motor=%d, speed=%d\n", 
               target_node, motor_id, speed);
    } else {
        printk("Failed to send motor command: %d\n", ret);
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
    
    if (ret == 0) {
        printk("Heartbeat sent OK\n");
    } else {
        printk("Heartbeat send FAILED: %d\n", ret);
    }
}


// # Build for MAIN board
// west build -b frdm_mcxn947/mcxn947/cpu0 -p always -- -DBOARD_ROLE_MAIN=ON
// west flash

// # Build for INPUT board
// west build -b frdm_mcxn947/mcxn947/cpu0 -p always -- -DBOARD_ROLE_INPUT=ON
// west flash

// # Build for OUTPUT board
// west build -b frdm_mcxn947/mcxn947/cpu0 -p always -- -DBOARD_ROLE_OUTPUT=ON
// west flash

int main(void)
{
    printk("\n=== Entered Main ===\n");
    
    // Determine node role from build-time configuration
    can_node_id_t node_role = NODE_MAIN;
    
    printk("Initializing as node: %d\n", node_role);
    
    // Initialize CAN protocol
    int ret = can_protocol_init(node_role);
    if (ret != 0) {
        printk("Failed to initialize CAN protocol: %d\n", ret);
        return ret;
    }
    
    printk("CAN protocol initialized successfully\n");
    
    // Create CAN handler thread
    k_tid_t can_tid = k_thread_create(
        &can_thread_data,
        can_thread_stack,
        K_THREAD_STACK_SIZEOF(can_thread_stack),
        can_handler_thread,
        NULL, NULL, NULL,
        CAN_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
    
    k_thread_name_set(can_tid, "can_handler");
    
    printk("CAN handler thread created\n");
    printk("System ready - waiting for CAN messages...\n\n");
    
    // Main thread - example of sending messages periodically
    while (1) {
        k_sleep(K_SECONDS(5));
        
        // Example: MAIN board sends periodic heartbeat
        if (node_role == NODE_INPUT) {
            send_heartbeat();
            
            // Example: Send a test motor command every 10 seconds
            static int counter = 0;
            counter++;
            if (counter >= 2) {
                send_motor_command_example(NODE_INPUT, 1, 5000);
                counter = 0;
            }
        }
    }
    
    return 0;
}