#include "board_logic.h"
#include "../system_states/system_states.h"
#include <zephyr/kernel.h>


void handle_output_board_logic(void)
{
    switch (g_system_state) {
        case STATE_IDLE:
            // TODO: Idle state - ensure motors stopped
            break;
            
        case STATE_HOMING:
            // TODO: Home comb mechanism and conveyor
            break;
            
        case STATE_COMPRESSION:
            // Nothing to do - INPUT board handles this
            break;
            
        case STATE_SEPARATION:
            // Nothing to do - INPUT board handles this
            break;
            
        case STATE_COMB_INSERTION:
            // TODO: Drive BLDC motors to insert comb between pleats
            // When complete: change_system_state(STATE_OUTPUT);
            break;
            
        case STATE_OUTPUT:
            k_sleep(K_MSEC(3000));
            broadcast_state_change(STATE_IDLE);
            // TODO: Run conveyor to eject completed filter
            // When complete: change_system_state(STATE_IDLE);
            break;
            
        case STATE_ERROR:
            // TODO: Stop all motors, safe state
            break;
            
        case STATE_EMERGENCY_STOP:
            // TODO: Immediate stop of all motors
            break;
            
        default:
            printk("Unknown state: %d", g_system_state);
            break;
    }
}