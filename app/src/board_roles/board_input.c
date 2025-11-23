// src/board_roles/board_input.c
#include "board_logic.h"
#include "../system_states/system_states.h"
#include <zephyr/kernel.h>

void handle_input_board_logic(void)
{
    switch (g_system_state) {
        case STATE_IDLE:
            // TODO: Idle state - ensure actuators stopped
            break;
            
        case STATE_HOMING:
            // TODO: Home linear actuator and lead screw
            k_sleep(K_MSEC(3000));
            broadcast_state_change(STATE_COMPRESSION);
            break;
            
        case STATE_COMPRESSION:
            // TODO: Run linear actuator to compress filter
            // When complete: change_system_state(STATE_SEPARATION);
            k_sleep(K_MSEC(3000));
            broadcast_state_change(STATE_SEPARATION);
            break;
            
        case STATE_SEPARATION:
            // TODO: Run lead screw to separate pleats
            // When complete: change_system_state(STATE_COMB_INSERTION);
            break;
            
        case STATE_COMB_INSERTION:
            // Nothing to do - OUTPUT board handles this
            break;
            
        case STATE_OUTPUT:
            // TODO: Push filter through lead screw onto conveyor
            break;
            
        case STATE_ERROR:
            // TODO: Stop all motors, safe state
            break;
            
        case STATE_EMERGENCY_STOP:
            // TODO: Immediate stop of all actuators
            break;
            
        default:
            printk("Unknown state: %d", g_system_state);
            break;
    }
}