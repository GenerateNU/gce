#ifndef SYSTEM_STATES_H
#define SYSTEM_STATES_H

#include <stdint.h>
#include "../can_protocol/can_protocol.h"

// Global system states - shared across all boards
typedef enum {
    STATE_IDLE = 0,         // System ready, waiting for cycle start
    STATE_HOMING,           // Initial homing of all actuators
    STATE_COMPRESSION,      // Linear actuator compressing filter
    STATE_SEPARATION,       // Lead screw separating pleats
    STATE_COMB_INSERTION,   // BLDC motors inserting holding comb
    STATE_OUTPUT,           // Lead screw + conveyor ejecting filter
    STATE_ERROR,            // Error condition - requires operator intervention 
    STATE_EMERGENCY_STOP    // E-stop activated
} system_state_t;

// Global state variable - updated via CAN messages
extern volatile system_state_t g_system_state;

// Gets state name
const char* get_state_name(system_state_t state);

// Helper functions for state management
void broadcast_state_change(system_state_t new_state);
void handle_state_change_message(const can_packet_t *packet);

#endif // SYSTEM_STATES_H