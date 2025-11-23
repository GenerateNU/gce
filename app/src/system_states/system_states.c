#include "system_states.h"
#include "../can_protocol/can_protocol.h"

volatile system_state_t g_system_state = STATE_IDLE;

// Gets string of current state
const char* get_state_name(system_state_t state)
{
    switch (state) {
        case STATE_IDLE:           return "IDLE";
        case STATE_HOMING:         return "HOMING";
        case STATE_COMPRESSION:    return "COMPRESSION";
        case STATE_SEPARATION:     return "SEPARATION";
        case STATE_COMB_INSERTION: return "COMB_INSERTION";
        case STATE_OUTPUT:         return "OUTPUT";
        case STATE_ERROR:          return "ERROR";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                   return "UNKNOWN";
    }
}

// Broadcast state change to all boards
void broadcast_state_change(system_state_t new_state) 
{
    can_packet_t packet;
    state_change_payload_t *payload = (state_change_payload_t*)packet.payload;

    // Update local state first
    g_system_state = new_state;

    // Build packet
    packet.msg_type = MSG_STATE_CHANGE;
    packet.sub_id = node_role;
    payload->new_state = (uint8_t)new_state;
    
    // Clear reserved bytes
    for (int i = 1; i < 6; i++) {
        packet.payload[i] = 0;
    }
    
    // Broadcast to all nodes (NODE_BROADCAST = 0x0F typically)
    int ret = can_protocol_send(0, NODE_BROADCAST, &packet);
    
    if (ret == 0) {
        printk("Broadcast state change: %s", get_state_name(new_state));
    } else {
        printk("Failed to broadcast state change: %d", ret);
    }
}

// Handle incoming state change message (called from CAN handler thread)
void handle_state_change_message(const can_packet_t *packet)
{
    const state_change_payload_t *payload = (const state_change_payload_t*)packet->payload;
    system_state_t new_state = (system_state_t)payload->new_state;
    
    // Validate state value
    if (new_state > STATE_EMERGENCY_STOP) {
        printk("Invalid state received: %d", new_state);
        return;
    }
    
    // Update global state
    system_state_t old_state = g_system_state;
    g_system_state = new_state;
    
    printk("State updated: %s -> %s", 
            get_state_name(old_state), 
            get_state_name(new_state));
}

