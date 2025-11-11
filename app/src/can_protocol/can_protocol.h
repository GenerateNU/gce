#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

/**
 * Message types for CAN communication
 */
typedef uint8_t can_msg_type_t;

#define MSG_MOTOR_CMD      0x01
#define MSG_MOTOR_STATUS   0x02
#define MSG_EMERGENCY_STOP 0x03
#define MSG_HEARTBEAT      0x04
#define MSG_ERROR          0x05
#define MSG_SYSTEM_STATUS  0x06

/**
 * Node identifiers for distributed system
 */
typedef enum {
    NODE_MAIN = 0x00,
    NODE_INPUT = 0x01,
    NODE_OUTPUT = 0x02,
    NODE_BROADCAST = 0x0F,
} can_node_id_t;

/**
 * CAN message packet structure
 * Fits within CAN-FD 8-byte payload
 */
typedef struct __attribute__((packed)) {
    can_msg_type_t msg_type;
    uint8_t sub_id;
    uint8_t payload[6];
} can_packet_t;

/**
 * Motor command payload structure
 * Used when msg_type = MSG_MOTOR_CMD
 */
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    int16_t speed;
    uint8_t direction;
    uint8_t brake;
    uint8_t reserved;
} motor_cmd_payload_t;

/**
 * Motor status payload structure
 * Used when msg_type = MSG_MOTOR_STATUS
 */
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    int16_t actual_speed;
    uint8_t hall_state;
    uint8_t fault_flags;
    uint8_t reserved;
} motor_status_payload_t;

/**
 * CAN extended identifier structure (29-bit)
 * Embeds routing information in the identifier
 */
typedef struct {
    uint8_t priority : 3;       // Message priority (0=highest, 7=lowest)
    uint8_t msg_type : 6;       // Message type
    uint8_t src_node : 4;       // Source node ID
    uint8_t dst_node : 4;       // Destination node ID
    uint16_t sub_id : 12;       // Sub-identifier
} can_identifier_t;

// Helper macros for building/parsing CAN identifiers
#define BUILD_CAN_ID(pri, type, src, dst, sub) \
    ((((uint32_t)(pri) & 0x07) << 26) | \
     (((uint32_t)(type) & 0x3F) << 20) | \
     (((uint32_t)(src) & 0x0F) << 16) | \
     (((uint32_t)(dst) & 0x0F) << 12) | \
     ((uint32_t)(sub) & 0xFFF))

#define PARSE_CAN_PRIORITY(id) (((id) >> 26) & 0x07)
#define PARSE_CAN_MSG_TYPE(id) (((id) >> 20) & 0x3F)
#define PARSE_CAN_SRC_NODE(id) (((id) >> 16) & 0x0F)
#define PARSE_CAN_DST_NODE(id) (((id) >> 12) & 0x0F)
#define PARSE_CAN_SUB_ID(id) ((id) & 0xFFF)

// Function declarations
#include <zephyr/kernel.h>

int can_protocol_init(can_node_id_t node_id);
int can_protocol_send(uint8_t priority, can_node_id_t dst_node, const can_packet_t *packet);
int can_protocol_receive(can_packet_t *packet, k_timeout_t timeout);
can_node_id_t can_protocol_get_node_id(void);
void can_protocol_purge_rx_queue(void);

#endif // CAN_PROTOCOL_H