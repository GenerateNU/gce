#include "can_protocol.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(can_protocol, LOG_LEVEL_DBG);

// CAN device
static const struct device *can_dev;

// Message queue for received CAN packets
#define MSG_QUEUE_SIZE 32
K_MSGQ_DEFINE(can_rx_msgq, sizeof(can_packet_t), MSG_QUEUE_SIZE, 4);

// Local node ID (set during initialization)
static can_node_id_t local_node_id = NODE_MAIN;

/**
 * CAN receive callback - called from ISR context
 */
static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    // Parse the CAN identifier
    uint32_t can_id = frame->id;
    uint8_t dst_node = PARSE_CAN_DST_NODE(can_id);
    
    // Check if message is for us or broadcast
    if (dst_node != local_node_id && dst_node != NODE_BROADCAST) {
        return; // Not for us, ignore
    }
    
    // Copy frame data into packet structure
    can_packet_t packet;
    memcpy(&packet, frame->data, sizeof(can_packet_t));
    
    // Put packet into message queue
    int ret = k_msgq_put(&can_rx_msgq, &packet, K_NO_WAIT);
    if (ret != 0) {
        printk("CAN RX queue full, message dropped\n");
    }
}

/**
 * Initialize CAN protocol
 * @param node_id: The role of this board (MAIN, INPUT, OUTPUT)
 * @return 0 on success, negative on error
 */
int can_protocol_init(can_node_id_t node_id)
{
    local_node_id = node_id;
    
    // Get CAN device
    can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(can_dev)) {
        printk("CAN device not ready\n");
        return -1;
    }
    
    // Set CAN mode to normal
    int ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
    if (ret != 0) {
        printk("Failed to set CAN mode: %d\n", ret);
        return ret;
    }
    
    // Set up CAN filter to receive messages for this node and broadcasts
    struct can_filter filter = {
        .flags = CAN_FILTER_IDE, // Extended ID
        .id = BUILD_CAN_ID(0, 0, 0, local_node_id, 0),
        .mask = 0x0000F000, // Mask for destination node field (bits 15-12)
    };
    
    int filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
    if (filter_id < 0) {
        printk("Failed to add CAN filter: %d\n", filter_id);
        return filter_id;
    }
    
    // Add filter for broadcast messages
    struct can_filter bcast_filter = {
        .flags = CAN_FILTER_IDE,
        .id = BUILD_CAN_ID(0, 0, 0, NODE_BROADCAST, 0),
        .mask = 0x0000F000,
    };
    
    filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &bcast_filter);
    if (filter_id < 0) {
        printk("Failed to add broadcast filter: %d\n", filter_id);
        return filter_id;
    }
    
    // Start CAN controller
    ret = can_start(can_dev);
    if (ret != 0) {
        printk("Failed to start CAN: %d\n", ret);
        return ret;
    }
    
    printk("CAN protocol initialized as node %d\n", node_id);
    return 0;
}

/**
 * Send a CAN packet
 * @param priority: Message priority (0=highest, 7=lowest)
 * @param dst_node: Destination node ID
 * @param packet: Pointer to packet to send
 * @return 0 on success, negative on error
 */
int can_protocol_send(uint8_t priority, can_node_id_t dst_node, const can_packet_t *packet)
{
    if (!can_dev) {
        printk("CAN not initialized\n");
        return -1;
    }
    
    // Build CAN extended identifier
    uint32_t can_id = BUILD_CAN_ID(
        priority,
        packet->msg_type,
        local_node_id,
        dst_node,
        packet->sub_id
    );
    
    // Prepare CAN frame
    struct can_frame frame = {
        .flags = CAN_FRAME_IDE, // Extended ID
        .id = can_id,
        .dlc = sizeof(can_packet_t),
    };
    
    // Copy packet data into frame
    memcpy(frame.data, packet, sizeof(can_packet_t));
    
    // Send frame
    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
        printk("Failed to send CAN frame: %d\n", ret);
        return ret;
    }
    
    return 0;
}

/**
 * Receive a CAN packet (blocking)
 * @param packet: Pointer to store received packet
 * @param timeout: Timeout duration
 * @return 0 on success, negative on timeout or error
 */
int can_protocol_receive(can_packet_t *packet, k_timeout_t timeout)
{
    int ret = k_msgq_get(&can_rx_msgq, packet, timeout);
    if (ret != 0) {
        return ret; // Timeout or error
    }
    
    return 0;
}

/**
 * Get local node ID
 */
can_node_id_t can_protocol_get_node_id(void)
{
    return local_node_id;
}

/**
 * Purge the receive message queue
 */
void can_protocol_purge_rx_queue(void)
{
    k_msgq_purge(&can_rx_msgq);
    printk("CAN RX queue purged\n");
}