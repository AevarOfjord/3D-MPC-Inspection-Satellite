#pragma once
/**
 * @file HILBridge.h
 * @brief Hardware-In-the-Loop (HIL) communication bridge.
 * 
 * Provides abstraction for Serial/CAN communication with flight hardware.
 */

#include "core/Types.h"
#include <string>
#include <functional>
#include <vector>
#include <cstdint>

namespace sat_sim::hil {

    /**
     * @brief Communication protocol type.
     */
    enum class Protocol {
        SERIAL,     // UART/RS-232
        CAN,        // CAN bus
        UDP         // UDP sockets
    };

    /**
     * @brief Message types for HIL communication.
     */
    enum class MessageType : uint8_t {
        STATE_UPDATE = 0x01,    // Sim -> Hardware: current state
        CONTROL_CMD = 0x02,     // Hardware -> Sim: control command
        HEARTBEAT = 0x03,       // Bidirectional keepalive
        CONFIG = 0x04,          // Configuration data
        ERROR = 0xFF            // Error message
    };

    /**
     * @brief Packed state message (32 bytes).
     */
    struct __attribute__((packed)) StateMessage {
        uint8_t type = static_cast<uint8_t>(MessageType::STATE_UPDATE);
        uint8_t seq;
        float pos[3];
        float quat[4];
        float vel[3];
        float omega[3];
        uint16_t checksum;
    };

    /**
     * @brief Packed control message.
     */
    struct __attribute__((packed)) ControlMessage {
        uint8_t type = static_cast<uint8_t>(MessageType::CONTROL_CMD);
        uint8_t seq;
        float rw_torque[3];
        float thrust[6];    // Max 6 thrusters in packed format
        uint16_t checksum;
    };

    /**
     * @brief Callback for received control commands.
     */
    using ControlCallback = std::function<void(const core::ControlInput&)>;

    /**
     * @brief HIL communication bridge.
     */
    class HILBridge {
    public:
        /**
         * @brief Create HIL bridge.
         * @param protocol Communication protocol.
         * @param config Protocol config (port name, baud rate, etc.)
         */
        HILBridge(Protocol protocol, const std::string& config);
        ~HILBridge();

        /**
         * @brief Open connection.
         */
        bool connect();

        /**
         * @brief Close connection.
         */
        void disconnect();

        /**
         * @brief Check if connected.
         */
        bool is_connected() const { return m_connected; }

        /**
         * @brief Send state update to hardware.
         */
        bool send_state(const core::StateVector& state);

        /**
         * @brief Register callback for control commands.
         */
        void set_control_callback(ControlCallback cb) { m_callback = cb; }

        /**
         * @brief Poll for incoming messages (call in main loop).
         */
        void poll();

        /**
         * @brief Get statistics.
         */
        struct Stats {
            uint64_t tx_messages = 0;
            uint64_t rx_messages = 0;
            uint64_t errors = 0;
            double latency_ms = 0;
        };
        const Stats& get_stats() const { return m_stats; }

    private:
        Protocol m_protocol;
        std::string m_config;
        bool m_connected = false;
        int m_fd = -1;  // File descriptor (serial/socket)
        uint8_t m_seq = 0;
        ControlCallback m_callback;
        Stats m_stats;

        uint16_t compute_checksum(const uint8_t* data, size_t len) const;
        void process_message(const std::vector<uint8_t>& data);
    };

} // namespace sat_sim::hil
