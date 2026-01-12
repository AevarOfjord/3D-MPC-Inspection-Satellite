#pragma once
/**
 * @file TelemetryServer.h
 * @brief ZMQ-based telemetry API for external monitoring.
 * 
 * Publishes state/control data and accepts commands via ZeroMQ.
 */

#include "core/Types.h"
#include "config/ConfigSchema.h"
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

namespace sat_sim::telemetry {

    /**
     * @brief Telemetry data packet (JSON-serializable).
     */
    struct TelemetryPacket {
        double timestamp;
        
        // State
        double pos[3];
        double quat[4];
        double vel[3];
        double omega[3];
        
        // Control
        double rw_torque[3];
        std::vector<double> thrust;
        
        // Mission
        int waypoint_index;
        double position_error;
        bool mission_complete;
        
        // Status
        bool faults_active;
        int healthy_thrusters;
    };

    /**
     * @brief Command types from external clients.
     */
    enum class TelemetryCommand {
        PAUSE,
        RESUME,
        ABORT,
        INJECT_FAULT,
        CLEAR_FAULTS,
        SET_TARGET
    };

    /**
     * @brief Command callback.
     */
    using CommandCallback = std::function<void(TelemetryCommand, const std::string&)>;

    /**
     * @brief ZMQ-based telemetry server.
     */
    class TelemetryServer {
    public:
        /**
         * @brief Create telemetry server.
         * @param pub_port Port for publishing telemetry (PUB socket).
         * @param cmd_port Port for receiving commands (REP socket).
         */
        TelemetryServer(int pub_port = 5555, int cmd_port = 5556);
        ~TelemetryServer();

        /**
         * @brief Start server (spawns background thread).
         */
        bool start();

        /**
         * @brief Stop server.
         */
        void stop();

        /**
         * @brief Publish telemetry packet.
         */
        void publish(const TelemetryPacket& packet);

        /**
         * @brief Build packet from simulation state.
         */
        TelemetryPacket build_packet(
            double time,
            const core::StateVector& state,
            const core::ControlInput& control,
            int waypoint_idx,
            double pos_error,
            bool mission_complete,
            bool faults_active,
            int healthy_thrusters);

        /**
         * @brief Register command callback.
         */
        void set_command_callback(CommandCallback cb) { m_cmd_callback = cb; }

        /**
         * @brief Get publish rate (Hz).
         */
        double get_publish_rate() const { return m_rate; }

    private:
        int m_pub_port;
        int m_cmd_port;
        std::atomic<bool> m_running{false};
        std::thread m_thread;
        std::mutex m_mutex;
        CommandCallback m_cmd_callback;
        double m_rate = 0;

        void run_loop();
        std::string packet_to_json(const TelemetryPacket& pkt);
    };

} // namespace sat_sim::telemetry
