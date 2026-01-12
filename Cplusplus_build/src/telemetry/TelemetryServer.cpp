#include "telemetry/TelemetryServer.h"
#include <spdlog/spdlog.h>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace sat_sim::telemetry {

    TelemetryServer::TelemetryServer(int pub_port, int cmd_port)
        : m_pub_port(pub_port), m_cmd_port(cmd_port) {}

    TelemetryServer::~TelemetryServer() {
        stop();
    }

    bool TelemetryServer::start() {
        if (m_running) return true;

        spdlog::info("TelemetryServer: Starting on ports {} (pub), {} (cmd)", 
            m_pub_port, m_cmd_port);

        // Note: Full ZMQ implementation would require linking libzmq
        // This is a stub that can be extended
        m_running = true;
        
        spdlog::info("TelemetryServer: Started (stub mode - ZMQ not linked)");
        return true;
    }

    void TelemetryServer::stop() {
        if (!m_running) return;
        m_running = false;
        
        if (m_thread.joinable()) {
            m_thread.join();
        }
        
        spdlog::info("TelemetryServer: Stopped");
    }

    void TelemetryServer::publish(const TelemetryPacket& packet) {
        if (!m_running) return;

        std::string json = packet_to_json(packet);
        
        // In full implementation, this would send via ZMQ PUB socket
        // For now, just log occasionally
        static int count = 0;
        if (++count % 100 == 0) {
            spdlog::debug("TelemetryServer: Published {} packets", count);
        }
    }

    TelemetryPacket TelemetryServer::build_packet(
            double time,
            const core::StateVector& state,
            const core::ControlInput& control,
            int waypoint_idx,
            double pos_error,
            bool mission_complete,
            bool faults_active,
            int healthy_thrusters) {
        
        TelemetryPacket pkt;
        pkt.timestamp = time;
        
        pkt.pos[0] = state.position.x();
        pkt.pos[1] = state.position.y();
        pkt.pos[2] = state.position.z();
        
        pkt.quat[0] = state.attitude(0);
        pkt.quat[1] = state.attitude(1);
        pkt.quat[2] = state.attitude(2);
        pkt.quat[3] = state.attitude(3);
        
        pkt.vel[0] = state.velocity.x();
        pkt.vel[1] = state.velocity.y();
        pkt.vel[2] = state.velocity.z();
        
        pkt.omega[0] = state.angular_velocity.x();
        pkt.omega[1] = state.angular_velocity.y();
        pkt.omega[2] = state.angular_velocity.z();
        
        pkt.rw_torque[0] = control.rw_torques.x();
        pkt.rw_torque[1] = control.rw_torques.y();
        pkt.rw_torque[2] = control.rw_torques.z();
        
        pkt.thrust = control.thruster_activations;
        
        pkt.waypoint_index = waypoint_idx;
        pkt.position_error = pos_error;
        pkt.mission_complete = mission_complete;
        pkt.faults_active = faults_active;
        pkt.healthy_thrusters = healthy_thrusters;
        
        return pkt;
    }

    std::string TelemetryServer::packet_to_json(const TelemetryPacket& pkt) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4);
        
        ss << "{";
        ss << "\"t\":" << pkt.timestamp << ",";
        ss << "\"pos\":[" << pkt.pos[0] << "," << pkt.pos[1] << "," << pkt.pos[2] << "],";
        ss << "\"quat\":[" << pkt.quat[0] << "," << pkt.quat[1] << "," << pkt.quat[2] << "," << pkt.quat[3] << "],";
        ss << "\"vel\":[" << pkt.vel[0] << "," << pkt.vel[1] << "," << pkt.vel[2] << "],";
        ss << "\"omega\":[" << pkt.omega[0] << "," << pkt.omega[1] << "," << pkt.omega[2] << "],";
        ss << "\"rw\":[" << pkt.rw_torque[0] << "," << pkt.rw_torque[1] << "," << pkt.rw_torque[2] << "],";
        ss << "\"thrust\":[";
        for (size_t i = 0; i < pkt.thrust.size(); ++i) {
            ss << pkt.thrust[i];
            if (i < pkt.thrust.size() - 1) ss << ",";
        }
        ss << "],";
        ss << "\"wp\":" << pkt.waypoint_index << ",";
        ss << "\"err\":" << pkt.position_error << ",";
        ss << "\"done\":" << (pkt.mission_complete ? "true" : "false") << ",";
        ss << "\"faults\":" << (pkt.faults_active ? "true" : "false") << ",";
        ss << "\"thrusters\":" << pkt.healthy_thrusters;
        ss << "}";
        
        return ss.str();
    }

} // namespace sat_sim::telemetry
