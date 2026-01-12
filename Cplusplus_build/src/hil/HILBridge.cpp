#include "hil/HILBridge.h"
#include <spdlog/spdlog.h>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

namespace sat_sim::hil {

    HILBridge::HILBridge(Protocol protocol, const std::string& config)
        : m_protocol(protocol), m_config(config) {}

    HILBridge::~HILBridge() {
        disconnect();
    }

    bool HILBridge::connect() {
        if (m_connected) return true;

        spdlog::info("HILBridge: Connecting via {} to {}", 
            m_protocol == Protocol::SERIAL ? "Serial" :
            m_protocol == Protocol::CAN ? "CAN" : "UDP",
            m_config);

#ifdef __linux__
        if (m_protocol == Protocol::SERIAL) {
            m_fd = open(m_config.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (m_fd < 0) {
                spdlog::error("HILBridge: Failed to open {}", m_config);
                return false;
            }

            struct termios tty;
            memset(&tty, 0, sizeof(tty));
            tcgetattr(m_fd, &tty);
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tcsetattr(m_fd, TCSANOW, &tty);

            m_connected = true;
        } else if (m_protocol == Protocol::UDP) {
            m_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (m_fd < 0) {
                spdlog::error("HILBridge: Failed to create UDP socket");
                return false;
            }
            m_connected = true;
        }
#else
        spdlog::warn("HILBridge: Hardware I/O not supported on this platform (stub mode)");
        m_connected = true; // Stub mode
#endif

        spdlog::info("HILBridge: Connected");
        return m_connected;
    }

    void HILBridge::disconnect() {
        if (!m_connected) return;

#ifdef __linux__
        if (m_fd >= 0) {
            close(m_fd);
            m_fd = -1;
        }
#endif
        m_connected = false;
        spdlog::info("HILBridge: Disconnected");
    }

    bool HILBridge::send_state(const core::StateVector& state) {
        if (!m_connected) return false;

        StateMessage msg;
        msg.seq = m_seq++;
        
        msg.pos[0] = static_cast<float>(state.position.x());
        msg.pos[1] = static_cast<float>(state.position.y());
        msg.pos[2] = static_cast<float>(state.position.z());
        
        msg.quat[0] = static_cast<float>(state.attitude(0));
        msg.quat[1] = static_cast<float>(state.attitude(1));
        msg.quat[2] = static_cast<float>(state.attitude(2));
        msg.quat[3] = static_cast<float>(state.attitude(3));
        
        msg.vel[0] = static_cast<float>(state.velocity.x());
        msg.vel[1] = static_cast<float>(state.velocity.y());
        msg.vel[2] = static_cast<float>(state.velocity.z());
        
        msg.omega[0] = static_cast<float>(state.angular_velocity.x());
        msg.omega[1] = static_cast<float>(state.angular_velocity.y());
        msg.omega[2] = static_cast<float>(state.angular_velocity.z());

        msg.checksum = compute_checksum(reinterpret_cast<uint8_t*>(&msg), 
                                         sizeof(msg) - sizeof(msg.checksum));

#ifdef __linux__
        if (m_fd >= 0) {
            ssize_t n = write(m_fd, &msg, sizeof(msg));
            if (n > 0) m_stats.tx_messages++;
            return n == sizeof(msg);
        }
#endif
        // Stub mode - just count
        m_stats.tx_messages++;
        return true;
    }

    void HILBridge::poll() {
        if (!m_connected) return;

#ifdef __linux__
        if (m_fd >= 0) {
            uint8_t buf[256];
            ssize_t n = read(m_fd, buf, sizeof(buf));
            if (n > 0) {
                process_message(std::vector<uint8_t>(buf, buf + n));
            }
        }
#endif
    }

    uint16_t HILBridge::compute_checksum(const uint8_t* data, size_t len) const {
        uint16_t sum = 0;
        for (size_t i = 0; i < len; ++i) {
            sum += data[i];
        }
        return sum;
    }

    void HILBridge::process_message(const std::vector<uint8_t>& data) {
        if (data.size() < 2) return;

        MessageType type = static_cast<MessageType>(data[0]);
        
        if (type == MessageType::CONTROL_CMD && data.size() >= sizeof(ControlMessage)) {
            const ControlMessage* msg = reinterpret_cast<const ControlMessage*>(data.data());
            
            core::ControlInput ctrl;
            ctrl.rw_torques = core::Vector3(msg->rw_torque[0], msg->rw_torque[1], msg->rw_torque[2]);
            ctrl.thruster_activations.resize(6);
            for (int i = 0; i < 6; ++i) {
                ctrl.thruster_activations[i] = msg->thrust[i];
            }

            m_stats.rx_messages++;
            if (m_callback) m_callback(ctrl);
        }
    }

} // namespace sat_sim::hil
