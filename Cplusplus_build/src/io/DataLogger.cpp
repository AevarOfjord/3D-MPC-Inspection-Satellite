#include "io/DataLogger.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cmath>

namespace sat_sim::io {

    DataLogger::DataLogger() {
        create_directories();
        // File opening deferred to first write or init
    }

    DataLogger::~DataLogger() {
        if (m_physics_file.is_open()) {
            m_physics_file.close();
        }
        if (m_control_file.is_open()) {
            m_control_file.close();
        }
    }

    void DataLogger::create_directories() {
        // Create Data/Simulation/<timestamp>
        // Path relative to execution directory (usually workspace root)
        m_base_path = std::filesystem::path("Data") / "Simulation";
        
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* tm_now = std::localtime(&now_time);
        
        std::stringstream ss;
        ss << std::put_time(tm_now, "%d-%m-%Y_%H-%M-%S");
        
        m_sim_dir = m_base_path / ss.str();
        
        try {
            std::filesystem::create_directories(m_sim_dir);
            std::cout << "Data Logger: Created directory " << m_sim_dir.string() << std::endl;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Data Logger Error: Could not create directories: " << e.what() << std::endl;
        }
    }

    core::Vector3 DataLogger::quat_to_euler(const core::Quaternion& q) {
        // q is [w, x, y, z]
        double w = q[0];
        double x = q[1];
        double y = q[2];
        double z = q[3];

        // XYZ order
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return core::Vector3(roll, pitch, yaw);
    }

    void DataLogger::log_physics_step(
        double time,
        const core::StateVector& current_state,
        const core::StateVector& target_state,
        const core::ControlInput& control
    ) {
        if (!m_physics_headers_written) {
            std::filesystem::path p_path = m_sim_dir / "physics_data.csv";
            m_physics_file.open(p_path);
            if (!m_physics_file.is_open()) {
                std::cerr << "Failed to open " << p_path << std::endl;
                return;
            }

            // Write Headers
            m_physics_file << "Time,"
                           << "Current_X,Current_Y,Current_Z,"
                           << "Current_Roll,Current_Pitch,Current_Yaw,"
                           << "Current_VX,Current_VY,Current_VZ,"
                           << "Current_WX,Current_WY,Current_WZ,"
                           << "Target_X,Target_Y,Target_Z,"
                           << "Target_Roll,Target_Pitch,Target_Yaw,"
                           << "Error_X,Error_Y,Error_Z,"
                           << "Error_Roll,Error_Pitch,Error_Yaw,"
                           << "Command_Vector";

            // Add thruster columns
            for (size_t i = 0; i < control.thruster_activations.size(); ++i) {
                m_physics_file << ",Thruster_" << (i + 1) << "_Cmd";
                m_physics_file << ",Thruster_" << (i + 1) << "_Val";
            }
            // Add RW columns
            m_physics_file << ",RW_Torque_X,RW_Torque_Y,RW_Torque_Z";
            m_physics_file << ",RW_Speed_X,RW_Speed_Y,RW_Speed_Z";
            
            m_physics_file << "\n";
            m_physics_headers_written = true;
        }

        // Calculations
        core::Vector3 cur_euler = quat_to_euler(current_state.attitude);
        core::Vector3 tar_euler = quat_to_euler(target_state.attitude);

        core::Vector3 pos_err = current_state.position - target_state.position;
        // Simple angle diff
        core::Vector3 att_err = cur_euler - tar_euler; 
        // Normalize angle? Legacy usually wraps to -pi, pi.
        auto wrap = [](double a){ return std::remainder(a, 2.0 * M_PI); };
        att_err[0] = wrap(att_err[0]);
        att_err[1] = wrap(att_err[1]);
        att_err[2] = wrap(att_err[2]);

        // Write row
        m_physics_file << std::fixed << std::setprecision(6)
                       << time << ","
                       << current_state.position.x() << "," << current_state.position.y() << "," << current_state.position.z() << ","
                       << cur_euler.x() << "," << cur_euler.y() << "," << cur_euler.z() << ","
                       << current_state.velocity.x() << "," << current_state.velocity.y() << "," << current_state.velocity.z() << ","
                       << current_state.angular_velocity.x() << "," << current_state.angular_velocity.y() << "," << current_state.angular_velocity.z() << ","
                       << target_state.position.x() << "," << target_state.position.y() << "," << target_state.position.z() << ","
                       << tar_euler.x() << "," << tar_euler.y() << "," << tar_euler.z() << ","
                       << pos_err.x() << "," << pos_err.y() << "," << pos_err.z() << ","
                       << att_err.x() << "," << att_err.y() << "," << att_err.z() << ",";
        
        // Command Vector String (simple representation)
        m_physics_file << "\"[";
        for (size_t i = 0; i < control.thruster_activations.size(); ++i) {
             m_physics_file << (i > 0 ? ", " : "") << std::setprecision(3) << control.thruster_activations[i];
        }
        m_physics_file << "]\"";

        // Thruster Vals
        for (double val : control.thruster_activations) {
            m_physics_file << "," << val << "," << val; // Cmd and Val same for now if instantaneous
        }
        
        // RW Torques (X,Y,Z)
        m_physics_file << "," << control.rw_torques.x() << "," << control.rw_torques.y() << "," << control.rw_torques.z();
        
        // RW Speeds (Now available in StateVector)
        m_physics_file << "," << current_state.rw_speeds.x() << "," 
                       << current_state.rw_speeds.y() << "," 
                       << current_state.rw_speeds.z();

        m_physics_file << "\n";
    }

    void DataLogger::log_control_step(
        double time,
        const core::StateVector& current_state,
        const core::StateVector& target_state,
        const core::ControlInput& control,
        double mpc_time_ms
    ) {
        if (!m_control_headers_written) {
            std::filesystem::path c_path = m_sim_dir / "control_data.csv";
            m_control_file.open(c_path);
            if (!m_control_file.is_open()) {
                std::cerr << "Failed to open " << c_path << std::endl;
                return;
            }

            // Write Headers (Minimal set for Visualizer)
            // Visualizer needs Control_Time and thrusters
            m_control_file << "Step,Control_Time,"
                           << "Current_X,Current_Y,Current_Z,"
                           << "Current_Roll,Current_Pitch,Current_Yaw,"
                           << "Target_X,Target_Y,Target_Z,"
                           << "Target_Roll,Target_Pitch,Target_Yaw,"
                           << "Error_X,Error_Y,Error_Z,"
                           << "Error_Roll,Error_Pitch,Error_Yaw,"
                           << "Command_Vector";
            
             // Add thruster columns (Legacy visualizer might check these in control too? No mostly physics)
             // But let's add them for consistency
            for (size_t i = 0; i < control.thruster_activations.size(); ++i) {
                m_control_file << ",Thruster_" << (i + 1) << "_Cmd";
                m_control_file << ",Thruster_" << (i + 1) << "_Val";
            }
            // Add RW columns
            m_control_file << ",RW_Torque_X,RW_Torque_Y,RW_Torque_Z";
            m_control_file << ",RW_Speed_X,RW_Speed_Y,RW_Speed_Z";
            
            // Add Timing columns
            m_control_file << ",MPC_Time_ms";

            m_control_file << "\n";
            m_control_headers_written = true;
        }

        core::Vector3 cur_euler = quat_to_euler(current_state.attitude);
        core::Vector3 tar_euler = quat_to_euler(target_state.attitude);

        core::Vector3 pos_err = current_state.position - target_state.position;
        core::Vector3 att_err = cur_euler - tar_euler;
        
        // Wrap angle
        auto wrap = [](double a){ return std::remainder(a, 2.0 * M_PI); };
        att_err[0] = wrap(att_err[0]);
        att_err[1] = wrap(att_err[1]);
        att_err[2] = wrap(att_err[2]);

        static int step_count = 0;
        step_count++;

        m_control_file << std::fixed << std::setprecision(6)
                       << step_count << "," << time << ","
                       << current_state.position.x() << "," << current_state.position.y() << "," << current_state.position.z() << ","
                       << cur_euler.x() << "," << cur_euler.y() << "," << cur_euler.z() << ","
                       << target_state.position.x() << "," << target_state.position.y() << "," << target_state.position.z() << ","
                       << tar_euler.x() << "," << tar_euler.y() << "," << tar_euler.z() << ","
                       << pos_err.x() << "," << pos_err.y() << "," << pos_err.z() << ","
                       << att_err.x() << "," << att_err.y() << "," << att_err.z() << ",";
        
        m_control_file << "\"[";
        for (size_t i = 0; i < control.thruster_activations.size(); ++i) {
             m_control_file << (i > 0 ? ", " : "") << std::setprecision(3) << control.thruster_activations[i];
        }
        m_control_file << "]\"";

        for (double val : control.thruster_activations) {
            m_control_file << "," << val << "," << val;
        }
        
        // Log RW Torques
        m_control_file << "," << control.rw_torques.x() << "," << control.rw_torques.y() << "," << control.rw_torques.z();
        
        // Log RW Speeds
        m_control_file << "," << current_state.rw_speeds.x() << "," << current_state.rw_speeds.y() << "," << current_state.rw_speeds.z();
        
        // Log MPC Time
        m_control_file << "," << mpc_time_ms;

        m_control_file << "\n";
    }

    void DataLogger::flush() {
        if (m_physics_file.is_open()) {
            m_physics_file.flush();
        }
        if (m_control_file.is_open()) {
            m_control_file.flush();
        }
    }

    void DataLogger::log_metrics(const SimulationMetrics& metrics) {
        std::filesystem::path m_path = m_sim_dir / "performance_metrics.json";
        std::ofstream out(m_path);
        if (!out.is_open()) {
            std::cerr << "Failed to open metrics file: " << m_path << std::endl;
            return;
        }

        out << "{\n";
        out << "  \"mpc\": {\n";
        out << "    \"solve_count\": " << metrics.mpc_count << ",\n";
        out << "    \"max_ms\": " << metrics.max_mpc_time_ms << ",\n";
        out << "    \"mean_ms\": " << (metrics.mpc_count > 0 ? metrics.total_mpc_time_ms / metrics.mpc_count : 0.0) << "\n";
        out << "  },\n";
        out << "  \"physics\": {\n";
        out << "    \"step_count\": " << metrics.physics_steps << ",\n";
        out << "    \"max_ms\": " << metrics.max_physics_time_ms << ",\n";
        out << "    \"avg_ms\": " << (metrics.physics_steps > 0 ? metrics.total_physics_time_ms / metrics.physics_steps : 0.0) << "\n";
        out << "  },\n";
        out << "  \"simulation\": {\n";
        out << "    \"total_time_s\": " << metrics.total_sim_time_s << ",\n";
        out << "    \"total_cycles\": " << metrics.total_cycles << "\n";
        out << "  }\n";
        out << "}\n";
        out.close();
    }


}
