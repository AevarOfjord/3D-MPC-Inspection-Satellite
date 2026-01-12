#pragma once

#include "core/Types.h"
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <memory>
#include <mutex>

#include "io/SimulationMetrics.h"

namespace sat_sim::io {

    class DataLogger {
    public:
        /**
         * @brief Initialize DataLogger. Creates a timestamped directory structure.
         */
        DataLogger();
        ~DataLogger();

        /**
         * @brief Log a physics step to physics_data.csv
         * @param time Simulation time [s]
         * @param current_state Current state vector
         * @param target_state Target state vector
         * @param control Applied control input
         */
        void log_physics_step(
            double time,
            const core::StateVector& current_state,
            const core::StateVector& target_state,
            const core::ControlInput& control
        );

        void log_control_step(
            double time,
            const core::StateVector& current_state,
            const core::StateVector& target_state,
            const core::ControlInput& control,
            double mpc_time_ms = 0.0
        );

        /**
         * @brief Log final performance metrics to JSON
         */
        void log_metrics(const SimulationMetrics& metrics);

        /**
         * @brief Write the CSV buffers to disk.
         */
        void flush();

    private:
        std::filesystem::path m_base_path;
        std::filesystem::path m_sim_dir;
        
        std::ofstream m_physics_file;
        bool m_physics_headers_written = false;

        std::ofstream m_control_file;
        bool m_control_headers_written = false;

        // Buffers could be used for performance, but simple direct write is fine for V1
        std::vector<std::string> m_physics_headers;

        void create_directories();
        void init_physics_log();
        
        // Helper to convert quaternion [w,x,y,z] to Euler [roll, pitch, yaw]
        core::Vector3 quat_to_euler(const core::Quaternion& q);
    };

}

