#pragma once
/**
 * @file FlightCodeGenerator.h
 * @brief Generates standalone C code for flight computer deployment.
 * 
 * Exports the control kernel as dependency-free C code that can be
 * compiled for ARM/FPGA targets without external libraries.
 */

#include "config/ConfigSchema.h"
#include <string>
#include <vector>
#include <fstream>

namespace sat_sim::export_gen {

    /**
     * @brief Target platform for code generation.
     */
    enum class TargetPlatform {
        GENERIC_C,          // Plain C99
        ARM_CORTEX_M,       // ARM Cortex-M optimized
        FPGA_HLS            // High-Level Synthesis compatible
    };

    /**
     * @brief Code generation options.
     */
    struct GeneratorOptions {
        TargetPlatform platform = TargetPlatform::GENERIC_C;
        bool include_comments = true;
        bool use_fixed_point = false;      // Use fixed-point instead of float
        int fixed_point_bits = 16;         // Q16.16 format
        bool inline_matrices = true;       // Inline B-matrix in code
        std::string output_dir = "generated";
        std::string prefix = "sat_ctrl_";  // Function/type prefix
    };

    /**
     * @brief Generates flight-ready C code from configuration.
     */
    class FlightCodeGenerator {
    public:
        explicit FlightCodeGenerator(const GeneratorOptions& opts = GeneratorOptions());

        /**
         * @brief Generate complete controller code from vehicle config.
         * @param vehicle Vehicle configuration.
         * @param mpc_horizon MPC prediction horizon.
         * @param mpc_dt Control timestep.
         * @return True if successful.
         */
        bool generate(const config::VehicleConfig& vehicle,
                      int mpc_horizon = 10,
                      double mpc_dt = 0.05);

        /**
         * @brief Get list of generated files.
         */
        const std::vector<std::string>& get_generated_files() const { return m_files; }

    private:
        GeneratorOptions m_opts;
        std::vector<std::string> m_files;

        void generate_types_header(const config::VehicleConfig& vehicle);
        void generate_control_allocator(const config::VehicleConfig& vehicle);
        void generate_controller(int horizon, double dt);
        void generate_main_interface();
        void generate_makefile();

        std::string type_prefix() const { return m_opts.prefix; }
        std::string float_type() const;
    };

} // namespace sat_sim::export_gen
