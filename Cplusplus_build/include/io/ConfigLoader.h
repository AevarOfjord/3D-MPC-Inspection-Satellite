#pragma once
/**
 * @file ConfigLoader.h
 * @brief Configuration loading with Hydra-style hierarchical composition.
 */

#include "config/ConfigSchema.h"
#include "core/Types.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <filesystem>

namespace sat_sim::io {

    class ConfigLoader {
    public:
        /**
         * @brief Load complete configuration from main.yaml.
         * 
         * Resolves defaults and merges sub-configs from the hierarchy.
         * @param main_config_path Path to main.yaml
         * @return Fully populated RootConfig
         */
        static config::RootConfig load(const std::string& main_config_path);

        /**
         * @brief Load vehicle configuration from a YAML file.
         * @param path Path to vehicle yaml file.
         * @return VehicleConfig struct.
         */
        static config::VehicleConfig load_vehicle_config(const std::string& path);

        /**
         * @brief Load MPC control configuration.
         */
        static config::ControlConfig load_control_config(const std::string& path);

        /**
         * @brief Load mission profile (waypoints).
         */
        static config::MissionConfig load_mission_config(const std::string& path);

        /**
         * @brief Load environment configuration.
         */
        static config::EnvironmentConfig load_environment_config(const std::string& path);

        // Legacy compatibility - converts to old core::* types
        static core::VehicleConfig load_vehicle_config_legacy(const std::string& path);
        static core::MissionConfig load_mission_config_legacy(const std::string& path);
        static core::MissionProfile load_mission_profile(const std::string& path);

    private:
        static config::Vector3 parse_vec3(const YAML::Node& node);
        static config::Quaternion parse_quat(const YAML::Node& node);
        static std::filesystem::path resolve_path(const std::string& base_dir, const std::string& relative);
    };

} // namespace sat_sim::io
