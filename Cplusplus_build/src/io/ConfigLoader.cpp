#include "io/ConfigLoader.h"
#include <spdlog/spdlog.h>
#include <iostream>

namespace sat_sim::io {

    using namespace sat_sim::config;
    namespace fs = std::filesystem;

    // ============================================================================
    // UTILITY FUNCTIONS
    // ============================================================================

    Vector3 ConfigLoader::parse_vec3(const YAML::Node& node) {
        if (!node.IsSequence() || node.size() != 3) {
            throw std::runtime_error("Invalid Vector3 in YAML");
        }
        return Vector3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    }

    Quaternion ConfigLoader::parse_quat(const YAML::Node& node) {
        if (!node.IsSequence() || node.size() != 4) {
            return Quaternion(1, 0, 0, 0); // Default identity
        }
        // [w, x, y, z] convention
        return Quaternion(node[0].as<double>(), node[1].as<double>(), 
                         node[2].as<double>(), node[3].as<double>());
    }

    fs::path ConfigLoader::resolve_path(const std::string& base_dir, const std::string& relative) {
        fs::path base(base_dir);
        fs::path rel(relative);
        return base / rel;
    }

    // ============================================================================
    // MAIN LOADER - Hydra-style composition
    // ============================================================================

    RootConfig ConfigLoader::load(const std::string& main_config_path) {
        spdlog::info("Loading configuration from: {}", main_config_path);
        
        fs::path main_path(main_config_path);
        fs::path config_dir = main_path.parent_path();
        
        YAML::Node main_node = YAML::LoadFile(main_config_path);
        RootConfig root;

        // Parse defaults section to determine which sub-configs to load
        if (main_node["defaults"] && main_node["defaults"].IsSequence()) {
            for (const auto& default_item : main_node["defaults"]) {
                for (auto it = default_item.begin(); it != default_item.end(); ++it) {
                    std::string key = it->first.as<std::string>();
                    std::string value = it->second.as<std::string>();
                    
                    fs::path sub_config = config_dir / key / (value + ".yaml");
                    
                    if (fs::exists(sub_config)) {
                        spdlog::debug("Loading sub-config: {}", sub_config.string());
                        
                        if (key == "vehicle") {
                            root.vehicle = load_vehicle_config(sub_config.string());
                        } else if (key == "control/mpc") {
                            root.control = load_control_config(sub_config.string());
                        } else if (key == "mission") {
                            root.mission = load_mission_config(sub_config.string());
                        } else if (key == "env") {
                            root.environment = load_environment_config(sub_config.string());
                        }
                    } else {
                        spdlog::warn("Sub-config not found: {}", sub_config.string());
                    }
                }
            }
        }

        // Override with values directly in main.yaml
        if (main_node["simulation"]) {
            auto sim = main_node["simulation"];
            if (sim["model_file"]) root.simulation.model_file = sim["model_file"].as<std::string>();
            if (sim["physics_dt"]) root.simulation.physics_dt = sim["physics_dt"].as<double>();
            if (sim["control_dt"]) root.simulation.control_dt = sim["control_dt"].as<double>();
            if (sim["headless"]) root.simulation.headless = sim["headless"].as<bool>();
        }

        if (main_node["config_version"]) {
            root.config_version = main_node["config_version"].as<std::string>();
        }

        spdlog::info("Configuration loaded: vehicle='{}', mission='{}'", 
                     root.vehicle.name, root.mission.name);
        return root;
    }

    // ============================================================================
    // VEHICLE CONFIG
    // ============================================================================

    VehicleConfig ConfigLoader::load_vehicle_config(const std::string& path) {
        spdlog::info("Loading Vehicle Config: {}", path);
        
        YAML::Node config = YAML::LoadFile(path);
        VehicleConfig cfg;
        
        const auto& v = config["vehicle"];
        if (!v) throw std::runtime_error("Missing 'vehicle' key in config: " + path);

        cfg.name = v["name"] ? v["name"].as<std::string>() : "Unnamed";
        cfg.mass = v["mass"] ? v["mass"].as<double>() : 10.0;
        
        if (v["inertia_diag"]) {
            cfg.inertia_diag = parse_vec3(v["inertia_diag"]);
        } else if (v["inertia"]) {
            cfg.inertia_diag = parse_vec3(v["inertia"]);
        }
        
        if (v["center_of_mass"]) {
            cfg.center_of_mass = parse_vec3(v["center_of_mass"]);
        }

        // Reaction Wheels
        if (v["reaction_wheels"]) {
            const auto& rw = v["reaction_wheels"];
            cfg.reaction_wheels.max_torque = rw["max_torque"] ? rw["max_torque"].as<double>() : 0.1;
            cfg.reaction_wheels.max_speed_rad_s = rw["max_speed_rad_s"] ? rw["max_speed_rad_s"].as<double>() : 600.0;
            cfg.reaction_wheels.inertia = rw["inertia"] ? rw["inertia"].as<double>() : 0.001;
            cfg.reaction_wheels.enabled = rw["enabled"] ? rw["enabled"].as<bool>() : true;
        }

        // Thrusters
        if (v["thrusters"] && v["thrusters"].IsSequence()) {
            for (const auto& t_node : v["thrusters"]) {
                ThrusterConfig t;
                t.id = t_node["id"] ? t_node["id"].as<std::string>() : "T?";
                t.position = parse_vec3(t_node["position"]);
                t.direction = parse_vec3(t_node["direction"]);
                t.max_thrust = t_node["max_thrust"] ? t_node["max_thrust"].as<double>() : 1.0;
                t.min_thrust = t_node["min_thrust"] ? t_node["min_thrust"].as<double>() : 0.0;
                t.group = t_node["group"] ? t_node["group"].as<std::string>() : "main";
                cfg.thrusters.push_back(t);
            }
        }

        if (v["mesh_file"]) {
            cfg.mesh_file = v["mesh_file"].as<std::string>();
        }

        spdlog::info("Loaded Vehicle '{}' with {} thrusters", cfg.name, cfg.thrusters.size());
        return cfg;
    }

    // ============================================================================
    // CONTROL CONFIG
    // ============================================================================

    ControlConfig ConfigLoader::load_control_config(const std::string& path) {
        YAML::Node config = YAML::LoadFile(path);
        ControlConfig ctrl;
        
        if (config["control"] && config["control"]["mpc"]) {
            const auto& mpc = config["control"]["mpc"];
            ctrl.mpc.horizon = mpc["horizon"] ? mpc["horizon"].as<int>() : 20;
            ctrl.mpc.dt = mpc["dt"] ? mpc["dt"].as<double>() : 0.05;
            
            if (mpc["weights"]) {
                const auto& w = mpc["weights"];
                ctrl.mpc.weights.position = w["position"] ? w["position"].as<double>() : 100.0;
                ctrl.mpc.weights.velocity = w["velocity"] ? w["velocity"].as<double>() : 1.0;
                ctrl.mpc.weights.attitude = w["attitude"] ? w["attitude"].as<double>() : 10.0;
                ctrl.mpc.weights.angular_velocity = w["angular_velocity"] ? w["angular_velocity"].as<double>() : 1.0;
                ctrl.mpc.weights.thrust = w["thrust"] ? w["thrust"].as<double>() : 0.1;
                ctrl.mpc.weights.reaction_wheel = w["reaction_wheel"] ? w["reaction_wheel"].as<double>() : 1.0;
            }
            
            ctrl.mpc.max_iterations = mpc["max_iterations"] ? mpc["max_iterations"].as<int>() : 1000;
            ctrl.mpc.tolerance = mpc["tolerance"] ? mpc["tolerance"].as<double>() : 1e-4;
            ctrl.mpc.verbose = mpc["verbose"] ? mpc["verbose"].as<bool>() : false;
        }
        
        return ctrl;
    }

    // ============================================================================
    // MISSION CONFIG
    // ============================================================================

    MissionConfig ConfigLoader::load_mission_config(const std::string& path) {
        YAML::Node config = YAML::LoadFile(path);
        MissionConfig mission;
        
        if (config["mission"]) {
            const auto& m = config["mission"];
            mission.name = m["name"] ? m["name"].as<std::string>() : "Unnamed";
            mission.max_duration = m["max_duration"] ? m["max_duration"].as<double>() : 300.0;
            mission.loop = m["loop"] ? m["loop"].as<bool>() : false;
            
            if (m["waypoints"] && m["waypoints"].IsSequence()) {
                for (const auto& wp_node : m["waypoints"]) {
                    Waypoint wp;
                    wp.position = parse_vec3(wp_node["position"]);
                    wp.attitude = wp_node["attitude"] ? parse_quat(wp_node["attitude"]) : Quaternion(1, 0, 0, 0);
                    wp.hold_time = wp_node["hold_time"] ? wp_node["hold_time"].as<double>() : 0.0;
                    wp.pos_tolerance = wp_node["pos_tolerance"] ? wp_node["pos_tolerance"].as<double>() : 0.05;
                    wp.att_tolerance = wp_node["att_tolerance"] ? wp_node["att_tolerance"].as<double>() : 0.1;
                    wp.vel_tolerance = wp_node["vel_tolerance"] ? wp_node["vel_tolerance"].as<double>() : 0.05;
                    wp.ang_vel_tolerance = wp_node["ang_vel_tolerance"] ? wp_node["ang_vel_tolerance"].as<double>() : 0.05;
                    mission.waypoints.push_back(wp);
                }
            }
        }
        
        spdlog::info("Loaded Mission '{}' with {} waypoints", mission.name, mission.waypoints.size());
        return mission;
    }

    // ============================================================================
    // ENVIRONMENT CONFIG
    // ============================================================================

    EnvironmentConfig ConfigLoader::load_environment_config(const std::string& path) {
        YAML::Node config = YAML::LoadFile(path);
        EnvironmentConfig env;
        
        if (config["environment"]) {
            const auto& e = config["environment"];
            env.name = e["name"] ? e["name"].as<std::string>() : "default";
            env.gravity = e["gravity"] ? e["gravity"].as<double>() : 0.0;
            env.orbital_altitude_km = e["orbital_altitude_km"] ? e["orbital_altitude_km"].as<double>() : 0.0;
            env.mean_motion = e["mean_motion"] ? e["mean_motion"].as<double>() : 0.0;
            env.enable_drag = e["enable_drag"] ? e["enable_drag"].as<bool>() : false;
            env.enable_j2 = e["enable_j2"] ? e["enable_j2"].as<bool>() : false;
            env.enable_gravity_gradient = e["enable_gravity_gradient"] ? e["enable_gravity_gradient"].as<bool>() : false;
        }
        
        return env;
    }

    // ============================================================================
    // LEGACY COMPATIBILITY - Convert to old core::* types
    // ============================================================================

    core::VehicleConfig ConfigLoader::load_vehicle_config_legacy(const std::string& path) {
        auto cfg = load_vehicle_config(path);
        
        core::VehicleConfig legacy;
        legacy.name = cfg.name;
        legacy.mass = cfg.mass;
        legacy.inertia_diag = cfg.inertia_diag;
        legacy.rw_config.max_torque = cfg.reaction_wheels.max_torque;
        legacy.rw_config.max_speed_rad_s = cfg.reaction_wheels.max_speed_rad_s;
        
        for (const auto& t : cfg.thrusters) {
            core::ThrusterConfig tc;
            tc.id = t.id;
            tc.position = t.position;
            tc.direction = t.direction;
            tc.max_thrust = t.max_thrust;
            legacy.thrusters.push_back(tc);
        }
        
        return legacy;
    }

    core::MissionConfig ConfigLoader::load_mission_config_legacy(const std::string& path) {
        YAML::Node config = YAML::LoadFile(path);
        core::MissionConfig m;
        
        if (config["control"]) {
            m.dt = config["control"]["dt"] ? config["control"]["dt"].as<double>() : 0.1;
            m.horizon = config["control"]["horizon"] ? config["control"]["horizon"].as<int>() : 10;
        } else if (config["control"] && config["control"]["mpc"]) {
            const auto& mpc = config["control"]["mpc"];
            m.dt = mpc["dt"] ? mpc["dt"].as<double>() : 0.1;
            m.horizon = mpc["horizon"] ? mpc["horizon"].as<int>() : 10;
        } else {
            m.dt = 0.1;
            m.horizon = 10;
        }
        
        return m;
    }

    core::MissionProfile ConfigLoader::load_mission_profile(const std::string& path) {
        auto cfg = load_mission_config(path);
        
        core::MissionProfile profile;
        profile.name = cfg.name;
        
        for (const auto& wp : cfg.waypoints) {
            core::Waypoint cwp;
            cwp.position = wp.position;
            cwp.attitude = wp.attitude;
            cwp.hold_time = wp.hold_time;
            cwp.pos_tolerance = wp.pos_tolerance;
            cwp.att_tolerance = wp.att_tolerance;
            cwp.vel_tolerance = wp.vel_tolerance;
            cwp.ang_vel_tolerance = wp.ang_vel_tolerance;
            profile.waypoints.push_back(cwp);
        }
        
        return profile;
    }

} // namespace sat_sim::io
