#pragma once
#include "core/Types.h"
#include <mujoco/mujoco.h>
#include <string>
#include <memory>

namespace sat_sim::physics {

    class PhysicsEngine {
    public:
        /**
         * @brief Initialize MuJoCo physics engine with a model file.
         * @param model_path Path to the XML model file.
         */
        PhysicsEngine(const std::string& model_path);
        
        ~PhysicsEngine();

        // Prevent copying to avoid double-freeing MuJoCo pointers
        PhysicsEngine(const PhysicsEngine&) = delete;
        PhysicsEngine& operator=(const PhysicsEngine&) = delete;

        /**
         * @brief Advance the simulation by one timestep.
         * Applies currently set controls.
         */
        void step();

        /**
         * @brief Apply control inputs to the simulation.
         * Note: This sets the `ctrl` array in m_data. 
         * Depending on the model, this might map to thrusters or generalized forces.
         * For this phase, we might treat it as applying "qfrc_applied" or setting actuator ctrls.
         */
        void set_control_input(const core::ControlInput& u);

        /**
         * @brief Get the current state derived from MuJoCo data.
         */
        core::StateVector get_state() const;

        double get_time() const;
        void reset();

    private:
        mjModel* m_model = nullptr;
        mjData* m_data = nullptr;

        void load_model(const std::string& path);
    };

} // namespace sat_sim::physics
