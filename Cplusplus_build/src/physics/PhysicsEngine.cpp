#include "physics/PhysicsEngine.h"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <iostream>
#include <cstring> // for strncpy

namespace sat_sim::physics {

    PhysicsEngine::PhysicsEngine(const std::string& model_path) {
        load_model(model_path);
    }

    PhysicsEngine::~PhysicsEngine() {
        if (m_data) mj_deleteData(m_data);
        if (m_model) mj_deleteModel(m_model);
    }

    void PhysicsEngine::load_model(const std::string& path) {
        spdlog::info("Loading MuJoCo Model: {}", path);
        
        char error[1000] = "Could not load binary model";
        m_model = mj_loadXML(path.c_str(), 0, error, 1000);
        
        if (!m_model) {
            spdlog::error("MuJoCo Load Error: {}", error);
            throw std::runtime_error(std::string("MuJoCo Load Error: ") + error);
        }

        m_data = mj_makeData(m_model);
        if (!m_data) {
             throw std::runtime_error("MuJoCo could not allocate data");
        }
        
        spdlog::info("Model Loaded. nv={}, nq={}", m_model->nv, m_model->nq);
    }

    void PhysicsEngine::step() {
        if (m_model && m_data) {
            mj_step(m_model, m_data);
        }
    }

    void PhysicsEngine::set_control_input(const core::ControlInput& u) {
        if (!m_model || !m_data) return;

        // Clear previous generalized forces (qfrc_applied)
        // Note: Do NOT use mj_resetData here as it wipes kinematic state.
        for (int i=0; i < m_model->nv; ++i) m_data->qfrc_applied[i] = 0.0;
        
        // --- Option A: Ideal Force/Torque Application (Debug/Ideal Control) ---
        // Force is usually World Frame in qfrc_applied for free joint translation?
        // Let's verify standard MuJoCo free joint mechanics.
        // qvel[0-2]: World linear velocity.
        // qvel[3-5]: Body angular velocity.
        // Therefore qfrc_applied[0-2] should be World Force.
        // qfrc_applied[3-5] should be Body Torque.

        // We have u.force_body. Need to rotate to World.
        // qpos[3..6] is Quaternion [w, x, y, z].
        double w = m_data->qpos[3];
        double x = m_data->qpos[4];
        double y = m_data->qpos[5];
        double z = m_data->qpos[6];

        Eigen::Quaterniond q(w, x, y, z);
        core::Vector3 f_world = q * u.force_body;

        m_data->qfrc_applied[0] = f_world.x();
        m_data->qfrc_applied[1] = f_world.y();
        m_data->qfrc_applied[2] = f_world.z();

        m_data->qfrc_applied[3] = u.torque_body.x();
        m_data->qfrc_applied[4] = u.torque_body.y();
        m_data->qfrc_applied[5] = u.torque_body.z();

        // --- Option B: Actuators ---
        // If we have actuators defined in XML, we set m_data->ctrl
        // Check if thruster_activations matches nu (number of actuators)
        if (u.thruster_activations.size() > 0) {
            int mapping_limit = std::min((int)u.thruster_activations.size(), m_model->nu);
            for (int i=0; i<mapping_limit; ++i) {
                m_data->ctrl[i] = u.thruster_activations[i];
            }
        }
    }

    core::StateVector PhysicsEngine::get_state() const {
        core::StateVector state;
        if (!m_data) return state;

        // Position (Index 0,1,2)
        state.position.x() = m_data->qpos[0];
        state.position.y() = m_data->qpos[1];
        state.position.z() = m_data->qpos[2];

        // Attitude (Index 3,4,5,6 -> w,x,y,z)
        // Eigen Quaternion constructor is (w, x, y, z)
        state.attitude = core::Quaternion(
            m_data->qpos[3], // w
            m_data->qpos[4], // x
            m_data->qpos[5], // y
            m_data->qpos[6]  // z
        );
        state.attitude.normalize();

        // Linear Velocity (Index 0,1,2 - World Frame usually)
        state.velocity.x() = m_data->qvel[0];
        state.velocity.y() = m_data->qvel[1];
        state.velocity.z() = m_data->qvel[2];

        // Angular Velocity (Index 3,4,5 - Body Frame usually)
        state.angular_velocity.x() = m_data->qvel[3];
        state.angular_velocity.y() = m_data->qvel[4];
        state.angular_velocity.z() = m_data->qvel[5];

        // Reaction Wheel Velocities (Index 6,7,8 - if 3 RWs)
        // Check nv size to be safe, but usually nv=9 for this model (6 base + 3 RW)
        if (m_model->nv >= 9) {
            state.rw_speeds.x() = m_data->qvel[6];
            state.rw_speeds.y() = m_data->qvel[7];
            state.rw_speeds.z() = m_data->qvel[8];
        } else {
            state.rw_speeds.setZero();
        }

        return state;
    }

    double PhysicsEngine::get_time() const {
        return m_data ? m_data->time : 0.0;
    }

    void PhysicsEngine::reset() {
        if (m_model && m_data) {
            mj_resetData(m_model, m_data);
            mj_forward(m_model, m_data);
        }
    }

} // namespace sat_sim::physics
