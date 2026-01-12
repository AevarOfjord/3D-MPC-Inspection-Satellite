#include "control/PulseWidthModulator.h"
#include <algorithm>
#include <cmath>

namespace sat_sim::control {

    PulseWidthModulator::PulseWidthModulator(double min_pulse_width)
        : m_min_pulse_width(min_pulse_width), m_num_thrusters(0) {
    }

    void PulseWidthModulator::update(const std::vector<double>& requested_activations, double control_dt, double current_time) {
        m_num_thrusters = requested_activations.size();
        m_schedule.resize(m_num_thrusters);

        for (int i = 0; i < m_num_thrusters; ++i) {
            double activation = std::clamp(requested_activations[i], 0.0, 1.0);
            double on_duration = activation * control_dt;

            if (on_duration < m_min_pulse_width && on_duration > 1e-6) {
                // Determine logic for small pulses. 
                // For now, if it's very small, ignore it. 
                // If it's close to min, maybe round up? 
                // Let's implement strict deadband for now.
                on_duration = 0.0;
            }

            if (on_duration > 0.0) {
                // Centered PWM
                double mid_time = current_time + (control_dt / 2.0);
                m_schedule[i].start_time = mid_time - (on_duration / 2.0);
                m_schedule[i].end_time = mid_time + (on_duration / 2.0);
                m_schedule[i].active = true;
            } else {
                m_schedule[i].active = false;
            }
        }
    }

    std::vector<double> PulseWidthModulator::get_instantaneous_commands(double time) const {
        std::vector<double> commands(m_num_thrusters, 0.0);
        for (int i = 0; i < m_num_thrusters; ++i) {
            if (m_schedule[i].active) {
                if (time >= m_schedule[i].start_time && time < m_schedule[i].end_time) {
                    commands[i] = 1.0;
                }
            }
        }
        return commands;
    }

}
