#pragma once
#include <vector>
#include <map>
#include "core/Types.h"

namespace sat_sim::control {

    class PulseWidthModulator {
    public:
        PulseWidthModulator(double min_pulse_width = 0.01);

        // Update the PWM schedule based on requested activations (0.0 to 1.0)
        // and the duration of the control cycle.
        void update(const std::vector<double>& requested_activations, double control_dt, double current_time);

        // Debug helper
        bool is_any_active() const {
            for(const auto& p : m_schedule) if(p.active) return true;
            return false;
        }

    public:
        // Returns vector of doubles (0.0 or 1.0) to match existing interfaces.
        std::vector<double> get_instantaneous_commands(double time) const;

    private:
        struct Pulse {
            double start_time;
            double end_time;
            bool active;
        };

        double m_min_pulse_width; 
        std::vector<Pulse> m_schedule; // One pulse schedule per thruster
        int m_num_thrusters;
    };

}
