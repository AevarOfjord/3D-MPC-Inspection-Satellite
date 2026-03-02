import type {
  MpcCoreSettings,
  MpcSettings,
  SimulationSettings,
  SettingReferenceSection,
} from './mpcSettingsTypes';

export const DEFAULT_MPC_SETTINGS: MpcSettings = {
  prediction_horizon: 50,
  control_horizon: 40,
  dt: 0.05,
  solver_time_limit: 0.035,
  solver_type: 'OSQP',
  Q_contour: 2400.0,
  Q_progress: 70.0,
  progress_reward: 0.0,
  Q_lag: 0.0,
  Q_lag_default: 4000.0,
  Q_velocity_align: 120.0,
  Q_s_anchor: 500.0,
  Q_smooth: 20.0,
  Q_attitude: 3500.0,
  Q_axis_align: 3000.0,
  Q_terminal_pos: 0.0,
  Q_terminal_s: 0.0,
  q_angular_velocity: 1200.0,
  r_thrust: 0.02,
  r_rw_torque: 0.003,
  thrust_l1_weight: 0.02,
  thrust_pair_weight: 0.8,
  thruster_type: 'CON',
  verbose_mpc: false,
  obstacle_margin: 0.1,
  enable_collision_avoidance: false,
  path_speed: 0.08,
  path_speed_min: 0.0,
  path_speed_max: 0.08,
  enable_thruster_hysteresis: true,
  thruster_hysteresis_on: 0.015,
  thruster_hysteresis_off: 0.007,
  max_linear_velocity: 0.0,
  max_angular_velocity: 0.0,
  enable_delta_u_coupling: false,
  enable_gyro_jacobian: false,
  enable_auto_state_bounds: false,
};

export const DEFAULT_SIMULATION_SETTINGS: SimulationSettings = {
  dt: 0.001,
  max_duration: 0.0,
  control_dt: 0.05,
};

export const DEFAULT_MPC_CORE_SETTINGS: MpcCoreSettings = {
  controller_profile: 'hybrid',
  solver_backend: 'OSQP',
};

export const MPC_CANONICAL_KEYS = new Set(Object.keys(DEFAULT_MPC_SETTINGS));

export const SETTING_REFERENCE_SECTIONS: SettingReferenceSection[] = [
  {
    title: 'Basic - Timing and Horizons',
    items: [
      {
        key: 'simulation.max_duration',
        label: 'Simulation Duration (s)',
        description: 'Maximum run time before simulation stop. A value of 0 disables duration-based stopping.',
        impact: 'Higher values allow long missions; lower values stop early for faster iteration.',
      },
      {
        key: 'mpc.dt',
        label: 'Control Step dt (s)',
        description: 'MPC update interval. This is synced to simulation control_dt when saved.',
        impact: 'Smaller dt reacts faster but increases compute load.',
      },
      {
        key: 'mpc.prediction_horizon',
        label: 'Prediction Horizon',
        description: 'Number of future timesteps the optimizer predicts.',
        impact: 'Longer horizon improves foresight but raises solve time.',
      },
      {
        key: 'mpc.control_horizon',
        label: 'Control Horizon',
        description: 'Number of steps with independent control actions (must be <= prediction horizon).',
        impact: 'Lower values smooth commands and reduce optimization complexity.',
      },
      {
        key: 'mpc.solver_time_limit',
        label: 'Solver Time Limit (s)',
        description: 'Maximum per-step optimizer runtime budget.',
        impact: 'Lower values improve real-time reliability but may reduce solution quality.',
      },
    ],
  },
  {
    title: 'Basic - Tracking Weights',
    items: [
      {
        key: 'mpc.Q_contour',
        label: 'Contour Error (Q_contour)',
        description: 'Penalty on distance from the reference path.',
        impact: 'Higher values force tighter path adherence.',
      },
      {
        key: 'mpc.Q_progress',
        label: 'Progress (Q_progress)',
        description: 'Penalty tied to path progress-speed tracking.',
        impact: 'Higher values prioritize consistent forward progress.',
      },
      {
        key: 'mpc.Q_attitude',
        label: 'Attitude (Q_attitude)',
        description: 'Penalty on orientation error relative to path tangent behavior.',
        impact: 'Higher values keep body alignment tighter.',
      },
      {
        key: 'mpc.Q_smooth',
        label: 'Smoothness (Q_smooth)',
        description: 'Penalty on control increments (delta-u).',
        impact: 'Higher values reduce command jitter but can slow response.',
      },
      {
        key: 'mpc.q_angular_velocity',
        label: 'Angular Velocity (q_angular_velocity)',
        description: 'Penalty on rotational rate in the cost function.',
        impact: 'Higher values damp angular motion and reduce spin.',
      },
    ],
  },
  {
    title: 'Basic - Actuation and Path',
    items: [
      {
        key: 'mpc.r_thrust',
        label: 'Thrust Cost (r_thrust)',
        description: 'Penalty on thruster usage.',
        impact: 'Higher values save thrust/fuel but can slow translational response.',
      },
      {
        key: 'mpc.r_rw_torque',
        label: 'RW Torque Cost (r_rw_torque)',
        description: 'Penalty on reaction wheel torque usage.',
        impact: 'Higher values reduce wheel effort but may weaken attitude authority.',
      },
      {
        key: 'mpc.path_speed',
        label: 'Path Speed (m/s)',
        description: 'Nominal desired progress speed along path.',
        impact: 'Higher values speed mission completion but demand more control effort.',
      },
    ],
  },
  {
    title: 'Advanced Settings',
    items: [
      {
        key: 'mpc.Q_lag',
        label: 'Lag Error (Q_lag)',
        description: 'Penalty on along-track lag relative to the path parameterization.',
        impact: 'Higher values reduce behind-path drift.',
      },
      {
        key: 'mpc.Q_lag_default',
        label: 'Lag Default (Q_lag_default)',
        description: 'Fallback lag weight when Q_lag <= 0. Use -1 to keep auto behavior.',
        impact: 'Controls default lag coupling without forcing it to contour weight.',
      },
      {
        key: 'mpc.Q_velocity_align',
        label: 'Velocity Align (Q_velocity_align)',
        description: 'Weight for velocity alignment along path tangent. 0 reuses Q_progress.',
        impact: 'Separates speed-tracking pressure from velocity-direction tracking.',
      },
      {
        key: 'mpc.Q_s_anchor',
        label: 'S Anchor (Q_s_anchor)',
        description: 'Weight anchoring path parameter state to the linearization reference. -1 keeps auto behavior.',
        impact: 'Lets you tune path-parameter anchoring strength explicitly.',
      },
      {
        key: 'mpc.Q_axis_align',
        label: 'Axis Align (Q_axis_align)',
        description: 'Extra alignment weight added on top of Q_attitude.',
        impact: 'Increase to bias stronger body-axis alignment without changing other attitude tuning.',
      },
      {
        key: 'mpc.path_speed_min',
        label: 'Path Speed Min (m/s)',
        description: 'Lower clamp for path progress speed.',
        impact: 'Higher minimum prevents stalling but can reduce precision near hard sections.',
      },
      {
        key: 'mpc.path_speed_max',
        label: 'Path Speed Max (m/s)',
        description: 'Upper clamp for path progress speed.',
        impact: 'Lower maximum limits aggressiveness and compute stress.',
      },
      {
        key: 'mpc.Q_terminal_pos',
        label: 'Terminal Position (Q_terminal_pos)',
        description: 'Terminal-stage position weight. 0 means automatic scaling.',
        impact: 'Higher values enforce stronger endpoint position convergence.',
      },
      {
        key: 'mpc.Q_terminal_s',
        label: 'Terminal Progress (Q_terminal_s)',
        description: 'Terminal-stage progress/path-parameter weight. 0 means automatic scaling.',
        impact: 'Higher values enforce stronger endpoint progress completion.',
      },
      {
        key: 'mpc.progress_reward',
        label: 'Progress Reward',
        description: 'Linear reward term for moving forward along the path.',
        impact: 'Higher values bias toward faster forward motion.',
      },
      {
        key: 'mpc.max_linear_velocity',
        label: 'Max Linear Velocity (m/s)',
        description: 'State bound for translational speed. 0 means automatic bound behavior.',
        impact: 'Lower bounds reduce aggressive motion and increase safety margin.',
      },
      {
        key: 'mpc.max_angular_velocity',
        label: 'Max Angular Velocity (rad/s)',
        description: 'State bound for rotational speed. 0 means automatic bound behavior.',
        impact: 'Lower bounds reduce spin and improve stability.',
      },
      {
        key: 'mpc.obstacle_margin',
        label: 'Obstacle Margin (m)',
        description: 'Extra safety clearance around modeled obstacles.',
        impact: 'Higher margin is safer but may make pathing more conservative.',
      },
      {
        key: 'mpc.enable_auto_state_bounds',
        label: 'Enable Auto State Bounds',
        description: 'Automatically derives velocity bounds when explicit bounds are unset.',
        impact: 'Improves robustness when limits are left at defaults.',
      },
      {
        key: 'mpc.enable_collision_avoidance',
        label: 'Enable Collision Avoidance',
        description: 'Enables online obstacle constraints in MPC.',
        impact: 'Improves safety around obstacles but can increase solver complexity.',
      },
      {
        key: 'mpc.thruster_type',
        label: 'Thruster Type',
        description: 'Thruster actuation mode: CON for continuous or PWM for binary-like behavior.',
        impact: 'Changes control style and can alter smoothness/realism tradeoffs.',
      },
      {
        key: 'mpc.solver_type',
        label: 'Solver',
        description: 'Optimizer backend selector.',
        impact: 'Currently OSQP is the supported runtime path in this stack.',
      },
      {
        key: 'mpc.enable_delta_u_coupling',
        label: 'Enable Delta-U Coupling',
        description: 'Uses full temporal coupling for smoothness cost terms.',
        impact: 'Can improve smoothness fidelity but increases solve load.',
      },
      {
        key: 'mpc.enable_gyro_jacobian',
        label: 'Enable Gyro Jacobian',
        description: 'Includes gyroscopic Jacobian terms in angular linearization.',
        impact: 'Improves high-rate rotational accuracy with extra computation.',
      },
      {
        key: 'mpc.verbose_mpc',
        label: 'Verbose MPC Solver Logs',
        description: 'Enables detailed solver logging output.',
        impact: 'Useful for tuning/debugging; increases log noise.',
      },
    ],
  },
  {
    title: 'Expert Settings',
    items: [
      {
        key: 'mpc.thrust_l1_weight',
        label: 'Thruster L1 Weight',
        description: 'Linear thruster usage penalty (fuel-bias term).',
        impact: 'Higher values promote coasting and sparse thrust usage.',
      },
      {
        key: 'mpc.thrust_pair_weight',
        label: 'Thruster Pair Weight',
        description: 'Penalty on opposing thrusters firing together.',
        impact: 'Higher values discourage wasteful opposing pair usage.',
      },
    ],
  },
];
