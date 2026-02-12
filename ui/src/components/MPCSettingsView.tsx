import React, { useState, useEffect } from 'react';
import { Save, RotateCcw, AlertCircle, Check, Loader2 } from 'lucide-react';

const API_BASE = 'http://localhost:8000'; // Adjust if needed

interface MPCConfig {
  control: {
    mpc: {
      prediction_horizon: number;
      control_horizon: number;
      weights: {
        Q_contour: number;
        Q_progress: number;
        Q_smooth: number;
        Q_attitude: number;
        thrust: number;
        rw_torque: number;
      };
      settings: {
        dt: number;
        max_linear_velocity: number;
        max_angular_velocity: number;
        enable_collision_avoidance: boolean;
        enable_auto_state_bounds: boolean;
      };
      path_following: {
        path_speed: number;
      }
    };
  };
  sim: {
    dt: number;
    duration: number;
  };
}

export function MPCSettingsView() {
  const [config, setConfig] = useState<MPCConfig | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [successMsg, setSuccessMsg] = useState<string | null>(null);

  useEffect(() => {
    fetchConfig();
  }, []);

  const fetchConfig = async () => {
    setIsLoading(true);
    setError(null);
    try {
      const res = await fetch(`${API_BASE}/runner/config`);
      if (!res.ok) throw new Error('Failed to fetch config');
      const data = await res.json();
      setConfig(data);
    } catch (err) {
      setError(String(err));
    } finally {
      setIsLoading(false);
    }
  };

  const handleSave = async () => {
    if (!config) return;
    setIsSaving(true);
    setError(null);
    setSuccessMsg(null);
    try {
        // We send back the structured overrides. 
        // The backend expects a dictionary that merges into AppConfig.
        // We can just send the whole 'control' and 'sim' sections since we are editing them.
        
        const overrides = {
            control: config.control,
            sim: config.sim
        };

        const res = await fetch(`${API_BASE}/runner/config`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(overrides)
        });
        
        if (!res.ok) throw new Error('Failed to save config');
        
        setSuccessMsg('Configuration saved successfully. Next run will use these settings.');
        setTimeout(() => setSuccessMsg(null), 3000);
    } catch (err) {
        setError("Failed to save: " + String(err));
    } finally {
        setIsSaving(false);
    }
  };

  const updateConfig = (path: string, value: any) => {
    if (!config) return;
    
    // Deep clone to avoid direct mutation
    const newConfig = JSON.parse(JSON.stringify(config));
    
    // Config path traversal
    const parts = path.split('.');
    let current = newConfig;
    for (let i = 0; i < parts.length - 1; i++) {
        current = current[parts[i]];
    }
    
    // Type conversion for numbers
    if (typeof current[parts[parts.length - 1]] === 'number') {
        const numVal = parseFloat(value);
        if (!isNaN(numVal)) {
            current[parts[parts.length - 1]] = numVal;
        }
    } else if (typeof current[parts[parts.length - 1]] === 'boolean') {
        current[parts[parts.length - 1]] = value;
    } else {
        current[parts[parts.length - 1]] = value;
    }
    
    setConfig(newConfig);
  };

  if (isLoading) {
    return (
        <div className="h-full flex items-center justify-center text-slate-400">
            <Loader2 className="animate-spin mr-2" /> Loading configuration...
        </div>
    );
  }

  if (error && !config) {
      return (
          <div className="h-full flex flex-col items-center justify-center text-red-400">
              <AlertCircle className="mb-2" size={32} />
              <p>{error}</p>
              <button 
                onClick={fetchConfig}
                className="mt-4 px-4 py-2 bg-slate-800 rounded hover:bg-slate-700 transition"
              >
                  Retry
              </button>
          </div>
      );
  }

  return (
    <div className="h-full flex flex-col bg-slate-950 text-slate-200 overflow-hidden">
      {/* Header */}
      <div className="flex-none p-4 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
          <div>
              <h2 className="text-lg font-bold text-white">MPC Settings</h2>
              <p className="text-xs text-slate-400">Configure solver parameters and limits</p>
          </div>
          
          <div className="flex gap-2">
              <button
                onClick={fetchConfig}
                className="flex items-center gap-2 px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-sm transition"
              >
                  <RotateCcw size={14} /> Reset
              </button>
              <button
                onClick={handleSave}
                disabled={isSaving}
                className="flex items-center gap-2 px-4 py-1.5 rounded bg-blue-600 hover:bg-blue-500 text-white font-semibold text-sm transition shadow-sm disabled:opacity-50"
              >
                  {isSaving ? <Loader2 size={14} className="animate-spin" /> : <Save size={14} />}
                  Save Changes
              </button>
          </div>
      </div>

      {/* Content */}
      <div className="flex-1 overflow-y-auto p-6">
          <div className="max-w-4xl mx-auto space-y-8">
              
              {/* Messages */}
              {error && (
                  <div className="p-3 bg-red-900/20 border border-red-800 rounded text-red-200 text-sm flex items-center gap-2">
                      <AlertCircle size={16} /> {error}
                  </div>
              )}
              {successMsg && (
                  <div className="p-3 bg-green-900/20 border border-green-800 rounded text-green-200 text-sm flex items-center gap-2">
                      <Check size={16} /> {successMsg}
                  </div>
              )}

              {/* General Simulation Settings */}
              <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                      Simulation & Time
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                      <ConfigField 
                        label="Simulation Duration (s)" 
                        value={config?.sim?.duration} 
                        onChange={(v) => updateConfig('sim.duration', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Control Step (dt)" 
                        value={config?.control?.mpc?.settings?.dt} 
                        onChange={(v) => updateConfig('control.mpc.settings.dt', v)}
                        isNumber
                        step={0.01}
                      />
                  </div>
              </section>

              {/* Horizons */}
              <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                      Horizons
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                      <ConfigField 
                        label="Prediction Horizon (Steps)" 
                        value={config?.control?.mpc?.prediction_horizon} 
                        onChange={(v) => updateConfig('control.mpc.prediction_horizon', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Control Horizon (Steps)" 
                        value={config?.control?.mpc?.control_horizon} 
                        onChange={(v) => updateConfig('control.mpc.control_horizon', v)}
                        isNumber
                      />
                  </div>
              </section>

              {/* Cost Weights */}
              <section>
                  <h3 className="text-sm uppercase tracking-wider text-blue-400 font-bold mb-4 border-b border-blue-900/30 pb-1">
                      Objective Weights (Tuning)
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                      <ConfigField 
                        label="Contour Error (Q_contour)" 
                        value={config?.control?.mpc?.weights?.Q_contour} 
                        onChange={(v) => updateConfig('control.mpc.weights.Q_contour', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Progress (Q_progress)" 
                        value={config?.control?.mpc?.weights?.Q_progress} 
                        onChange={(v) => updateConfig('control.mpc.weights.Q_progress', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Attitude Alignment (Q_attitude)" 
                        value={config?.control?.mpc?.weights?.Q_attitude} 
                        onChange={(v) => updateConfig('control.mpc.weights.Q_attitude', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Input Smoothness (Q_smooth)" 
                        value={config?.control?.mpc?.weights?.Q_smooth} 
                        onChange={(v) => updateConfig('control.mpc.weights.Q_smooth', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="Thrust Usage (R_thrust)" 
                        value={config?.control?.mpc?.weights?.thrust} 
                        onChange={(v) => updateConfig('control.mpc.weights.thrust', v)}
                        isNumber
                      />
                      <ConfigField 
                        label="RW Torque Usage (R_rw_torque)" 
                        value={config?.control?.mpc?.weights?.rw_torque} 
                        onChange={(v) => updateConfig('control.mpc.weights.rw_torque', v)}
                        isNumber
                      />
                  </div>
              </section>

               {/* Constraints */}
              <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                      Constraints & Limits
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                      <ConfigField 
                        label="Max Linear Velocity (m/s)" 
                        value={config?.control?.mpc?.settings?.max_linear_velocity} 
                        onChange={(v) => updateConfig('control.mpc.settings.max_linear_velocity', v)}
                        isNumber
                        desc="0 = Auto"
                      />
                      <ConfigField 
                        label="Max Angular Velocity (rad/s)" 
                        value={config?.control?.mpc?.settings?.max_angular_velocity} 
                        onChange={(v) => updateConfig('control.mpc.settings.max_angular_velocity', v)}
                        isNumber
                        desc="0 = Auto"
                      />
                      <div className="flex items-center justify-between p-3 bg-slate-900 rounded border border-slate-800">
                          <span className="text-sm font-medium text-slate-300">Enable Collision Avoidance</span>
                          <input 
                              type="checkbox" 
                              checked={config?.control?.mpc?.settings?.enable_collision_avoidance || false} 
                              onChange={(e) => updateConfig('control.mpc.settings.enable_collision_avoidance', e.target.checked)}
                              className="w-5 h-5 rounded border-slate-600 bg-slate-700 text-blue-600 focus:ring-offset-slate-900"
                          />
                      </div>
                       <div className="flex items-center justify-between p-3 bg-slate-900 rounded border border-slate-800">
                          <span className="text-sm font-medium text-slate-300">Auto State Bounds</span>
                          <input 
                              type="checkbox" 
                              checked={config?.control?.mpc?.settings?.enable_auto_state_bounds || false} 
                              onChange={(e) => updateConfig('control.mpc.settings.enable_auto_state_bounds', e.target.checked)}
                              className="w-5 h-5 rounded border-slate-600 bg-slate-700 text-blue-600 focus:ring-offset-slate-900"
                          />
                      </div>
                  </div>
              </section>
              
               {/* Path Following */}
              <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                     Path Following
                  </h3>
                   <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                      <ConfigField 
                        label="Reference Path Speed (m/s)" 
                        value={config?.control?.mpc?.path_following?.path_speed} 
                        onChange={(v) => updateConfig('control.mpc.path_following.path_speed', v)}
                        isNumber
                        step={0.01}
                      />
                   </div>
              </section>

          </div>
      </div>
    </div>
  );
}

interface ConfigFieldProps {
    label: string;
    value: any;
    onChange: (value: any) => void;
    isNumber?: boolean;
    desc?: string;
    step?: number;
}

function ConfigField({ label, value, onChange, isNumber, desc, step }: ConfigFieldProps) {
    return (
        <div className="flex flex-col gap-1">
            <label className="text-xs font-semibold text-slate-400 uppercase">{label}</label>
            <input 
                type={isNumber ? "number" : "text"}
                step={step || (isNumber ? 1 : undefined)}
                value={value ?? ''}
                onChange={(e) => onChange(e.target.value)}
                className="bg-slate-900 border border-slate-700 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 transition-colors"
            />
            {desc && <span className="text-[10px] text-slate-500">{desc}</span>}
        </div>
    );
}
