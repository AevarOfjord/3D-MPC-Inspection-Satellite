import { useEffect, useState } from 'react';
import { Activity } from 'lucide-react';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { HudPanel } from './HudComponents';

// ── helpers ──────────────────────────────────────────────────────────────────

const MODE_STYLES: Record<string, string> = {
  TRACK:    'bg-indigo-600/80 text-indigo-100',
  RECOVER:  'bg-amber-600/80 text-amber-100',
  SETTLE:   'bg-sky-600/80 text-sky-100',
  HOLD:     'bg-emerald-600/80 text-emerald-100',
  COMPLETE: 'bg-green-600/80 text-green-100',
};

function ModeBadge({ label }: { label: string }) {
  const style = MODE_STYLES[label.toUpperCase()] ?? 'bg-slate-600/80 text-slate-100';
  return (
    <span className={`px-1.5 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider ${style}`}>
      {label}
    </span>
  );
}

function SolverDot({ status }: { status: string }) {
  const color =
    status === 'ok' ? 'bg-green-400 shadow-[0_0_6px_rgba(74,222,128,0.8)]' :
    status === 'degraded' ? 'bg-yellow-400 shadow-[0_0_6px_rgba(250,204,21,0.8)]' :
    'bg-red-400 shadow-[0_0_6px_rgba(248,113,113,0.8)]';
  return <span className={`inline-block w-2.5 h-2.5 rounded-full ${color} flex-shrink-0`} />;
}

function GateFlag({ label, ok }: { label: string; ok: boolean }) {
  return (
    <div className="flex items-center justify-between bg-slate-900/40 rounded px-1 py-0.5">
      <span className="text-slate-400 font-bold text-[10px]">{label}</span>
      <span className={`text-sm leading-none ${ok ? 'text-green-400' : 'text-red-400'}`}>●</span>
    </div>
  );
}

function ThrusterBar({ value, label }: { value: number; label: string }) {
  const active = value > 0.01;
  return (
    <div className="flex flex-col items-center gap-1">
      <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
        <div
          className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${
            active ? 'bg-orange-500 shadow-[0_0_8px_rgba(249,115,22,0.8)]' : 'bg-slate-700'
          }`}
          style={{ height: `${Math.min(value * 100, 100)}%` }}
        />
      </div>
      <span className={`text-[8px] font-mono ${active ? 'text-orange-400 font-bold' : 'text-slate-600'}`}>
        {label}
      </span>
    </div>
  );
}

function ReactionWheelBar({ value, label }: { value: number; label: string }) {
  const active = Math.abs(value) > 0.00001;
  const heightPct = active
    ? Math.min(Math.max((Math.log10(Math.abs(value) * 1000 + 1) / 3) * 100, 15), 100)
    : 0;
  return (
    <div className="flex flex-col items-center gap-1">
      <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
        <div
          className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${
            active ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.8)]' : 'bg-slate-700'
          }`}
          style={{ height: `${heightPct}%` }}
        />
      </div>
      <span className={`text-[8px] font-mono ${active ? 'text-green-400 font-bold' : 'text-slate-600'}`}>
        {label}
      </span>
    </div>
  );
}

// ── main component ────────────────────────────────────────────────────────────

export function ControllerActuatorPanel() {
  const [data, setData] = useState<TelemetryData | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe(setData);
    return () => { unsub(); };
  }, []);

  if (!data) return null;

  const {
    thrusters,
    rw_torque,
    solve_time = 0,
    pos_error = 0,
    ang_error = 0,
    mode_state = null,
    completion_gate = null,
    solver_health = null,
    pointing_status = null,
    controller_core = 'v6',
  } = data;

  const angErrorDeg = ang_error * (180 / Math.PI);
  const modeLabel = mode_state?.current_mode ?? 'TRACK';
  const modeTime = mode_state?.time_in_mode_s ?? 0;
  const holdElapsed = completion_gate?.hold_elapsed_s ?? 0;
  const holdRequired = completion_gate?.hold_required_s ?? 0;
  const holdProgress = holdRequired > 0 ? Math.min(1, holdElapsed / holdRequired) : 0;

  return (
    <HudPanel title="CONTROLLER / ACTUATORS" live className="min-w-[240px]">
      <div className="space-y-2 font-mono text-xs">

        {/* Solve time */}
        <div className="flex justify-between items-center">
          <div className="flex items-center gap-2">
            <Activity size={14} className="text-yellow-500" />
            <span className="text-slate-400 font-bold text-[10px]">SOLVE TIME</span>
          </div>
          <span className={`tabular-nums ${solve_time < 20 ? 'text-green-400' : solve_time < 40 ? 'text-yellow-400' : 'text-red-400'}`}>
            {(solve_time * 1000).toFixed(1)} ms
          </span>
        </div>

        <div className="h-px bg-slate-700/50" />

        {/* Mode */}
        <div className="flex justify-between items-center">
          <span className="text-slate-400 font-bold text-[10px]">MODE</span>
          <ModeBadge label={modeLabel} />
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">CORE</span>
          <span className="text-indigo-300">{String(controller_core).toUpperCase()}</span>
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">MODE TIME</span>
          <span className="text-slate-200 tabular-nums">{modeTime.toFixed(1)} s</span>
        </div>

        {/* Errors */}
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">POS ERROR</span>
          <span className={`tabular-nums ${pos_error < 0.1 ? 'text-green-400' : 'text-slate-200'}`}>
            {pos_error.toFixed(3)} m
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">ANG ERROR</span>
          <span className={`tabular-nums ${angErrorDeg < 1.0 ? 'text-green-400' : 'text-slate-200'}`}>
            {angErrorDeg.toFixed(1)}°
          </span>
        </div>

        {/* Pointing */}
        {pointing_status && (
          <>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">X AXIS ERR</span>
              <span className={`tabular-nums ${(Number(pointing_status.x_axis_error_deg ?? 0) <= 6.0) ? 'text-green-400' : 'text-amber-300'}`}>
                {Number(pointing_status.x_axis_error_deg ?? 0).toFixed(2)}°
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">Z AXIS ERR</span>
              <span className={`tabular-nums ${(Number(pointing_status.z_axis_error_deg ?? 0) <= 4.0) ? 'text-green-400' : 'text-amber-300'}`}>
                {Number(pointing_status.z_axis_error_deg ?? 0).toFixed(2)}°
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">POINTING</span>
              <span className={pointing_status.pointing_guardrail_breached ? 'text-amber-300' : 'text-emerald-300'}>
                {pointing_status.pointing_guardrail_breached ? 'BREACH' : 'OK'}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">VISIBLE SIDE</span>
              <span className="text-slate-200">{pointing_status.object_visible_side ?? '--'}</span>
            </div>
          </>
        )}

        {/* Completion gate */}
        {completion_gate && (
          <>
            <div className="h-px bg-slate-700/50" />
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">HOLD</span>
              <span className={`tabular-nums ${holdProgress >= 1 ? 'text-green-400' : 'text-slate-200'}`}>
                {holdElapsed.toFixed(1)} / {holdRequired.toFixed(1)} s
              </span>
            </div>
            <div className="w-full h-1 rounded bg-slate-800 overflow-hidden">
              <div
                className="h-full bg-emerald-500 transition-all duration-100"
                style={{ width: `${(holdProgress * 100).toFixed(1)}%` }}
              />
            </div>
            <div className="grid grid-cols-2 gap-1 text-[10px]">
              <GateFlag label="POS" ok={completion_gate.position_ok} />
              <GateFlag label="ANG" ok={completion_gate.angle_ok} />
              <GateFlag label="VEL" ok={completion_gate.velocity_ok} />
              <GateFlag label="OMEGA" ok={completion_gate.angular_velocity_ok} />
            </div>
            {completion_gate.last_breach_reason && (
              <div className="text-[10px] text-amber-300">
                Last breach: {completion_gate.last_breach_reason}
              </div>
            )}
          </>
        )}

        {/* Solver health */}
        {solver_health && (
          <>
            <div className="h-px bg-slate-700/50" />
            <div className="flex justify-between items-center">
              <span className="text-slate-400 font-bold text-[10px]">SOLVER</span>
              <div className="flex items-center gap-1.5">
                <SolverDot status={solver_health.status} />
                <span className="text-slate-200 uppercase text-[10px]">{solver_health.status}</span>
              </div>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">FALLBACKS</span>
              <span className="text-slate-200 tabular-nums">{solver_health.fallback_count}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">HARD BREACHES</span>
              <span className="text-slate-200 tabular-nums">{solver_health.hard_limit_breaches}</span>
            </div>
            {solver_health.fallback_active && (
              <div className="text-[10px] text-amber-300">
                Active fallback: age {Number(solver_health.fallback_age_s ?? 0).toFixed(2)} s, scale{' '}
                {Number(solver_health.fallback_scale ?? 0).toFixed(2)}
              </div>
            )}
            {solver_health.last_fallback_reason && (
              <div className="text-[10px] text-amber-300">
                Last fallback: {solver_health.last_fallback_reason}
              </div>
            )}
          </>
        )}

        {/* Actuators divider */}
        <div className="h-px bg-slate-700/50 mt-1" />
        <div className="flex gap-4 justify-between pt-1">
          <div className="flex flex-col items-center gap-2">
            <span className="text-[9px] font-bold text-slate-500 tracking-wider">THRUSTERS</span>
            <div className="flex gap-1">
              {thrusters.slice(0, 6).map((val, i) => (
                <ThrusterBar key={i} value={val} label={['+X', '-X', '+Y', '-Y', '+Z', '-Z'][i]} />
              ))}
            </div>
          </div>
          <div className="flex flex-col items-center gap-2">
            <span className="text-[9px] font-bold text-slate-500 tracking-wider">REACTION WHEELS</span>
            <div className="flex gap-1">
              {(rw_torque.length > 0 ? rw_torque : [0, 0, 0]).map((val, i) => (
                <ReactionWheelBar key={i} value={val} label={['X', 'Y', 'Z'][i]} />
              ))}
            </div>
          </div>
        </div>
      </div>
    </HudPanel>
  );
}
