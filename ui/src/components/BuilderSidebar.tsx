import { useEffect, useMemo, useState } from 'react';
import { Trash2, Plus, Save, Undo, Redo, RefreshCcw, Gauge } from 'lucide-react';
import { HudButton, HudInput, HudSection } from './HudComponents';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { ConstraintReadout } from './ConstraintVisualizer';
import type { MissionSegment, ScanSegment, TransferSegment } from '../api/unifiedMission';

type BuilderHook = ReturnType<typeof useMissionBuilder>;

interface BuilderSidebarProps {
    builder: BuilderHook;
    onExit?: () => void;
}

const makeTransferSegment = (position: [number, number, number]): TransferSegment => ({
    type: 'transfer',
    end_pose: { frame: 'ECI', position },
    constraints: { speed_max: 0.25, accel_max: 0.05, angular_rate_max: 0.1 },
});

export function BuilderSidebar({ builder }: BuilderSidebarProps) {
    const { state, setters, actions } = builder;
    const [unifiedMissionName, setUnifiedMissionName] = useState('');
    const [selectedUnifiedMission, setSelectedUnifiedMission] = useState('');
    const [busyAction, setBusyAction] = useState<string | null>(null);

    useEffect(() => {
        actions.refreshUnifiedMissions().catch((err: unknown) => {
            console.error(err);
        });
        // Intentionally run once on mount.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const scanIndex = state.segments.findIndex((seg) => seg.type === 'scan');
    const scanSegment = scanIndex >= 0 ? (state.segments[scanIndex] as ScanSegment) : null;
    const transferIndex = state.segments.findIndex((seg) => seg.type === 'transfer');
    const transferSegment = transferIndex >= 0 ? (state.segments[transferIndex] as TransferSegment) : null;

    const targetPosition = scanSegment?.target_pose?.position ?? null;
    const selectedTargetLabel =
        state.selectedOrbitTargetId || scanSegment?.target_id || 'None';

    const addVec = (a: [number, number, number], b: [number, number, number]) => [
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2],
    ] as [number, number, number];

    const subVec = (a: [number, number, number], b: [number, number, number]) => [
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    ] as [number, number, number];

    const startRelative = targetPosition ? subVec(state.startPosition, targetPosition) : state.startPosition;
    const endRelative = transferSegment
        ? (targetPosition ? subVec(transferSegment.end_pose.position as [number, number, number], targetPosition) : transferSegment.end_pose.position as [number, number, number])
        : ([0, 0, 0] as [number, number, number]);

    const setStartRelative = (next: [number, number, number]) => {
        const absolute = targetPosition ? addVec(targetPosition, next) : next;
        setters.setStartPosition(absolute);
    };

    const setEndRelative = (next: [number, number, number]) => {
        if (!targetPosition && !transferSegment) return;
        const absolute = targetPosition ? addVec(targetPosition, next) : next;
        const nextSegments = [...state.segments];
        if (transferIndex >= 0 && transferSegment) {
            nextSegments[transferIndex] = {
                ...transferSegment,
                end_pose: { ...transferSegment.end_pose, position: absolute },
            };
        } else {
            nextSegments.push(makeTransferSegment(absolute));
        }
        setters.setSegments(nextSegments as MissionSegment[]);
    };

    const toggleEndEnabled = () => {
        if (transferIndex >= 0) {
            const nextSegments = state.segments.filter((_, idx) => idx !== transferIndex);
            setters.setSegments(nextSegments);
            return;
        }
        const absolute = targetPosition ? addVec(targetPosition, [0, 0, 0]) : ([0, 0, 0] as [number, number, number]);
        setters.setSegments([...state.segments, makeTransferSegment(absolute)]);
    };

    const updateScanSegment = (patch: Partial<ScanSegment>) => {
        if (!scanSegment || scanIndex < 0) return;
        const nextSegments = [...state.segments];
        nextSegments[scanIndex] = { ...scanSegment, ...patch };
        setters.setSegments(nextSegments as MissionSegment[]);
    };

    const updateScanConfig = (patch: Partial<ScanSegment['scan']>) => {
        if (!scanSegment || scanIndex < 0) return;
        updateScanSegment({
            ...scanSegment,
            scan: {
                ...scanSegment.scan,
                ...patch,
            },
        });
    };

    const handleGeneratePath = async () => {
        setBusyAction('generate');
        try {
            await actions.generateUnifiedPath();
        } catch (err) {
            console.error(err);
            alert(`Generate failed: ${String(err)}`);
        } finally {
            setBusyAction(null);
        }
    };

    const handleSaveUnified = async () => {
        if (!unifiedMissionName.trim()) return;
        setBusyAction('save');
        try {
            await actions.saveUnifiedMission(unifiedMissionName.trim());
            await actions.refreshUnifiedMissions();
        } catch (err) {
            console.error(err);
            alert(`Save failed: ${String(err)}`);
        } finally {
            setBusyAction(null);
        }
    };

    const handleLoadUnified = async () => {
        if (!selectedUnifiedMission) return;
        setBusyAction('load');
        try {
            await actions.loadUnifiedMission(selectedUnifiedMission);
        } catch (err) {
            console.error(err);
            alert(`Load failed: ${String(err)}`);
        } finally {
            setBusyAction(null);
        }
    };

    const updateObstacleRelative = (idx: number, relative: [number, number, number]) => {
        const absolute = targetPosition ? addVec(targetPosition, relative) : relative;
        actions.updateObstacle(idx, { position: absolute });
    };

    const obstacleRelative = (absolute: [number, number, number]) =>
        targetPosition ? subVec(absolute, targetPosition) : absolute;

    const selectedWaypointPosition = (() => {
        if (!state.selectedObjectId || !state.selectedObjectId.startsWith('waypoint-')) {
            return null;
        }
        const idx = Number(state.selectedObjectId.split('-')[1]);
        if (!Number.isFinite(idx)) return null;
        if (!state.previewPath || idx < 0 || idx >= state.previewPath.length) return null;
        return state.previewPath[idx] as [number, number, number];
    })();

    const scanConfig = scanSegment?.scan;

    const autoPitch = useMemo(() => {
        if (!scanConfig) return null;
        const halfRad = (scanConfig.fov_deg * Math.PI) / 360;
        const footprint = 2 * scanConfig.standoff * Math.tan(halfRad);
        const clampedOverlap = Math.min(Math.max(scanConfig.overlap, 0), 0.9);
        return footprint * (1 - clampedOverlap);
    }, [scanConfig]);

    return (
        <div className="w-96 bg-slate-950 border-r border-slate-800 flex flex-col shadow-2xl z-20 h-full">
            <div className="p-4 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
                <h2 className="font-bold text-lg tracking-wider text-white">PLANNER</h2>
                <div className="flex gap-1">
                    <HudButton size="sm" variant="ghost" disabled={!state.canUndo} onClick={actions.undo}>
                        <Undo size={16} />
                    </HudButton>
                    <HudButton size="sm" variant="ghost" disabled={!state.canRedo} onClick={actions.redo}>
                        <Redo size={16} />
                    </HudButton>
                </div>
            </div>

            <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
                {state.previewPath.length > 0 && (
                    <div className="bg-slate-900/50 p-3 rounded border border-slate-700">
                        <div className="flex items-center gap-2 mb-2 text-cyan-400 font-bold text-xs uppercase">
                            <Gauge size={14} /> ENGINEERING CHECKS
                        </div>
                        <ConstraintReadout points={state.previewPath} />
                        {state.isManualMode && (
                            <div className="mt-2 text-[10px] text-orange-400 font-mono bg-orange-500/10 px-2 py-1 rounded border border-orange-500/30 flex items-center gap-2">
                                <span>⚠️ MANUAL MODE ACTIVE</span>
                                <button
                                    onClick={handleGeneratePath}
                                    title="Regenerate Path"
                                    className="text-white hover:underline"
                                >
                                    <RefreshCcw size={10} />
                                </button>
                            </div>
                        )}
                    </div>
                )}

                <HudSection title="Target" defaultOpen={true}>
                    <div className="space-y-2">
                        <div className="flex items-center justify-between text-[10px] uppercase text-slate-500">
                            <span>Selected Target</span>
                            <span className="text-slate-300">{selectedTargetLabel || 'None'}</span>
                        </div>
                        <div className="text-[11px] text-slate-500">
                            Select a target from the Orbit Targets panel.
                        </div>
                    </div>
                </HudSection>

                <HudSection title="Start / End (Target Frame)" defaultOpen={true}>
                    <div className={`space-y-3 ${targetPosition ? '' : 'opacity-60 pointer-events-none'}`}>
                        <Vec3Input
                            label="Start Offset"
                            value={startRelative}
                            onChange={setStartRelative}
                        />
                        <div className="flex items-center justify-between">
                            <span className="text-[10px] uppercase text-slate-500">End Offset</span>
                            <HudButton
                                variant="ghost"
                                size="sm"
                                onClick={toggleEndEnabled}
                            >
                                {transferSegment ? 'Remove End' : 'Add End'}
                            </HudButton>
                        </div>
                        {transferSegment && (
                            <Vec3Input
                                label="End Offset"
                                value={endRelative}
                                onChange={setEndRelative}
                            />
                        )}
                        {!targetPosition && (
                            <div className="text-[11px] text-slate-500">Select a target to edit offsets.</div>
                        )}
                    </div>
                </HudSection>

                <HudSection title="Scan Settings" defaultOpen={true}>
                    {!scanConfig ? (
                        <div className="text-[11px] text-slate-500">Select a target to configure scan.</div>
                    ) : (
                        <div className="space-y-3">
                            <div className="grid grid-cols-2 gap-2">
                                <HudInput
                                    label="Standoff (m)"
                                    value={scanConfig.standoff}
                                    type="number"
                                    step={0.5}
                                    onChange={(val) => updateScanConfig({ standoff: Number.isFinite(val) ? val : scanConfig.standoff })}
                                />
                                <HudInput
                                    label="Overlap"
                                    value={scanConfig.overlap}
                                    type="number"
                                    step={0.05}
                                    min={0}
                                    max={0.9}
                                    onChange={(val) => updateScanConfig({ overlap: Number.isFinite(val) ? val : scanConfig.overlap })}
                                />
                                <HudInput
                                    label="FOV (deg)"
                                    value={scanConfig.fov_deg}
                                    type="number"
                                    step={1}
                                    onChange={(val) => updateScanConfig({ fov_deg: Number.isFinite(val) ? val : scanConfig.fov_deg })}
                                />
                                <HudInput
                                    label="Revolutions"
                                    value={scanConfig.revolutions}
                                    type="number"
                                    step={1}
                                    onChange={(val) => updateScanConfig({ revolutions: Number.isFinite(val) ? val : scanConfig.revolutions })}
                                />
                            </div>

                            <div className="grid grid-cols-2 gap-2">
                                <div>
                                    <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                        Spiral Axis
                                    </label>
                                    <select
                                        value={scanConfig.axis}
                                        onChange={(e) => updateScanConfig({ axis: e.target.value as ScanSegment['scan']['axis'] })}
                                        className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                                    >
                                        {['+X', '-X', '+Y', '-Y', '+Z', '-Z'].map((axis) => (
                                            <option key={axis} value={axis}>
                                                {axis}
                                            </option>
                                        ))}
                                    </select>
                                </div>
                                <div>
                                    <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                        Sensor Axis
                                    </label>
                                    <select
                                        value={scanConfig.sensor_axis}
                                        onChange={(e) => updateScanConfig({ sensor_axis: e.target.value as ScanSegment['scan']['sensor_axis'] })}
                                        className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                                    >
                                        {['+Y', '-Y'].map((axis) => (
                                            <option key={axis} value={axis}>
                                                {axis}
                                            </option>
                                        ))}
                                    </select>
                                </div>
                                <div>
                                    <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                        Direction
                                    </label>
                                    <select
                                        value={scanConfig.direction}
                                        onChange={(e) => updateScanConfig({ direction: e.target.value as ScanSegment['scan']['direction'] })}
                                        className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                                    >
                                        {['CW', 'CCW'].map((dir) => (
                                            <option key={dir} value={dir}>
                                                {dir}
                                            </option>
                                        ))}
                                    </select>
                                </div>
                                <div>
                                    <HudInput
                                        label="Pitch (m)"
                                        value={scanConfig.pitch ?? ''}
                                        type="number"
                                        step={0.1}
                                        placeholder={autoPitch ? `auto ${autoPitch.toFixed(2)}` : 'auto'}
                                        allowEmpty
                                        onChange={(val) => updateScanConfig({ pitch: Number.isFinite(val) ? val : null })}
                                    />
                                </div>
                            </div>
                        </div>
                    )}
                </HudSection>

                <HudSection title="Obstacles (Target Frame)" defaultOpen={false}>
                    <div className={`space-y-3 ${targetPosition ? '' : 'opacity-60 pointer-events-none'}`}>
                        {state.obstacles.length === 0 && (
                            <div className="text-[10px] text-slate-500 italic">No obstacles yet.</div>
                        )}
                        {state.obstacles.map((obs, i) => (
                            <div key={i} className="bg-slate-900/50 p-2 rounded border border-slate-800">
                                <div className="flex justify-between items-center mb-2">
                                    <span className="text-xs font-bold text-slate-400">OBSTACLE {i + 1}</span>
                                    <button
                                        onClick={() => actions.removeObstacle(i)}
                                        className="text-red-400 hover:text-red-300"
                                    >
                                        <Trash2 size={14} />
                                    </button>
                                </div>
                                <Vec3Input
                                    label="Offset"
                                    value={obstacleRelative(obs.position)}
                                    onChange={(v) => updateObstacleRelative(i, v)}
                                />
                                <div className="mt-2">
                                    <HudInput
                                        label="Radius (m)"
                                        value={obs.radius}
                                        type="number"
                                        step={0.1}
                                        onChange={(v) => actions.updateObstacle(i, { radius: v })}
                                    />
                                </div>
                            </div>
                        ))}
                        <HudButton
                            variant="secondary"
                            size="sm"
                            onClick={() => actions.addObstacle(targetPosition ?? undefined, [2, 0, 0])}
                            className="w-full"
                        >
                            <Plus size={14} /> ADD OBSTACLE
                        </HudButton>
                        {!targetPosition && (
                            <div className="text-[11px] text-slate-500">Select a target to edit obstacles.</div>
                        )}
                    </div>
                </HudSection>

                <HudSection title="Path" defaultOpen={true}>
                    <div className="space-y-3">
                        <HudButton
                            variant="primary"
                            size="md"
                            className="w-full"
                            onClick={handleGeneratePath}
                            disabled={busyAction === 'generate' || !targetPosition}
                        >
                            {busyAction === 'generate' ? 'GENERATING…' : 'GENERATE PATH'}
                        </HudButton>
                        {!targetPosition && (
                            <div className="text-[11px] text-slate-500">Select a target to generate the path.</div>
                        )}

                        <div className="border border-slate-800 rounded-sm p-3 space-y-2 bg-slate-950/60">
                            <div className="text-[10px] uppercase font-bold text-slate-500">Spline Controls</div>
                            <div className="flex gap-2">
                                <HudButton variant="secondary" size="sm" onClick={() => actions.addSplineControl()}>
                                    + CONTROL
                                </HudButton>
                                <HudButton
                                    variant="ghost"
                                    size="sm"
                                    disabled={!selectedWaypointPosition}
                                    onClick={() => {
                                        if (selectedWaypointPosition) {
                                            actions.addSplineControl(selectedWaypointPosition);
                                        }
                                    }}
                                >
                                    FROM WAYPOINT
                                </HudButton>
                            </div>

                            {state.splineControls.length === 0 && (
                                <div className="text-[10px] text-slate-500 italic">No spline controls yet.</div>
                            )}

                            <div className="space-y-2">
                                {state.splineControls.map((control, idx) => (
                                    <div key={`spline-${idx}`} className="border border-slate-800 rounded-sm p-2 bg-slate-900/40">
                                        <div className="flex items-center justify-between mb-2">
                                            <span className="text-[10px] uppercase text-slate-500">Control {idx + 1}</span>
                                            <button
                                                onClick={() => actions.removeSplineControl(idx)}
                                                className="text-[10px] uppercase text-red-400 hover:text-red-300"
                                            >
                                                Remove
                                            </button>
                                        </div>
                                        <Vec3Input
                                            label="Position"
                                            value={control.position as [number, number, number]}
                                            onChange={(next) => {
                                                actions.updateSplineControl(idx, {
                                                    ...control,
                                                    position: next,
                                                });
                                            }}
                                        />
                                        <div className="mt-2">
                                            <HudInput
                                                label="Weight"
                                                value={control.weight ?? 1.0}
                                                type="number"
                                                step={0.1}
                                                onChange={(val) => {
                                                    const weight = Number.isFinite(val) ? Math.max(0, val) : 1.0;
                                                    actions.updateSplineControl(idx, { ...control, weight });
                                                }}
                                            />
                                        </div>
                                    </div>
                                ))}
                            </div>
                        </div>
                    </div>
                </HudSection>

                <HudSection title="Save / Load" defaultOpen={false}>
                    <div className="space-y-3">
                        <HudInput
                            label="Mission Name"
                            value={unifiedMissionName}
                            onChange={(val) => setUnifiedMissionName(String(val))}
                        />
                        <HudButton
                            variant="primary"
                            size="sm"
                            className="w-full"
                            onClick={handleSaveUnified}
                            disabled={busyAction === 'save' || !unifiedMissionName.trim()}
                        >
                            <Save size={14} /> {busyAction === 'save' ? 'SAVING…' : 'SAVE'}
                        </HudButton>
                        <div className="border border-slate-800 rounded-sm p-2 bg-slate-950/60 space-y-2">
                            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400">
                                Load Mission
                            </label>
                            <select
                                value={selectedUnifiedMission}
                                onChange={(e) => setSelectedUnifiedMission(e.target.value)}
                                className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                            >
                                <option value="">Select…</option>
                                {state.savedUnifiedMissions.map((mission) => (
                                    <option key={mission} value={mission}>
                                        {mission}
                                    </option>
                                ))}
                            </select>
                            <div className="flex gap-2">
                                <HudButton
                                    variant="secondary"
                                    size="sm"
                                    className="flex-1"
                                    onClick={handleLoadUnified}
                                    disabled={busyAction === 'load' || !selectedUnifiedMission}
                                >
                                    {busyAction === 'load' ? 'LOADING…' : 'LOAD'}
                                </HudButton>
                                <HudButton
                                    variant="ghost"
                                    size="sm"
                                    className="flex-1"
                                    onClick={() => actions.refreshUnifiedMissions()}
                                >
                                    REFRESH
                                </HudButton>
                            </div>
                        </div>
                    </div>
                </HudSection>
            </div>
        </div>
    );
}

// --- Helpers ---
function Vec3Input({ label, value, onChange, unit }: { label: string; value: [number, number, number]; onChange: (v: [number, number, number]) => void; unit?: string }) {
    return (
        <div className="space-y-1">
            <div className="flex justify-between">
                <span className="text-[10px] font-bold text-slate-500 uppercase">{label}</span>
                {unit && <span className="text-[10px] text-slate-600">{unit}</span>}
            </div>
            <div className="grid grid-cols-3 gap-1">
                <HudInput value={value[0]} type="number" step={0.1} onChange={(v) => onChange([v, value[1], value[2]])} />
                <HudInput value={value[1]} type="number" step={0.1} onChange={(v) => onChange([value[0], v, value[2]])} />
                <HudInput value={value[2]} type="number" step={0.1} onChange={(v) => onChange([value[0], value[1], v])} />
            </div>
        </div>
    );
}
