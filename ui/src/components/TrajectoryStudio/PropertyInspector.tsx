import React from 'react';
import { Settings, X } from 'lucide-react';
import { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { HudInput } from '../HudComponents';
import type { ScanSegment, TransferSegment } from '../../api/unifiedMission';
import { orbitSnapshot } from '../../data/orbitSnapshot';

interface PropertyInspectorProps {
    builder: ReturnType<typeof useMissionBuilder>;
}

export function PropertyInspector({ builder }: PropertyInspectorProps) {
    const { state, actions } = builder;
    const { selectedSegmentIndex } = state;
    const issues = state.validationReport?.issues ?? [];
    const hasIssueForPath = (pathFragment: string) =>
        issues.some((issue) => issue.path.includes(pathFragment));
    const issueMessageForPath = (pathFragment: string) =>
        issues.find((issue) => issue.path.includes(pathFragment))?.message ?? null;
    const startHasIssue =
        hasIssueForPath('start_pose') ||
        hasIssueForPath('start_target_id') ||
        hasIssueForPath('epoch');
    
    if (selectedSegmentIndex === null) return null;
    
    // --- Start Configuration Mode ---
    if (selectedSegmentIndex === -1) {
        return (
            <div className={`w-72 bg-slate-950/90 backdrop-blur-md border rounded-lg shadow-2xl flex flex-col max-h-[80vh] overflow-y-auto custom-scrollbar ${
                startHasIssue ? 'border-amber-500/70' : 'border-slate-800'
            }`}>
                <div className="p-3 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
                    <div className="flex items-center gap-2">
                        <Settings size={16} className="text-emerald-400" />
                        <h3 className="font-bold text-sm tracking-wider text-slate-200">START POINT</h3>
                    </div>
                    <button onClick={() => actions.selectSegment(null)} className="text-slate-500 hover:text-white">
                        <X size={14} />
                    </button>
                </div>
                <div className="p-3 space-y-4">
                    {startHasIssue && (
                        <div className="text-[10px] text-amber-200 bg-amber-950/50 border border-amber-700/60 rounded px-2 py-1.5">
                            Start configuration has validation issues. Review frame/target/position values.
                        </div>
                    )}
                     {/* Frame Selection */}
                     <div>
                        <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                            Reference Frame
                        </label>
                        <div className="text-xs py-1.5 rounded border border-slate-700 bg-slate-900/50 text-slate-300 uppercase text-center">
                            LVLH
                        </div>
                    </div>

                    <div>
                        <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                            Relative To
                        </label>
                        <select
                            value={state.startTargetId || ''}
                            onChange={(e) => builder.setters.setStartTargetId(e.target.value || undefined)}
                            className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded px-2 py-1.5 outline-none focus:border-emerald-500"
                        >
                            <option value="">Select Object...</option>
                            {orbitSnapshot.objects.map(obj => (
                                <option key={obj.id} value={obj.id}>{obj.name}</option>
                            ))}
                        </select>
                    </div>

                    <div>
                        <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                            Start Position (m)
                        </label>
                        <div className="grid grid-cols-3 gap-2">
                            <HudInput
                                label="X"
                                value={state.startPosition[0]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...state.startPosition] as [number, number, number];
                                    pos[0] = Number.isFinite(val) ? val : pos[0];
                                    builder.setters.setStartPosition(pos);
                                }}
                            />
                            <HudInput
                                label="Y"
                                value={state.startPosition[1]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...state.startPosition] as [number, number, number];
                                    pos[1] = Number.isFinite(val) ? val : pos[1];
                                    builder.setters.setStartPosition(pos);
                                }}
                            />
                            <HudInput
                                label="Z"
                                value={state.startPosition[2]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...state.startPosition] as [number, number, number];
                                    pos[2] = Number.isFinite(val) ? val : pos[2];
                                    builder.setters.setStartPosition(pos);
                                }}
                            />
                        </div>
                    </div>
                </div>
            </div>
        );
    }
    
    const segment = state.segments[selectedSegmentIndex];
    if (!segment) return null;
    const segmentPathPrefix = `segments[${selectedSegmentIndex}]`;
    const segmentHasIssues = hasIssueForPath(segmentPathPrefix);
    const targetIssue = issueMessageForPath(`${segmentPathPrefix}.target_id`);
    const pathAssetIssue = issueMessageForPath(`${segmentPathPrefix}.path_asset`);

    const updateTransferConfig = (patch: any) => {
        const s = segment as TransferSegment;
        actions.updateSegment(selectedSegmentIndex, {
            ...s,
            end_pose: { ...s.end_pose, ...patch }
        });
    };

    return (
        <div className={`w-72 bg-slate-950/90 backdrop-blur-md border rounded-lg shadow-2xl flex flex-col max-h-[80vh] overflow-y-auto custom-scrollbar ${
            segmentHasIssues ? 'border-amber-500/70' : 'border-slate-800'
        }`}>
            {/* Header */}
            <div className="p-3 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
                <div className="flex items-center gap-2">
                    <Settings size={16} className="text-orange-400" />
                    <h3 className="font-bold text-sm tracking-wider text-slate-200">INSPECTOR</h3>
                </div>
                <button onClick={() => actions.selectSegment(null)} className="text-slate-500 hover:text-white">
                    <X size={14} />
                </button>
            </div>

            <div className="p-3 space-y-4">
                {segmentHasIssues && (
                    <div className="text-[10px] text-amber-200 bg-amber-950/50 border border-amber-700/60 rounded px-2 py-1.5">
                        This segment has validation issues. Open the Validate step for full details.
                    </div>
                )}
                {/* Common Props */}
                <div>
                   <label className="text-[10px] font-bold uppercase tracking-wider text-slate-500 block mb-1">Type</label>
                   <div className="text-sm font-mono text-slate-200">{segment.type}</div>
                </div>

                {/* Transfer Specific */}
                {segment.type === 'transfer' && (
                    <div className="space-y-3">
                        {/* Frame Selection */}
                        <div>
                            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                Reference Frame
                            </label>
                            <div className="text-xs py-1.5 rounded border border-slate-700 bg-slate-900/50 text-slate-300 uppercase text-center">
                                LVLH
                            </div>
                        </div>

                         <div>
                            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                Relative To
                            </label>
                            <select
                                value={(segment as TransferSegment).target_id || ''}
                                onChange={(e) => {
                                    const s = segment as TransferSegment;
                                    actions.updateSegment(selectedSegmentIndex, {
                                        ...s,
                                        target_id: e.target.value || undefined,
                                        end_pose: { ...s.end_pose, frame: 'LVLH' },
                                    });
                                }}
                                className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded px-2 py-1.5 outline-none focus:border-cyan-500"
                            >
                                <option value="">Start / Previous State</option>
                                {orbitSnapshot.objects.map(obj => (
                                    <option key={obj.id} value={obj.id}>{obj.name}</option>
                                ))}
                            </select>
                        </div>

                        <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                            Target Position (m)
                        </label>
                        <div className="grid grid-cols-3 gap-2">
                             <HudInput
                                label="X"
                                value={(segment as TransferSegment).end_pose.position[0]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...(segment as TransferSegment).end_pose.position] as [number, number, number];
                                    pos[0] = Number.isFinite(val) ? val : pos[0];
                                    updateTransferConfig({ position: pos });
                                }}
                            />
                            <HudInput
                                label="Y"
                                value={(segment as TransferSegment).end_pose.position[1]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...(segment as TransferSegment).end_pose.position] as [number, number, number];
                                    pos[1] = Number.isFinite(val) ? val : pos[1];
                                    updateTransferConfig({ position: pos });
                                }}
                            />
                            <HudInput
                                label="Z"
                                value={(segment as TransferSegment).end_pose.position[2]}
                                type="number"
                                onChange={(val) => {
                                    const pos = [...(segment as TransferSegment).end_pose.position] as [number, number, number];
                                    pos[2] = Number.isFinite(val) ? val : pos[2];
                                    updateTransferConfig({ position: pos });
                                }}
                            />
                        </div>
                    </div>
                )}

                {/* Scan Specific */}
                {segment.type === 'scan' && (
                    <div className="space-y-3">
                        <div>
                            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                Target Object
                            </label>
                            <select
                                value={(segment as ScanSegment).target_id || ''}
                                onChange={(e) => {
                                    const targetId = e.target.value || '';
                                    if (!targetId) {
                                        actions.updateSegment(selectedSegmentIndex, {
                                            ...(segment as ScanSegment),
                                            target_id: '',
                                            target_pose: undefined,
                                        });
                                        actions.setSelectedOrbitTargetId(null);
                                        return;
                                    }
                                    const target = orbitSnapshot.objects.find(obj => obj.id === targetId);
                                    const targetPos = target ? (target.position_m as [number, number, number]) : undefined;
                                    actions.assignScanTarget(targetId, targetPos);
                                }}
                                className={`w-full bg-slate-900/50 border text-slate-200 text-xs rounded px-2 py-1.5 outline-none focus:border-cyan-500 ${
                                    targetIssue ? 'border-amber-500/70' : 'border-slate-700'
                                }`}
                            >
                                <option value="">Select Object...</option>
                                {orbitSnapshot.objects.map(obj => (
                                    <option key={obj.id} value={obj.id}>{obj.name}</option>
                                ))}
                            </select>
                            {!(segment as ScanSegment).target_id && (
                                <div className="text-[10px] text-amber-400 mt-1">
                                    Select the object the scan path is relative to.
                                </div>
                            )}
                            {targetIssue && (
                                <div className="text-[10px] text-amber-300 mt-1">{targetIssue}</div>
                            )}
                        </div>

                        <div>
                            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                                Path Asset
                            </label>
                            <select
                                value={(segment as ScanSegment).path_asset || ''}
                                onChange={(e) => {
                                    const s = segment as ScanSegment;
                                    actions.updateSegment(selectedSegmentIndex, {
                                        ...s,
                                        path_asset: e.target.value || undefined,
                                    });
                                }}
                                className={`w-full bg-slate-900/50 border text-slate-200 text-xs rounded px-2 py-1.5 outline-none focus:border-cyan-500 ${
                                    pathAssetIssue ? 'border-amber-500/70' : 'border-slate-700'
                                }`}
                            >
                                <option value="">Select Path Asset...</option>
                                {state.pathAssets.map((asset) => (
                                    <option key={asset.id} value={asset.id}>
                                        {asset.name}
                                    </option>
                                ))}
                            </select>
                            {!(segment as ScanSegment).path_asset && (
                                <div className="text-[10px] text-amber-400 mt-1">
                                    Path assets are created in the Scan Definition step.
                                </div>
                            )}
                            {pathAssetIssue && (
                                <div className="text-[10px] text-amber-300 mt-1">{pathAssetIssue}</div>
                            )}
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}
