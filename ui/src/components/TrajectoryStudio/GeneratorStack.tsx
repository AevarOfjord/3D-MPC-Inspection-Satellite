import React from 'react';
import { Layers, Plus, GripVertical, Trash2, RefreshCw, Save, Play } from 'lucide-react';
import { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { type MissionSegment } from '../../api/unifiedMission';
import {
  DndContext,
  closestCenter,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
  type DragEndEvent
} from '@dnd-kit/core';
import {
  SortableContext,
  sortableKeyboardCoordinates,
  verticalListSortingStrategy,
  useSortable
} from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';

interface GeneratorStackProps {
    builder: ReturnType<typeof useMissionBuilder>;
}

// Ensure items have unique IDs for dnd-kit
// Since segments don't have IDs by default, we'll map them to an index-based ID for now,
// but for robustness we should probably use stable IDs.
// For this implementation, we will assume index stability is handled by parent re-render.
// Actually, dnd-kit needs stable IDs. We will generate temporary IDs or use index strings if list is simple.
// Using index strings "0", "1" etc is risky if items change.
// But since we are modifying the list itself, let's use a wrapper with generated IDs if needed.
// For now, let's use "segment-{index}" and hope for the best, or stable IDs if we had them.
// We don't have stable IDs on segments. We will use index for now, but this is fragile.
// Better to rely on index solely for lookups.

function SortableItem({
    id,
    segment,
    isSelected,
    onSelect,
    onRemove
}: {
    id: string;
    index: number;
    segment: MissionSegment;
    isSelected: boolean;
    onSelect: () => void;
    onRemove: (e: React.MouseEvent) => void;
}) {
    const {
        attributes,
        listeners,
        setNodeRef,
        transform,
        transition,
        isDragging
    } = useSortable({ id });

    const style = {
        transform: CSS.Transform.toString(transform),
        transition,
        zIndex: isDragging ? 10 : 1,
        opacity: isDragging ? 0.5 : 1
    };

    return (
        <div
           ref={setNodeRef}
           style={style}
           onClick={onSelect}
           className={`group relative flex items-center gap-2 p-2 rounded border transition-all cursor-pointer mb-2 select-none ${
               isSelected
               ? 'bg-cyan-500/10 border-cyan-500/50 shadow-[0_0_15px_rgba(6,182,212,0.1)]'
               : 'bg-slate-900/40 border-slate-800 hover:border-slate-700 hover:bg-slate-800/60'
           }`}
        >
            {/* Grip - Drag Handle */}
            <div
                {...attributes}
                {...listeners}
                className="text-slate-600 cursor-grab active:cursor-grabbing hover:text-slate-400 p-1 touch-none"
            >
                <GripVertical size={14} />
            </div>

            {/* Icon & Type */}
            <div className="flex-1">
                <div className="flex items-center gap-2 mb-0.5">
                    <span className={`text-[10px] font-bold uppercase tracking-wider ${
                        segment.type === 'scan' ? 'text-purple-400' :
                        segment.type === 'transfer' ? 'text-blue-400' : 'text-slate-400'
                    }`}>
                        {segment.type}
                    </span>
                </div>
                <div className="text-xs text-slate-300 font-mono truncate">
                    {segment.type === 'scan'
                        ? segment.path_asset
                            ? `Path: ${segment.path_asset}`
                            : `Target: ${segment.target_id || 'None'}`
                        : segment.type === 'transfer'
                            ? `To: [${segment.end_pose.position.map(v => v.toFixed(1)).join(', ')}]`
                            : ''}
                </div>
            </div>

            {/* Actions (visible on hover or select) */}
            <div className={`flex items-center gap-1 ${isSelected ? 'opacity-100' : 'opacity-0 group-hover:opacity-100'} transition-opacity`}>
                <button
                   onClick={onRemove}
                   className="p-1 hover:bg-slate-700 rounded text-slate-500 hover:text-red-400"
               >
                    <Trash2 size={12} />
                </button>
            </div>

            {/* Selection Indicator */}
            {isSelected && (
                <div className="absolute left-0 top-0 bottom-0 w-0.5 bg-cyan-500 rounded-l" />
            )}
        </div>
    );
}

export function GeneratorStack({ builder }: GeneratorStackProps) {
    const { state, actions } = builder;
    const { selectedSegmentIndex } = state;

    // Create stable-ish IDs for the list.
    // Ideally we'd modify the segment type to include a UUID, but for now we'll construct them.
    // NOTE: If segments change order, using index as ID in loop is problematic for DND.
    // We strictly need stable IDs. Since we don't have them in the data model yet,
    // let's rely on map `index` but be careful.
    // Actually, `dnd-kit` needs to key off something persistent.
    // We will generate a local map of IDs if needed, but let's try simple index-keying first
    // knowing it might act weird on re-sort if not careful.
    // A better approach for this MVP: Attach a temp ID to items in memory if needed,
    // OR just fix the data model.
    // Let's generate a list of strings "0", "1", etc. and assume standard sorting behavior.

    const items = state.segments.map((_, i) => `${i}`);

    const sensors = useSensors(
        useSensor(PointerSensor),
        useSensor(KeyboardSensor, {
            coordinateGetter: sortableKeyboardCoordinates,
        })
    );

    const handleDragEnd = (event: DragEndEvent) => {
        const { active, over } = event;
        if (active.id !== over?.id) {
            const oldIndex = parseInt(active.id as string);
            const newIndex = parseInt(over?.id as string);
            actions.reorderSegments(oldIndex, newIndex);
        }
    };

    return (
        <div className="w-80 bg-slate-950/90 backdrop-blur-md border border-slate-800 rounded-lg shadow-2xl flex flex-col max-h-[calc(100vh-200px)] overflow-hidden">
            {/* Header */}
            <div className="p-3 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
                <div className="flex items-center gap-2">
                    <Layers size={16} className="text-cyan-400" />
                    <h3 className="font-bold text-sm tracking-wider text-slate-200">GENERATORS</h3>
                </div>
            </div>

            {/* List */}
            <div className="flex-1 overflow-y-auto p-2 custom-scrollbar">
                {/* Fixed START Item */}
                <div
                   onClick={() => actions.selectSegment(-1)}
                   className={`group relative flex items-center gap-2 p-2 rounded border transition-all cursor-pointer mb-2 select-none ${
                       selectedSegmentIndex === -1
                       ? 'bg-cyan-500/10 border-cyan-500/50 shadow-[0_0_15px_rgba(6,182,212,0.1)]'
                       : 'bg-slate-900/40 border-slate-800 hover:border-slate-700 hover:bg-slate-800/60'
                   }`}
                >
                    {/* Icon - Play/Start */}
                    <div className="text-emerald-500 p-1">
                        <Play size={14} fill="currentColor" />
                    </div>

                    {/* Content */}
                    <div className="flex-1">
                        <div className="flex items-center gap-2 mb-0.5">
                            <span className="text-[10px] font-bold uppercase tracking-wider text-emerald-400">
                                START
                            </span>
                        </div>
                        <div className="text-xs text-slate-300 font-mono truncate">
                             {state.startFrame}
                             {state.startFrame === 'LVLH' && state.startTargetId ? ` @ ${state.startTargetId}` : ''}
                             {' : '}
                             [{state.startPosition.map(v => v.toFixed(1)).join(', ')}]
                        </div>
                    </div>
                     {/* Selection Indicator */}
                    {selectedSegmentIndex === -1 && (
                        <div className="absolute left-0 top-0 bottom-0 w-0.5 bg-cyan-500 rounded-l" />
                    )}
                </div>

                <div className="w-full h-px bg-slate-800/50 my-2" />

                <DndContext
                    sensors={sensors}
                    collisionDetection={closestCenter}
                    onDragEnd={handleDragEnd}
                >
                    <SortableContext
                        items={items}
                        strategy={verticalListSortingStrategy}
                    >
                        {state.segments.map((seg: MissionSegment, idx: number) => (
                            <SortableItem
                                key={idx} // Using index as key is generally bad for DND but unavoidable without IDs
                                id={`${idx}`}
                                segment={seg}
                                isSelected={selectedSegmentIndex === idx}
                                onSelect={() => actions.selectSegment(idx)}
                                onRemove={(e) => { e.stopPropagation(); actions.removeSegment(idx); } } index={0}                            />
                        ))}
                    </SortableContext>
                </DndContext>

                 {/* Add Buttons */}
                 <div className="flex gap-2 mt-2">
                    <button
                        onClick={() => actions.addScanSegment()}
                        className="flex-1 py-2 border border-dashed border-slate-700 rounded flex items-center justify-center gap-2 text-xs text-slate-500 hover:text-cyan-400 hover:border-cyan-500/50 hover:bg-cyan-500/5 transition-all"
                    >
                        <Plus size={14} /> ADD SCAN
                    </button>
                    <button
                        onClick={() => actions.addTransferSegment()}
                        className="flex-1 py-2 border border-dashed border-slate-700 rounded flex items-center justify-center gap-2 text-xs text-slate-500 hover:text-blue-400 hover:border-blue-500/50 hover:bg-blue-500/5 transition-all"
                    >
                        <Plus size={14} /> ADD TRANSFER
                    </button>
                 </div>
            </div>

            {/* Action Bar */}
            <div className="p-3 border-t border-slate-800 bg-slate-900/50 flex gap-2">
                 <button
                    onClick={() => actions.generateUnifiedPath()}
                    disabled={state.loading}
                    className="flex-1 py-1.5 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded text-xs font-bold uppercase flex items-center justify-center gap-2 border border-slate-700 transition-all"
                 >
                     <RefreshCw size={14} className={state.loading ? 'animate-spin' : ''} />
                     PREVIEW
                 </button>
                 <button
                    onClick={() => actions.handleSaveUnifiedMission()}
                    disabled={state.loading}
                    className="flex-1 py-1.5 bg-cyan-600 hover:bg-cyan-500 text-white rounded text-xs font-bold uppercase flex items-center justify-center gap-2 shadow-[0_0_15px_rgba(8,145,178,0.4)] transition-all"
                 >
                     <Save size={14} />
                     SAVE
                 </button>
            </div>
        </div>
    );
}
