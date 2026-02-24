import { Suspense } from 'react';
import { MissionStudioLeftPanel } from './MissionStudioLeftPanel';
import { MissionStudioRightPanel } from './MissionStudioRightPanel';

function CanvasPlaceholder() {
  return (
    <div className="flex-1 flex items-center justify-center text-slate-500 text-sm select-none"
         style={{ background: '#070b14' }}>
      3D Canvas — coming in Task 3
    </div>
  );
}

export function MissionStudioLayout() {
  return (
    <div className="flex-1 flex min-h-0 overflow-hidden" style={{ background: '#070b14' }}>
      {/* Left panel */}
      <div className="w-[280px] shrink-0 flex flex-col border-r border-slate-800/60 overflow-y-auto"
           style={{ background: 'rgba(13,21,36,0.97)' }}>
        <MissionStudioLeftPanel />
      </div>

      {/* 3D Canvas */}
      <div className="flex-1 relative min-w-0">
        <Suspense fallback={null}>
          <CanvasPlaceholder />
        </Suspense>
      </div>

      {/* Right panel */}
      <div className="w-[260px] shrink-0 flex flex-col border-l border-slate-800/60 overflow-y-auto"
           style={{ background: 'rgba(13,21,36,0.97)' }}>
        <MissionStudioRightPanel />
      </div>
    </div>
  );
}
