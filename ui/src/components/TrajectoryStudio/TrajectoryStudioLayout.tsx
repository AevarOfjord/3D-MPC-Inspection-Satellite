import React from 'react';
import { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { GeneratorStack } from './GeneratorStack';
import { StateTimeline } from './StateTimeline';
import { PropertyInspector } from './PropertyInspector';
import { PathStudioPanel } from './PathStudioPanel';

interface TrajectoryStudioLayoutProps {
    builder: ReturnType<typeof useMissionBuilder>;
    viewport: React.ReactNode;
    showPathStudio?: boolean;
    showGeneratorStack?: boolean;
    showTimeline?: boolean;
    showInspector?: boolean;
}

export function TrajectoryStudioLayout({
    builder,
    viewport,
    showPathStudio = false,
    showGeneratorStack = true,
    showTimeline = true,
    showInspector = true,
}: TrajectoryStudioLayoutProps) {
    return (
        <div className="flex-1 relative flex flex-col h-full overflow-hidden">
            {/* Main Viewport Area */}
            <div className="flex-1 relative min-h-0">
                 {/* The Viewport (Leaflet/Three map) */}
                 {viewport}

                 {/* Floating Panels */}
                 {/* Path Studio - Top Left */}
                 {showPathStudio && (
                     <div className="absolute top-4 left-4 z-20">
                         <PathStudioPanel builder={builder} />
                     </div>
                 )}

                 {/* Generator Stack - Top Right */}
                 {showGeneratorStack && (
                     <div className="absolute top-4 right-4 z-20">
                         <GeneratorStack builder={builder} />
                     </div>
                 )}

                 {/* Property Inspector - Below Generator Stack or Separate?
                     Let's put it next to it or allow it to be separate.
                     For now, maybe just below it if selected.
                 */}
                 {showInspector && (
                     <div className="absolute top-4 right-[22rem] z-20">
                         {builder.state.selectedSegmentIndex !== null && (
                             <PropertyInspector builder={builder} />
                         )}
                     </div>
                 )}
            </div>

            {/* Bottom Timeline Panel */}
            {showTimeline && (
                <div className="h-48 z-30 shadow-[0_-5px_20px_rgba(0,0,0,0.5)]">
                     <StateTimeline builder={builder} />
                </div>
            )}
        </div>
    );
}
