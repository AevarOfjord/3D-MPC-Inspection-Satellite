import { useMemo } from 'react';
import { useTelemetryStore } from '../store/telemetryStore';
import { useViewportStore } from '../store/viewportStore';
import { useUiStore } from '../store/uiStore';

export function ExportControls() {
  const history = useTelemetryStore(s => s.history);
  const canvas = useViewportStore(s => s.canvas);
  const setMissionError = useUiStore(s => s.setMissionError);

  const csvPayload = useMemo(() => {
    if (history.length === 0) return null;

    const headers = [
      'time',
      'pos_error',
      'ang_error_deg',
      'velocity',
      'solve_time_ms',
    ];

    const rows = history.map((d) => {
      return [
        d.time,
        d.posError,
        d.angError,
        d.velocity,
        d.solveTime,
      ];
    });

    return [headers.join(','), ...rows.map(r => r.join(','))].join('\n');
  }, [history]);

  const download = (content: BlobPart, filename: string, type: string) => {
    const blob = new Blob([content], { type });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    link.click();
    URL.revokeObjectURL(url);
  };

  const handleExportCsv = () => {
    if (!csvPayload) {
      setMissionError('No telemetry data to export');
      return;
    }
    download(csvPayload, 'telemetry.csv', 'text/csv');
  };

  const handleScreenshot = () => {
    if (!canvas) {
      setMissionError('Viewport not ready for screenshot');
      return;
    }
    canvas.toBlob((blob) => {
      if (!blob) {
        setMissionError('Failed to capture screenshot');
        return;
      }
      download(blob, 'viewport.png', 'image/png');
    }, 'image/png');
  };

  return (
    <div className="flex items-center gap-2 bg-gray-900/80 border border-gray-700 rounded-lg px-3 py-2">
      <button
        onClick={handleExportCsv}
        className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
      >
        Export CSV
      </button>
      <button
        onClick={handleScreenshot}
        className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
      >
        Screenshot
      </button>
    </div>
  );
}
