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
    const maxThrusters = Math.max(...history.map(d => d.thrusters?.length ?? 0));
    const maxRw = Math.max(...history.map(d => d.rw_torque?.length ?? 0));

    const headers = [
      'time',
      'pos_x', 'pos_y', 'pos_z',
      'vel_x', 'vel_y', 'vel_z',
      'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
      'target_x', 'target_y', 'target_z',
      'target_roll', 'target_pitch', 'target_yaw',
      'pos_error', 'ang_error', 'solve_time',
      'paused', 'sim_speed',
    ];

    for (let i = 0; i < maxThrusters; i += 1) {
      headers.push(`thruster_${i}`);
    }
    for (let i = 0; i < maxRw; i += 1) {
      headers.push(`rw_${i}`);
    }

    const rows = history.map((d) => {
      const row = [
        d.time,
        d.position[0], d.position[1], d.position[2],
        d.velocity[0], d.velocity[1], d.velocity[2],
        d.angular_velocity[0], d.angular_velocity[1], d.angular_velocity[2],
        d.target_position[0], d.target_position[1], d.target_position[2],
        d.target_orientation[0], d.target_orientation[1], d.target_orientation[2],
        d.pos_error ?? 0,
        d.ang_error ?? 0,
        d.solve_time ?? 0,
        d.paused ?? false,
        d.sim_speed ?? 1.0,
      ];
      for (let i = 0; i < maxThrusters; i += 1) {
        row.push(d.thrusters?.[i] ?? '');
      }
      for (let i = 0; i < maxRw; i += 1) {
        row.push(d.rw_torque?.[i] ?? '');
      }
      return row;
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
