import React, { useEffect, useMemo, useState } from 'react';
import {
  ChevronLeft,
  Download,
  FileCode,
  FileText,
  Film,
  Folder,
  Image as ImageIcon,
} from 'lucide-react';
import { API_BASE_URL } from '../config/endpoints';
import type { SimulationRun } from '../api/simulations';
import { runEvents } from '../services/runEvents';
import { Panel } from './ui-v4/Panel';
import { StatusPill } from './ui-v4/StatusPill';

interface FileNode {
  path: string;
  name: string;
  size?: number;
  type: 'file' | 'directory';
  children?: FileNode[];
}

const RUN_LIST_REFRESH_MS = 5000;

function formatUpdatedAt(timestamp: number | null): string {
  if (!timestamp) return '--';
  return new Date(timestamp).toLocaleTimeString();
}

function describeFileKind(file: FileNode | null): string {
  if (!file) return 'Select an artifact to inspect.';
  if (file.type === 'directory') return 'Open folder';
  const ext = file.name.split('.').pop()?.toLowerCase();
  if (ext === 'json') return 'Structured config / metrics file';
  if (ext === 'csv') return 'Tabular export';
  if (ext === 'mp4' || ext === 'webm') return 'Rendered media output';
  if (ext === 'png' || ext === 'jpg' || ext === 'jpeg' || ext === 'gif') return 'Image artifact';
  if (ext === 'txt' || ext === 'log' || ext === 'md' || ext === 'py') return 'Text preview';
  return 'Artifact preview';
}

export const SimulationDataView: React.FC = () => {
  const [runs, setRuns] = useState<SimulationRun[]>([]);
  const [refreshingRuns, setRefreshingRuns] = useState(false);
  const [lastRunsRefreshAt, setLastRunsRefreshAt] = useState<number | null>(null);
  const [selectedRunId, setSelectedRunId] = useState<string | null>(null);
  const [files, setFiles] = useState<FileNode[]>([]);
  const [currentDir, setCurrentDir] = useState<string>('');
  const [selectedFile, setSelectedFile] = useState<FileNode | null>(null);
  const [fileContent, setFileContent] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [contentType, setContentType] = useState<'text' | 'image' | 'video' | 'other' | null>(null);

  const applyRunsUpdate = (nextRuns: SimulationRun[]) => {
    setRuns(nextRuns);
    setLastRunsRefreshAt(Date.now());
    if (selectedRunId && !nextRuns.some((run) => run.id === selectedRunId)) {
      setSelectedRunId(null);
      setSelectedFile(null);
      setFiles([]);
      setFileContent(null);
      setContentType(null);
    }
  };

  useEffect(() => {
    const unsubscribe = runEvents.subscribe((event) => {
      applyRunsUpdate(event.runs);
    });
    return () => unsubscribe();
  }, [selectedRunId]);

  useEffect(() => {
    let cancelled = false;
    const loadRuns = () => {
      setRefreshingRuns(true);
      fetch(`${API_BASE_URL}/simulations`)
        .then((res) => {
          if (!res.ok) throw new Error(`HTTP ${res.status}`);
          return res.json();
        })
        .then((data) => {
          if (cancelled) return;
          applyRunsUpdate(data.runs || []);
        })
        .catch((err) => console.error('Failed to fetch runs:', err))
        .finally(() => {
          if (!cancelled) setRefreshingRuns(false);
        });
    };
    loadRuns();
    const timer = window.setInterval(loadRuns, RUN_LIST_REFRESH_MS);
    const onFocus = () => loadRuns();
    window.addEventListener('focus', onFocus);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
      window.removeEventListener('focus', onFocus);
    };
  }, []);

  useEffect(() => {
    if (!selectedRunId) {
      setFiles([]);
      setCurrentDir('');
      return;
    }

    fetch(`${API_BASE_URL}/simulations/${selectedRunId}/files`)
      .then((res) => {
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
      })
      .then((data) => {
        setFiles(data.files || []);
        setCurrentDir('');
      })
      .catch((err) => console.error('Failed to fetch files:', err));
  }, [selectedRunId]);

  const visibleEntries = useMemo(() => {
    const parentOf = (path: string) => {
      const idx = path.lastIndexOf('/');
      return idx === -1 ? '' : path.slice(0, idx);
    };
    return files
      .filter((file) => parentOf(file.path) === currentDir)
      .sort((a, b) => {
        if (a.type !== b.type) return a.type === 'directory' ? -1 : 1;
        return a.name.localeCompare(b.name);
      });
  }, [files, currentDir]);

  useEffect(() => {
    if (!selectedRunId || !selectedFile || selectedFile.type === 'directory') {
      setFileContent(null);
      setContentType(null);
      return;
    }

    const ext = selectedFile.name.split('.').pop()?.toLowerCase();

    if (['png', 'jpg', 'jpeg', 'gif'].includes(ext || '')) {
      setContentType('image');
      setFileContent(`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`);
      return;
    }

    if (['mp4', 'webm'].includes(ext || '')) {
      setContentType('video');
      setFileContent(`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`);
      return;
    }

    if (['txt', 'log', 'csv', 'json', 'md', 'py'].includes(ext || '')) {
      setContentType('text');
      setIsLoading(true);
      setFileContent(null);
      const controller = new AbortController();
      fetch(`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`, {
        signal: controller.signal,
      })
        .then((res) => {
          if (!res.ok) throw new Error(`HTTP ${res.status}`);
          return res.text();
        })
        .then((text) => setFileContent(text))
        .catch((err) => {
          if (err.name === 'AbortError') return;
          console.error(err);
          setFileContent('Error loading file content.');
        })
        .finally(() => setIsLoading(false));
      return () => controller.abort();
    }

    setContentType('other');
    setFileContent(null);
  }, [selectedRunId, selectedFile]);

  const selectedRun = useMemo(
    () => runs.find((run) => run.id === selectedRunId) ?? null,
    [runs, selectedRunId]
  );

  return (
    <div className="h-full w-full overflow-hidden bg-[color:var(--v4-bg)] text-[color:var(--v4-text-1)]">
      <div className="flex h-full flex-col gap-4 overflow-hidden p-4">
        <div className="grid min-h-0 flex-1 gap-4 xl:grid-cols-[280px_320px_minmax(0,1fr)]">
          <Panel
            title="Saved Runs"
            className="min-h-0 flex flex-col"
            bodyClassName="flex min-h-0 flex-1 flex-col p-0"
          >
            <div className="min-h-0 flex-1 overflow-y-auto">
              {runs.length === 0 ? (
                <div className="flex h-full items-center justify-center px-6 text-center text-sm text-[color:var(--v4-text-3)]">
                  No saved runs yet. Launch a simulation in Runner, then come back here to inspect
                  its outputs.
                </div>
              ) : (
                runs.map((run) => (
                  <button
                    key={run.id}
                    type="button"
                    onClick={() => {
                      setSelectedRunId(run.id);
                      setSelectedFile(null);
                      setCurrentDir('');
                    }}
                    className={`flex w-full flex-col gap-1 border-b border-[color:var(--v4-border)]/60 px-4 py-3 text-left transition-colors ${
                      selectedRunId === run.id
                        ? 'bg-cyan-950/35 text-cyan-100'
                        : 'text-[color:var(--v4-text-2)] hover:bg-white/5'
                    }`}
                  >
                    <div className="text-[12px] font-semibold leading-snug">{run.id}</div>
                  </button>
                ))
              )}
            </div>
          </Panel>

          <Panel className="min-h-0 flex flex-col" bodyClassName="flex min-h-0 flex-1 flex-col p-0">
            <div className="flex items-center justify-between gap-3 border-b border-[color:var(--v4-border)]/60 px-4 py-2 text-xs text-[color:var(--v4-text-3)]">
              <span className="truncate">/{currentDir}</span>
              {selectedRunId ? (
                currentDir ? (
                  <button
                    type="button"
                    onClick={() => {
                      const idx = currentDir.lastIndexOf('/');
                      setCurrentDir(idx === -1 ? '' : currentDir.slice(0, idx));
                      setSelectedFile(null);
                    }}
                    className="inline-flex items-center gap-1 rounded-lg border border-[color:var(--v4-border)] px-2 py-1 text-[11px] text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200"
                  >
                    <ChevronLeft size={12} />
                    Back
                  </button>
                ) : (
                  <StatusPill tone="neutral">Root</StatusPill>
                )
              ) : null}
            </div>
            <div className="min-h-0 flex-1 overflow-y-auto">
              {!selectedRunId ? (
                <div className="flex h-full items-center justify-center px-6 text-center text-sm text-[color:var(--v4-text-3)]">
                  Select a saved run to browse its result tree.
                </div>
              ) : visibleEntries.length === 0 ? (
                <div className="flex h-full items-center justify-center px-6 text-center text-sm text-[color:var(--v4-text-3)]">
                  No artifacts found in this directory.
                </div>
              ) : (
                visibleEntries.map((file) => (
                  <button
                    key={file.path}
                    type="button"
                    onClick={() => {
                      if (file.type === 'directory') {
                        setCurrentDir(file.path);
                        setSelectedFile(null);
                        return;
                      }
                      setSelectedFile(file);
                    }}
                    className={`flex w-full items-center gap-2 border-b border-[color:var(--v4-border)]/50 px-4 py-2 text-left transition-colors ${
                      selectedFile?.path === file.path
                        ? 'bg-cyan-950/35 text-cyan-100'
                        : 'text-[color:var(--v4-text-2)] hover:bg-white/5'
                    }`}
                  >
                    {file.type === 'directory' ? (
                      <Folder size={14} className="text-yellow-400" />
                    ) : file.name.endsWith('.json') ? (
                      <FileCode size={14} className="text-emerald-400" />
                    ) : file.name.endsWith('.csv') ? (
                      <FileText size={14} className="text-cyan-400" />
                    ) : file.name.endsWith('.mp4') || file.name.endsWith('.webm') ? (
                      <Film size={14} className="text-purple-400" />
                    ) : file.name.endsWith('.png') || file.name.endsWith('.jpg') || file.name.endsWith('.jpeg') ? (
                      <ImageIcon size={14} className="text-amber-400" />
                    ) : (
                      <FileText size={14} />
                    )}
                    <span className="truncate text-[12px]">{file.name}</span>
                  </button>
                ))
              )}
            </div>
          </Panel>

          <Panel className="min-h-0 flex flex-col" bodyClassName="flex min-h-0 flex-1 flex-col p-0">
            <div className="relative min-h-0 flex-1 overflow-hidden bg-[#050816]">
              {selectedFile && selectedRunId ? (
                <div className="absolute right-4 top-4 z-10">
                  <a
                    href={`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`}
                    target="_blank"
                    rel="noreferrer"
                    className="inline-flex items-center gap-2 rounded-lg border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]/80 px-3 py-1.5 text-xs text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200"
                  >
                    <Download size={14} />
                    Download
                  </a>
                </div>
              ) : null}
              {isLoading ? (
                <div className="absolute inset-0 flex items-center justify-center text-sm text-slate-500">
                  Loading artifact preview…
                </div>
              ) : null}

              {selectedFile && !isLoading && contentType === 'text' ? (
                <pre className="h-full w-full overflow-auto p-4 text-xs leading-relaxed text-slate-300">
                  {fileContent}
                </pre>
              ) : null}

              {selectedFile && !isLoading && contentType === 'image' ? (
                <div className="flex h-full w-full items-center justify-center p-4">
                  <img
                    src={fileContent as string}
                    alt={selectedFile.name}
                    className="max-h-full max-w-full object-contain border border-slate-700"
                  />
                </div>
              ) : null}

              {selectedFile && !isLoading && contentType === 'video' ? (
                <div className="flex h-full w-full items-center justify-center bg-black p-4">
                  <video controls src={fileContent as string} className="max-h-full max-w-full" />
                </div>
              ) : null}

              {selectedFile && !isLoading && contentType === 'other' ? (
                <div className="absolute inset-0 flex items-center justify-center px-8 text-center text-sm text-slate-500">
                  Preview is not available for this file type. Download the artifact to inspect it
                  externally.
                </div>
              ) : null}
            </div>
          </Panel>
        </div>
      </div>
    </div>
  );
};
