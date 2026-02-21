import React, { useMemo, useState, useEffect } from 'react';
import { Folder, FileText, Film, Image as ImageIcon, FileCode, ChevronLeft, Download } from 'lucide-react';
import { API_BASE_URL } from '../config/endpoints';
import { runEvents } from '../services/runEvents';
import type { SimulationRun } from '../api/simulations';

interface FileNode {
  path: string;
  name: string;
  size?: number;
  type: 'file' | 'directory';
  children?: FileNode[];
}

const RUN_LIST_REFRESH_MS = 5000;

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

  // Fetch runs on mount
  useEffect(() => {
    let cancelled = false;
    const loadRuns = () => {
      setRefreshingRuns(true);
      fetch(`${API_BASE_URL}/simulations`)
        .then(res => res.json())
        .then(data => {
          if (cancelled) return;
          applyRunsUpdate(data.runs || []);
        })
        .catch(err => console.error("Failed to fetch runs:", err))
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

  // Fetch files when run is selected
  useEffect(() => {
    if (!selectedRunId) {
        setFiles([]);
        setCurrentDir('');
        return;
    }

    fetch(`${API_BASE_URL}/simulations/${selectedRunId}/files`)
      .then(res => res.json())
      .then(data => {
          // Flattened list from API, let's keep it simple or tree-ify if needed.
          // For now, listing flat is okay, but tree is better.
          // The API returns a flat list of all files/dirs recursively.
          // Let's just display them as a list for now, maybe sort by path.
          setFiles(data.files || []);
          setCurrentDir('');
      })
      .catch(err => console.error("Failed to fetch files:", err));
  }, [selectedRunId]);

  const visibleEntries = useMemo(() => {
    const parentOf = (p: string) => {
      const idx = p.lastIndexOf('/');
      return idx === -1 ? '' : p.slice(0, idx);
    };
    return files
      .filter((f) => parentOf(f.path) === currentDir)
      .sort((a, b) => {
        if (a.type !== b.type) return a.type === 'directory' ? -1 : 1;
        return a.name.localeCompare(b.name);
      });
  }, [files, currentDir]);

  // Fetch content when file is selected
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
    } else if (['mp4', 'webm'].includes(ext || '')) {
        setContentType('video');
        setFileContent(`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`);
    } else if (['txt', 'log', 'csv', 'json', 'md', 'py'].includes(ext || '')) {
        setContentType('text');
        setIsLoading(true);
        fetch(`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`)
            .then(res => res.text())
            .then(text => {
                setFileContent(text);
                setIsLoading(false);
            })
            .catch(err => {
                console.error(err);
                setFileContent("Error loading file content.");
                setIsLoading(false);
            });
    } else {
        setContentType('other');
        setFileContent(null);
    }

  }, [selectedRunId, selectedFile]);

  return (
    <div className="flex h-full text-white overflow-hidden font-mono text-sm">
      {/* Sidebar: Runs */}
      <div className="w-64 border-r border-slate-700 flex flex-col bg-slate-900/50">
        <div className="p-3 border-b border-slate-700 font-bold text-slate-300">
          Simulation Runs
          <div className="text-[10px] font-normal text-slate-500 mt-1">
            {refreshingRuns
              ? 'Refreshing...'
              : `Updated ${
                  lastRunsRefreshAt
                    ? new Date(lastRunsRefreshAt).toLocaleTimeString()
                    : '--:--:--'
                }`}
          </div>
        </div>
        <div className="flex-1 overflow-y-auto">
          {runs.map(run => (
            <div
              key={run.id}
              onClick={() => {
                setSelectedRunId(run.id);
                setSelectedFile(null);
                setCurrentDir('');
              }}
              className={`p-3 cursor-pointer hover:bg-slate-800 border-b border-slate-800/50 ${selectedRunId === run.id ? 'bg-blue-900/30 text-blue-300' : 'text-slate-400'}`}
            >
              <div className="font-semibold">{run.id}</div>
              <div className="text-xs opacity-60 mt-1">
                {new Date(run.modified * 1000).toLocaleString()}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Middle: File List */}
      <div className="w-72 border-r border-slate-700 flex flex-col bg-slate-900/30">
        <div className="p-3 border-b border-slate-700 font-bold text-slate-300 flex justify-between items-center">
          <span>Files</span>
          {selectedRunId && <span className="text-xs opacity-50">{selectedRunId}</span>}
        </div>
        {selectedRunId && (
          <div className="px-3 py-2 border-b border-slate-800 text-xs text-slate-400 flex items-center gap-2">
            {currentDir ? (
              <button
                type="button"
                onClick={() => {
                  const idx = currentDir.lastIndexOf('/');
                  setCurrentDir(idx === -1 ? '' : currentDir.slice(0, idx));
                  setSelectedFile(null);
                }}
                className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-200 flex items-center gap-1"
              >
                <ChevronLeft size={12} />
                Up
              </button>
            ) : (
              <span className="px-2 py-1 rounded bg-slate-800 text-slate-500">Root</span>
            )}
            <span className="truncate">/{currentDir}</span>
          </div>
        )}
        <div className="flex-1 overflow-y-auto">
          {visibleEntries.map((file) => (
             <div
                key={file.path}
                onClick={() => {
                  if (file.type === 'directory') {
                    setCurrentDir(file.path);
                    setSelectedFile(null);
                    return;
                  }
                  setSelectedFile(file);
                }}
                className={`p-2 px-4 cursor-pointer hover:bg-slate-800 flex items-center gap-2 ${selectedFile?.path === file.path ? 'bg-blue-900/30 text-blue-300' : 'text-slate-400'}`}
             >
                {file.type === 'directory' ? <Folder size={14} className="text-yellow-500" /> :
                 file.name.endsWith('.json') ? <FileCode size={14} className="text-green-500" /> :
                 file.name.endsWith('.csv') ? <FileText size={14} className="text-blue-500" /> :
                 file.name.endsWith('.mp4') ? <Film size={14} className="text-purple-500" /> :
                 file.name.endsWith('.png') ? <ImageIcon size={14} className="text-orange-500" /> :
                 <FileText size={14} />
                }
                <span className="truncate">{file.name}</span>
             </div>
          ))}
          {visibleEntries.length === 0 && selectedRunId && (
              <div className="p-4 text-center opacity-50 italic">No files found</div>
          )}
           {!selectedRunId && (
              <div className="p-4 text-center opacity-50 italic">Select a run</div>
          )}
        </div>
      </div>

      {/* Main: Content Preview */}
      <div className="flex-1 flex flex-col bg-black">
        <div className="p-3 border-b border-slate-700 flex justify-between items-center bg-slate-900">
          <span className="font-bold text-slate-300">
            {selectedFile ? selectedFile.name : "Preview"}
          </span>
          {selectedFile && selectedRunId && (
             <a
                href={`${API_BASE_URL}/simulations/${selectedRunId}/files/${selectedFile.path}`}
                target="_blank"
                rel="noreferrer"
                className="flex items-center gap-2 text-xs bg-slate-700 hover:bg-slate-600 px-3 py-1 rounded transition-colors"
             >
                <Download size={14} /> Download
             </a>
          )}
        </div>
        <div className="flex-1 overflow-hidden relative">
            {isLoading && (
                <div className="absolute inset-0 flex items-center justify-center text-slate-500">
                    Loading...
                </div>
            )}

            {!selectedFile && (
                <div className="absolute inset-0 flex items-center justify-center text-slate-600">
                    Select a file to view content
                </div>
            )}

            {selectedFile && !isLoading && contentType === 'text' && (
                <pre className="w-full h-full overflow-auto p-4 text-xs text-slate-300 leading-relaxed custom-scrollbar">
                    {fileContent}
                </pre>
            )}

            {selectedFile && !isLoading && contentType === 'image' && (
                <div className="w-full h-full flex items-center justify-center p-4">
                    <img src={fileContent as string} alt={selectedFile.name} className="max-w-full max-h-full object-contain border border-slate-700" />
                </div>
            )}

            {selectedFile && !isLoading && contentType === 'video' && (
                <div className="w-full h-full flex items-center justify-center p-4 bg-black">
                     <video controls src={fileContent as string} className="max-w-full max-h-full" />
                </div>
            )}

             {selectedFile && !isLoading && contentType === 'other' && (
                <div className="absolute inset-0 flex items-center justify-center text-slate-500">
                    Preview not available for this file type.
                </div>
            )}
        </div>
      </div>
    </div>
  );
};
