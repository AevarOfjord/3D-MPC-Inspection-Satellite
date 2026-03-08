export interface RecordingFormat {
  mimeType: string;
  extension: 'mp4' | 'webm';
}

const CANDIDATE_FORMATS: RecordingFormat[] = [
  { mimeType: 'video/mp4', extension: 'mp4' },
  { mimeType: 'video/webm;codecs=vp9', extension: 'webm' },
  { mimeType: 'video/webm', extension: 'webm' },
];

export function selectRecordingFormat(
  isTypeSupported: ((mimeType: string) => boolean) | undefined
): RecordingFormat | null {
  if (!isTypeSupported) return null;
  for (const candidate of CANDIDATE_FORMATS) {
    if (isTypeSupported(candidate.mimeType)) {
      return candidate;
    }
  }
  return null;
}

function pad(value: number): string {
  return value.toString().padStart(2, '0');
}

function sanitizeRunId(runId: string): string {
  const trimmed = runId.trim();
  if (!trimmed) return 'unknown-run';
  return trimmed.replace(/[^a-zA-Z0-9_-]+/g, '_');
}

export function buildRecordingFilename(
  runId: string,
  extension: RecordingFormat['extension'],
  now = new Date()
): string {
  const stamp = [
    now.getFullYear(),
    pad(now.getMonth() + 1),
    pad(now.getDate()),
  ].join('');
  const time = [pad(now.getHours()), pad(now.getMinutes()), pad(now.getSeconds())].join('');
  return `viewer-replay-${sanitizeRunId(runId)}-${stamp}_${time}.${extension}`;
}
