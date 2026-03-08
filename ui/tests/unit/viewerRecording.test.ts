import { describe, expect, it } from 'vitest';

import {
  buildRecordingFilename,
  selectRecordingFormat,
} from '../../src/utils/viewerRecording';

describe('viewerRecording', () => {
  it('selects the first supported recording format', () => {
    const supported = new Set(['video/webm;codecs=vp9', 'video/webm']);
    const format = selectRecordingFormat((mimeType) => supported.has(mimeType));
    expect(format).toEqual({
      mimeType: 'video/webm;codecs=vp9',
      extension: 'webm',
    });
  });

  it('returns null when no recording format is supported', () => {
    expect(selectRecordingFormat(() => false)).toBeNull();
    expect(selectRecordingFormat(undefined)).toBeNull();
  });

  it('builds deterministic filenames with sanitized run ids', () => {
    const filename = buildRecordingFilename(
      '07-03-2026 09:12 run',
      'webm',
      new Date(2026, 2, 7, 9, 31, 26)
    );
    expect(filename).toBe('viewer-replay-07-03-2026_09_12_run-20260307_093126.webm');
  });
});
