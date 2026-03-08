import { createElement } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { beforeEach, describe, expect, it, vi } from 'vitest';

describe('MissionStudioRightPanel', () => {
  beforeEach(() => {
    vi.resetModules();
    (globalThis as { window?: unknown }).window = {
      location: { hostname: 'localhost', port: '4173', protocol: 'http:' },
    };
  });

  it('shows empty-state save guidance before authoring begins', async () => {
    const { useStudioStore } = await import('../useStudioStore');
    useStudioStore.setState({
      missionName: '',
      validationReport: null,
      validationBusy: false,
      saveBusy: false,
      assembly: [],
      paths: [],
      wires: [],
      holds: [],
      obstacles: [],
      points: [],
      satelliteStart: [0, 0, 20],
    } as never);
    const { MissionStudioRightPanel } = await import('../MissionStudioRightPanel');
    const markup = renderToStaticMarkup(createElement(MissionStudioRightPanel));

    expect(markup).toContain('Enter a mission name to unlock validation.');
    expect(markup).toContain('Name Mission First');
    expect(markup).toContain('Add segments using the left panel');
    expect(markup).toContain('Validate the mission before saving');
    expect(markup).toContain('Route: empty');
    expect(markup).toContain('Route Issues (0)');
  });
});
