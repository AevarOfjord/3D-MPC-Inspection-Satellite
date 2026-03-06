import { createElement } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { beforeEach, describe, expect, it } from 'vitest';

import { useStudioStore } from '../useStudioStore';

function seedBaseState() {
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
}

describe('MissionStudioRightPanel', () => {
  beforeEach(() => {
    (globalThis as { window?: unknown }).window = {
      location: { hostname: 'localhost', port: '4173', protocol: 'http:' },
    };
    seedBaseState();
  });

  it('shows empty-state save guidance before authoring begins', async () => {
    const { MissionStudioRightPanel } = await import('../MissionStudioRightPanel');
    const markup = renderToStaticMarkup(createElement(MissionStudioRightPanel));

    expect(markup).toContain('Name and assembly required');
    expect(markup).toContain('Add segments using the left panel');
    expect(markup).toContain('Validate the mission before saving');
  });
});
