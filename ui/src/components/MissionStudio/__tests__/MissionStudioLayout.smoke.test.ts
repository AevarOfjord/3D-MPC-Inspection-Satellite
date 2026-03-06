import { createElement } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { beforeEach, describe, expect, it, vi } from 'vitest';

import { useStudioStore } from '../useStudioStore';

vi.mock('../MissionStudioCanvas', () => ({
  MissionStudioCanvas: () =>
    createElement('div', { 'data-testid': 'mission-studio-canvas' }, 'Canvas Stub'),
}));

describe('MissionStudioLayout', () => {
  beforeEach(() => {
    (globalThis as { window?: unknown }).window = {
      location: { hostname: 'localhost', port: '4173', protocol: 'http:' },
    };
    useStudioStore.setState({
      welcomeDismissed: false,
      missionName: '',
      assembly: [],
      paths: [],
      wires: [],
      holds: [],
      obstacles: [],
      points: [],
    } as never);
  });

  it('renders the Studio shell with welcome overlay by default', async () => {
    const { MissionStudioLayout } = await import('../MissionStudioLayout');
    const markup = renderToStaticMarkup(createElement(MissionStudioLayout));

    expect(markup).toContain('Choose a target object');
    expect(markup).toContain('Create Path');
    expect(markup).toContain('Studio Status');
    expect(markup).toContain('Canvas Stub');
  });
});
