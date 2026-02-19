import { expect, test, type Page } from '@playwright/test';

async function dismissIntroIfVisible(page: Page) {
  const dismiss = page.getByRole('button', { name: 'Dismiss' });
  const visible = await dismiss
    .waitFor({ state: 'visible', timeout: 1200 })
    .then(() => true)
    .catch(() => false);
  if (visible) {
    await dismiss.click();
  }
}

test('app shell loads', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByText('MISSION CONTROL')).toBeVisible();
  await expect(page.getByRole('button', { name: 'RUNNER' })).toBeVisible();
});

test('planner flow renders V4.2 steps and mission saver is gated', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
    window.localStorage.setItem('mission_control_planner_ux_mode_v1', 'advanced');
  });
  await page.route('**/api/v2/missions/validate', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        valid: false,
        issues: [
          {
            code: 'MISSING_PATH_ASSET',
            severity: 'error',
            path: 'segments[0].path_asset',
            message: 'Path asset required',
            suggestion: 'Select a saved path asset',
          },
        ],
        summary: { errors: 1, warnings: 0, info: 0 },
      }),
    });
  });

  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await dismissIntroIfVisible(page);
  await expect
    .poll(async () =>
      page.evaluate(() => window.localStorage.getItem('mission_control_planner_ux_mode_v1'))
    )
    .toBe('advanced');
  await expect(page.getByRole('button', { name: /Path Maker/ })).toBeVisible();
  await page.getByRole('button', { name: 'Quick Inspect' }).click();
  await page.keyboard.press('Alt+5');
  await expect(page.getByRole('heading', { name: 'Step 5 · Save Mission' })).toBeVisible();
  await expect(page.getByRole('button', { name: /^Save Mission$/ })).toBeDisabled();
  await expect(page.getByRole('button', { name: /^Open Runner$/ })).toBeDisabled();
});

test('draft restore banner is one-shot and discard clears it', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.setItem('mission_control_draft_id_v2', 'draft-e2e-1');
  });
  await page.route('**/api/v2/missions/drafts/draft-e2e-1', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        draft_id: 'draft-e2e-1',
        revision: 2,
        saved_at: '2026-02-17T11:24:58Z',
        mission: {
          schema_version: 2,
          mission_id: 'M-e2e',
          name: 'DraftE2E',
          epoch: '2026-02-17T00:00:00Z',
          start_pose: { frame: 'ECI', position: [0, 0, 0] },
          segments: [],
          metadata: { version: 1 },
          overrides: {},
        },
      }),
    });
  });

  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await expect(page.getByText('Restore mission draft from')).toBeVisible();
  await page.getByRole('button', { name: 'Discard' }).click();
  await expect(page.getByText('Restore mission draft from')).toHaveCount(0);
  await page.reload();
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await expect(page.getByText('Restore mission draft from')).toHaveCount(0);
});

test('command palette and planner shortcuts work', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
    window.localStorage.setItem('mission_control_planner_ux_mode_v1', 'advanced');
  });
  await page.goto('/');
  await page.getByRole('button', { name: /Command Palette/ }).click();
  await expect(page.getByPlaceholder('Search commands...')).toBeVisible();
  await page.getByPlaceholder('Search commands...').fill('switch to planner');
  await page.keyboard.press('Enter');
  await expect(page.getByRole('button', { name: /Path Maker/ })).toBeVisible();
  await expect
    .poll(async () =>
      page.evaluate(() => window.localStorage.getItem('mission_control_planner_ux_mode_v1'))
    )
    .toBe('advanced');

  await page.keyboard.press('Alt+5');
  await expect(page.getByText('Step 5 · Save Mission')).toBeVisible();

  await page.keyboard.press('Escape');
  await page.keyboard.press('?');
  await expect(page.getByText('Keyboard Shortcuts')).toBeVisible();
});

test('guided advanced mode persists and onboarding banner is one-shot', async ({ page }) => {
  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();

  await expect(page.getByText('Take 60s Tour')).toBeVisible();
  await page.getByRole('button', { name: 'Dismiss' }).click();
  await expect(page.getByText('Take 60s Tour')).toHaveCount(0);

  await page.getByRole('button', { name: 'Advanced' }).click();
  await expect
    .poll(async () =>
      page.evaluate(() => window.localStorage.getItem('mission_control_planner_ux_mode_v1'))
    )
    .toBe('advanced');

  await page.reload();
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await expect(page.getByText('Take 60s Tour')).toHaveCount(0);
  await expect
    .poll(async () =>
      page.evaluate(() => window.localStorage.getItem('mission_control_planner_ux_mode_v1'))
    )
    .toBe('advanced');
});
