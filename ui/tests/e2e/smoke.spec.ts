import { expect, test } from '@playwright/test';

test('app shell loads', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByText('MISSION CONTROL')).toBeVisible();
  await expect(page.getByRole('button', { name: 'RUNNER' })).toBeVisible();
});

test('planner flow renders unified steps and save/launch is gated', async ({ page }) => {
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
  await expect(page.getByRole('button', { name: 'Scan Definition' })).toBeVisible();
  await page.getByRole('button', { name: 'Quick Inspect' }).click();
  await page.getByRole('button', { name: 'Run Validation' }).first().click();

  const scanDefinitionStep = page.getByRole('button', { name: /^Scan Definition/ });
  await expect(scanDefinitionStep).toContainText('1');

  await page.getByRole('button', { name: 'Save/Launch' }).click();
  await expect(page.getByRole('button', { name: 'Save Mission' })).toBeDisabled();
  await expect(page.getByRole('button', { name: 'Launch Mission' })).toBeDisabled();
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
