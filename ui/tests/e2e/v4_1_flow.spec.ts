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

test('v4.1 happy path: 5-step mission flow stays usable and save-ready', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
    window.localStorage.setItem('mission_control_planner_ux_mode_v1', 'advanced');
  });
  await page.route('**/api/v2/missions/validate', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        valid: true,
        issues: [],
        summary: { errors: 0, warnings: 0, info: 0 },
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

  await expect(page.getByRole('button', { name: /Path Library/ })).toBeVisible();
  await expect(page.getByRole('button', { name: /Start \+ Transfer/ })).toBeVisible();
  await expect(page.getByRole('button', { name: /Obstacles/ })).toBeVisible();
  await expect(page.getByRole('button', { name: /Path Edit/ })).toBeVisible();
  await expect(page.getByRole('button', { name: /Save Mission/ })).toBeVisible();

  await page
    .locator('#coachmark-step_rail')
    .getByRole('button', { name: /Start \+ Transfer/ })
    .first()
    .click();
  await expect(page.getByRole('heading', { name: 'Step 2 · Start + Auto Transfer' })).toBeVisible();
  await page.getByRole('button', { name: /\+ Transfer Segment/ }).click();

  await page.keyboard.press('Alt+5');
  await expect(page.getByRole('heading', { name: 'Step 5 · Save Mission' })).toBeVisible();
  const saveButton = page.locator('#coachmark-save_launch').getByRole('button', { name: /^Save$/ });
  await expect.poll(async () => saveButton.isEnabled()).toBe(true);
});

test('v4.1 failure path: validation issue can route back to the fixing step', async ({ page }) => {
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
  await page.locator('#coachmark-step_rail').getByRole('button', { name: /Save Mission/ }).first().click();
  await expect(page.getByRole('heading', { name: 'Step 5 · Save Mission' })).toBeVisible();

  await expect(page.getByText('Validation Requires Attention')).toBeVisible();
  await expect(
    page.locator('#coachmark-step_rail').getByRole('button', { name: /Path Library/ }).first()
  ).toContainText('1');
});
