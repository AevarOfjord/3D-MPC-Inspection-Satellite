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

test('v4.2 happy path: 5-step mission flow stays usable and save-ready', async ({ page }) => {
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

  const stepRail = page.locator('#coachmark-step_rail');
  await expect(stepRail.getByRole('button', { name: /Step 1/ }).first()).toContainText('Path Maker');
  await expect(stepRail.getByRole('button', { name: /Step 2/ }).first()).toContainText('Transfer');
  await expect(stepRail.getByRole('button', { name: /Step 3/ }).first()).toContainText('Obstacles');
  await expect(stepRail.getByRole('button', { name: /Step 4/ }).first()).toContainText('Path Edit');
  await expect(stepRail.getByRole('button', { name: /Step 5/ }).first()).toContainText('Mission Saver');

  await stepRail.getByRole('button', { name: /Step 2/ }).first().click();
  await expect(page.getByRole('heading', { name: 'Step 2 · Transfer' })).toBeVisible();

  await page.keyboard.press('Alt+5');
  await expect(page.getByRole('heading', { name: 'Step 5 · Save Mission' })).toBeVisible();
  const saveButton = page.locator('#coachmark-save').getByRole('button', { name: /^Save Mission$/ });
  await expect.poll(async () => saveButton.isEnabled()).toBe(true);
});

test('v4.2 failure path: validation issue can route back to the fixing step', async ({ page }) => {
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
  await page.locator('#coachmark-step_rail').getByRole('button', { name: /Step 5/ }).first().click();
  await expect(page.getByRole('heading', { name: 'Step 5 · Save Mission' })).toBeVisible();

  await expect(page.getByText('Validation Requires Attention')).toBeVisible();
  await expect(
    page.locator('#coachmark-step_rail').getByRole('button', { name: /Path Maker/ }).first()
  ).toContainText('1');
});
