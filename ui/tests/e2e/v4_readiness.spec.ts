import { expect, test, type Page } from '@playwright/test';

async function dismissIntroIfVisible(page: Page) {
  const dismiss = page.getByRole('button', { name: 'Dismiss' });
  if (await dismiss.isVisible().catch(() => false)) {
    await dismiss.click();
  }
}

test('v4 kpi: create + validate + reach save-ready state within 5 minutes from clean storage', async ({ page }) => {
  let validateCalls = 0;

  await page.addInitScript(() => {
    window.localStorage.clear();
  });

  await page.route('**/api/v2/missions/validate', async (route) => {
    validateCalls += 1;
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

  await page.route('**/api/v2/missions', async (route) => {
    const method = route.request().method();
    if (method === 'GET') {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify([]),
      });
      return;
    }
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        mission_id: 'M-v4-kpi',
        version: 1,
        saved_at: new Date().toISOString(),
        filename: 'M-v4-kpi.json',
      }),
    });
  });

  const startMs = Date.now();

  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await dismissIntroIfVisible(page);

  await page.getByRole('button', { name: /Segments/ }).first().click();
  await page.getByRole('button', { name: /^Transfer$/ }).click();

  await page.getByRole('button', { name: /Validate/ }).first().click();
  await page.getByRole('button', { name: 'Re-run' }).click();

  await page.getByRole('button', { name: /Save.*Launch/ }).click();
  const saveLaunchPanel = page.locator('#coachmark-save_launch');
  const saveButton = saveLaunchPanel.getByRole('button', { name: /^Save$/ });
  await expect(saveButton).toBeEnabled();

  await expect.poll(() => validateCalls).toBeGreaterThan(0);

  const elapsedMs = Date.now() - startMs;
  expect(elapsedMs).toBeLessThan(300_000);
});

test('v4 desktop layouts are stable at 1280/1440/1920', async ({ page }) => {
  const sizes = [
    { width: 1280, height: 800 },
    { width: 1440, height: 900 },
    { width: 1920, height: 1080 },
  ] as const;

  for (const size of sizes) {
    await page.setViewportSize(size);
    await page.goto('/');
    await page.getByRole('button', { name: 'PLANNER' }).click();
    await dismissIntroIfVisible(page);

    await expect(page.getByText('Mission Planner')).toBeVisible();
    await expect(page.getByText('Step 1 · Target')).toBeVisible();
    await expect(page.getByText('Diagnostics Timeline')).toBeVisible();

    const hasHorizontalOverflow = await page.evaluate(() => {
      const root = document.documentElement;
      return root.scrollWidth > root.clientWidth + 1;
    });
    expect(hasHorizontalOverflow).toBe(false);
  }
});

test('v4 keyboard flow: focus-visible + step navigation without trap', async ({ page }) => {
  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await dismissIntroIfVisible(page);

  const segmentsStepButton = page.locator('#coachmark-step_rail').getByRole('button', { name: /Segments/ }).first();
  await segmentsStepButton.focus();
  await page.keyboard.press('Enter');
  await expect(page.getByText('Segment Composer')).toBeVisible();

  let foundFocusVisible = false;
  for (let i = 0; i < 25; i += 1) {
    await page.keyboard.press('Tab');
    const focused = await page.evaluate(() => {
      const active = document.activeElement as HTMLElement | null;
      if (!active) return null;
      return {
        className: active.className,
        boxShadow: getComputedStyle(active).boxShadow,
      };
    });

    if (focused && typeof focused.className === 'string' && focused.className.includes('v4-focus')) {
      expect(focused.boxShadow).not.toBe('none');
      foundFocusVisible = true;
      break;
    }
  }

  expect(foundFocusVisible).toBe(true);

  await page.keyboard.press('Alt+5');
  await expect(page.getByText('Step 5 · Validate')).toBeVisible();
});
