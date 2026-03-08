import { expect, test, type Page } from '@playwright/test';

async function dismissIntroIfVisible(page: Page) {
  const dismiss = page.getByRole('button', { name: 'Dismiss' });
  const visible = await dismiss
    .waitFor({ state: 'visible', timeout: 1000 })
    .then(() => true)
    .catch(() => false);
  if (visible) {
    await dismiss.click();
  }
}

test('path density panel clamps input range', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });
  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await dismissIntroIfVisible(page);

  const densityInput = page.getByLabel('Path density multiplier');
  await expect(densityInput).toBeVisible();

  await densityInput.fill('50');
  await densityInput.blur();
  await expect(densityInput).toHaveValue('20');

  await densityInput.fill('0.1');
  await densityInput.blur();
  await expect(densityInput).toHaveValue('0.25');
});

test('path density apply keeps selected multiplier', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });
  await page.goto('/');
  await page.getByRole('button', { name: 'PLANNER' }).click();
  await dismissIntroIfVisible(page);

  const densityInput = page.getByLabel('Path density multiplier');
  await expect(densityInput).toBeVisible();

  await densityInput.fill('2.5');
  await page.getByRole('button', { name: 'Apply Density' }).click();
  await expect(densityInput).toHaveValue('2.5');
});
