import { expect, test } from '@playwright/test';

test('app shell loads', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByText('MISSION CONTROL')).toBeVisible();
  await expect(page.getByRole('button', { name: 'RUNNER' })).toBeVisible();
});
