import { expect, test, type Page } from '@playwright/test';

async function chooseEmptyScene(page: Page) {
  const emptySceneButton = page.getByRole('button', { name: 'Empty Scene' });
  const visible = await emptySceneButton
    .waitFor({ state: 'visible', timeout: 1200 })
    .then(() => true)
    .catch(() => false);
  if (visible) {
    await emptySceneButton.click();
  }
}

test('app shell loads', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByRole('button', { name: 'VIEWER' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'STUDIO' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'RUNNER' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'VIEWER' })).toHaveClass(/bg-cyan/);
});

test('toolbench tabs switch between viewer, runner, data, and settings', async ({
  page,
}) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });

  await page.goto('/');
  await chooseEmptyScene(page);
  await expect(page.getByRole('button', { name: 'VIEWER' })).toHaveClass(/bg-cyan/);

  await page.getByRole('button', { name: 'RUNNER' }).click();
  await expect(page.getByText('Run Context')).toBeVisible();
  await expect(page.getByRole('button', { name: 'Run Simulation' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'Chase Sat' })).toHaveCount(0);

  await page.getByRole('button', { name: 'DATA' }).click();
  await expect(page.getByText('Saved Runs')).toBeVisible();

  await page.getByRole('button', { name: 'SETTINGS' }).click();
  await expect(page.getByRole('button', { name: 'MPC Settings' })).toBeVisible();
  await page.getByRole('button', { name: 'General Settings' }).click();
  await expect(page.getByText('System Readiness')).toBeVisible();
  await expect(page.getByText('Build & Package')).toBeVisible();
});

test('Studio shell renders its authoring panels from a clean launch', async ({
  page,
}) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });

  await page.goto('/');
  await page.getByRole('button', { name: 'STUDIO' }).click();
  await chooseEmptyScene(page);
  await expect(page.getByText('Status')).toBeVisible();
  await expect(page.getByRole('button', { name: 'Create Path' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'Validate' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'Save Mission' })).toBeDisabled();
});

test('command palette and Studio shortcuts work', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });
  await page.goto('/');
  await page.getByRole('button', { name: /Command Palette/ }).click();
  await expect(page.getByPlaceholder('Search commands...')).toBeVisible();
  await page.getByPlaceholder('Search commands...').fill('mission studio');
  await page.keyboard.press('Enter');
  await expect(page.getByRole('button', { name: 'STUDIO' })).toHaveClass(
    /bg-fuchsia/
  );

  await page.keyboard.press('Escape');
  await page.keyboard.press('?');
  await expect(page.getByText('Keyboard Shortcuts')).toBeVisible();
});
