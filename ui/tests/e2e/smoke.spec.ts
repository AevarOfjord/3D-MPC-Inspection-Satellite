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
  await expect(page.getByText('ORBITAL INSPECTOR')).toBeVisible();
  await expect(page.getByRole('button', { name: 'STUDIO' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'RUNNER' })).toBeVisible();
});

test('Studio shell renders its authoring panels from a clean launch', async ({
  page,
}) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });

  await page.goto('/');
  await chooseEmptyScene(page);
  await expect(page.getByText('Studio Status')).toBeVisible();
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
  await page.getByPlaceholder('Search commands...').fill('switch to mission studio');
  await page.keyboard.press('Enter');
  await expect(page.getByRole('button', { name: 'STUDIO' })).toHaveClass(
    /bg-fuchsia/
  );

  await page.keyboard.press('Escape');
  await page.keyboard.press('?');
  await expect(page.getByText('Keyboard Shortcuts')).toBeVisible();
});
