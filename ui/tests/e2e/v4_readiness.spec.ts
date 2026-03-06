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

test('Studio flow: disconnected authoring fails locally before backend validation', async ({
  page,
}) => {
  let validateCalls = 0;

  await page.addInitScript(() => {
    window.localStorage.clear();
  });

  await page.on('dialog', async (dialog) => {
    if (dialog.type() === 'prompt') {
      await dialog.accept('0.5');
      return;
    }
    await dialog.dismiss();
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
  await page.route('**/api/models/generate_scan_path', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        waypoints: [
          [0, 0, -5],
          [0, 0, 5],
          [0, 0, 10],
        ],
      }),
    });
  });

  const startMs = Date.now();

  await page.goto('/');
  await chooseEmptyScene(page);
  await page.getByRole('button', { name: 'Place Satellite' }).click();
  await page.getByRole('button', { name: 'Create Path' }).click();
  const scanPathResponse = page.waitForResponse(
    (response) =>
      response.url().includes('/api/models/generate_scan_path') &&
      response.request().method() === 'POST'
  );
  await page.getByRole('button', { name: 'Add Path' }).click();
  await scanPathResponse;
  await expect(
    page.getByText('Connect satellite:start to the first path or point.').first()
  ).toBeVisible();

  await page.getByPlaceholder('Mission name...').fill('Studio E2E Mission');
  await expect(page.getByRole('button', { name: 'Validate' })).toBeDisabled();
  await expect.poll(() => validateCalls).toBe(0);
  await expect(
    page.getByText('Connect satellite:start to the first path or point.').first()
  ).toBeVisible();

  const elapsedMs = Date.now() - startMs;
  expect(elapsedMs).toBeLessThan(300_000);
});

test('Studio desktop layouts stay stable at 1280/1440/1920', async ({ page }) => {
  const sizes = [
    { width: 1280, height: 800 },
    { width: 1440, height: 900 },
    { width: 1920, height: 1080 },
  ] as const;

  for (const size of sizes) {
    await page.addInitScript(() => {
      window.localStorage.clear();
    });
    await page.setViewportSize(size);
    await page.goto('/');
    await chooseEmptyScene(page);

    await expect(page.getByText('Studio Status')).toBeVisible();
    await expect(page.getByRole('button', { name: 'Create Path' })).toBeVisible();
    await expect(page.getByRole('button', { name: 'Save Mission' })).toBeVisible();

    const hasHorizontalOverflow = await page.evaluate(() => {
      const root = document.documentElement;
      return root.scrollWidth > root.clientWidth + 1;
    });
    expect(hasHorizontalOverflow).toBe(false);
  }
});

test('Studio keyboard flow keeps focus navigation usable', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.clear();
  });
  await page.goto('/');
  await chooseEmptyScene(page);

  const createPathButton = page.getByRole('button', { name: 'Create Path' });
  await createPathButton.focus();
  await page.keyboard.press('Enter');
  await expect(page.getByRole('button', { name: 'Add Path' })).toBeVisible();

  let foundFocusableAction = false;
  for (let i = 0; i < 25; i += 1) {
    await page.keyboard.press('Tab');
    const focused = await page.evaluate(() => {
      const active = document.activeElement as HTMLElement | null;
      if (!active) return null;
      return {
        className: active.className,
        text: active.textContent,
      };
    });

    if (
      focused &&
      typeof focused.className === 'string' &&
      focused.className.includes('border')
    ) {
      foundFocusableAction = true;
      break;
    }
  }

  expect(foundFocusableAction).toBe(true);
});
