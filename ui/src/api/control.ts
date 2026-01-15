import { API_BASE_URL } from '../config/endpoints';

const postJson = async (path: string, body?: any) => {
  const response = await fetch(`${API_BASE_URL}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!response.ok) {
    throw new Error(`Request failed: ${path}`);
  }
  return response.json();
};

export const controlApi = {
  pause: async () => postJson('/control', { action: 'pause' }),
  resume: async () => postJson('/control', { action: 'resume' }),
  step: async (steps = 1) => postJson('/control', { action: 'step', steps }),
  setSpeed: async (speed: number) => postJson('/speed', { speed }),
  replan: async () => postJson('/replan'),
  reset: async () => postJson('/reset'),
};
