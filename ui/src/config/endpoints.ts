const DEFAULT_HOST = window.location.hostname || 'localhost';
const DEFAULT_PORT = '8000';

const rawApiBase =
  import.meta.env.VITE_API_URL ??
  `${window.location.protocol}//${DEFAULT_HOST}:${DEFAULT_PORT}`;

export const API_BASE_URL = rawApiBase.endsWith('/')
  ? rawApiBase.slice(0, -1)
  : rawApiBase;

export const WS_URL =
  import.meta.env.VITE_WS_URL ??
  `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${DEFAULT_HOST}:${DEFAULT_PORT}/ws`;
