const isDev = import.meta.env.DEV;
const DEFAULT_HOST = window.location.hostname || 'localhost';
const DEFAULT_PORT = isDev ? '8000' : window.location.port;
const _portStr = DEFAULT_PORT ? `:${DEFAULT_PORT}` : '';

const DEFAULT_HTTP_BASE = `${window.location.protocol}//${DEFAULT_HOST}${_portStr}`;
const DEFAULT_WS_BASE = `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${DEFAULT_HOST}${_portStr}`;

const normalizeBase = (value: string) => (value.endsWith('/') ? value.slice(0, -1) : value);

const rawApiBase =
  import.meta.env.VITE_API_BASE ??
  import.meta.env.VITE_API_URL ??
  DEFAULT_HTTP_BASE;

const rawWsBase =
  import.meta.env.VITE_WS_BASE ??
  import.meta.env.VITE_WS_URL ??
  DEFAULT_WS_BASE;

export const API_BASE_URL = normalizeBase(rawApiBase);
const WS_BASE_URL = normalizeBase(rawWsBase);

const apiUrl = (path: string) =>
  `${API_BASE_URL}${path.startsWith('/') ? path : `/${path}`}`;

export const wsUrl = (path: string) =>
  `${WS_BASE_URL}${path.startsWith('/') ? path : `/${path}`}`;

export const RUNNER_API_URL = apiUrl('/runner');
export const RUNNER_WS_URL = wsUrl('/runner/ws');
export const MISSIONS_API_URL = apiUrl('/saved_missions_v2');
