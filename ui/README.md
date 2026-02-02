# Mission Control UI

React + TypeScript + Vite frontend for the Mission Control interface.

## Prerequisites

- Node.js 18+
- Backend running on `http://localhost:8000` (see `run_dashboard.py`)

## Configure API Endpoints (optional)

The UI defaults to `http://localhost:8000` and `ws://localhost:8000/ws`.
Override with:

```
VITE_API_URL=http://localhost:8000
VITE_WS_URL=ws://localhost:8000/ws
```

## Development

```
npm install
npm run dev
```

## Production build

```
npm run build
npm run preview
```
