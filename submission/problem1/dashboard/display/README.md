# `dashboard/display/` — React + Leaflet frontend

## Stack

Vite 8 + React 19 + `react-leaflet@5` + `leaflet@1.9.4` + TypeScript 5.9
(see `package.json`).

## Files

| File | Role |
|------|------|
| `src/App.tsx` (367 lines) | Entire dashboard: 2 s polling of `GET /telemetry`, `LOW_BATTERY/ACTIVE/RETURNING/IDLE` bucketing, sidebar metrics + filters + selected-drone panel, charging-pads + queued-requests panels, Leaflet map centered on BASE STATION (`[31.7812939, 76.997502]`, zoom 15) with per-drone markers, tooltips, popups, and dashed route polylines |
| `src/main.tsx` | Entry point (`StrictMode` + `createRoot`) |
| `src/App.css` (398 lines) | Sidebar/topbar/map-shell styling (note: `fleet-overview` / `info-panel` classes used by `App.tsx` have no rules here yet) |
| `src/index.css` | Empty — no global styles |

## Run

```bash
cd dashboard/display
npm install
npm run dev      # → http://localhost:5173
```

Scripts: `dev`, `build` (`tsc -b && vite build`), `preview`, `lint`.
Requires the API on `:8000` (hardcoded fetch URL in `App.tsx:103`).
