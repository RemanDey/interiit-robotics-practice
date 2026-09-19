# `dashboard/display/` — React + Leaflet frontend (map + request page)

## 🎬 Demo Video

[![Fleet Telemetry System Demo](https://img.youtube.com/vi/Uo0r4DJiyb4/0.jpg)](https://youtu.be/Uo0r4DJiyb4)

Watch the full demo: https://youtu.be/Uo0r4DJiyb4

---

## Stack

Vite 8 + React 19 + `react-leaflet@5` + `leaflet@1.9.4` +
`react-router-dom@7` + TypeScript 5.9 (see `package.json`).

## Files

| File | Role |
|------|------|
| `src/App.tsx` (611 lines) | `BrowserRouter` with `/` (`Dashboard`) + `/request` (`RequestPage`). Dashboard: 2 s polling of `GET /telemetry`, `LOW_BATTERY/ACTIVE/RETURNING/IDLE` bucketing, sidebar metrics + filters + selected-drone panel, charging-pads + queued-requests panels (with `→ Drone N` / queue position), Leaflet map with BASE rectangle + pad diamonds + numbered drone badges + active/queued delivery triangles + legend |
| `src/pages/RequestPage.tsx` (244 lines) | Delivery-request form (`package_id`, weight ≤ 2.5 kg, lat/lng/address, deadline) + click-to-pick Leaflet map → `POST /assign-request`; renders accepted/selected vs allotted/queue-position outcome plus the full per-drone audit |
| `src/main.tsx` | Entry point (`StrictMode` + `createRoot`) |
| `src/App.css` (719 lines) | Sidebar/topbar/map-shell styling + `drone-badge` / `dest-badge` (purple, active) / `queued-badge` (amber, queued) / `pad-diamond` markers, `map-legend`, `request-layout` / `request-form` / `picker-map`, `fleet-overview` / `info-panel` panels |
| `src/index.css` | Empty — no global styles |

## Run

```bash
cd dashboard/display
npm install
npm run dev      # → http://localhost:5173  (map)
                 # → http://localhost:5173/request  (new delivery)
```

Scripts: `dev`, `build` (`tsc -b && vite build`), `preview`, `lint`.
Requires the API on `:8000` (hardcoded fetch URLs in
`App.tsx` and `RequestPage.tsx:81`) and the simulator running so the
fleet is parked `LANDED` and ready for on-demand tasking.

## Map legend (matches `App.tsx` markers)

- Square outline — base station (`Rectangle` around `BASE_LAT/LNG`)
- Green diamond — pad available; blue diamond — pad occupied (`padIcon`)
- Numbered circle — drone, colored by state: blue active
  (`CRUISE/TAKEOFF/APPROACH/DELIVERY`), orange returning, grey
  idle/charging, red low battery
- Purple triangle + drone # — active delivery destination
- Amber triangle + drone # — queued (allotted) delivery; launches
  automatically when that drone lands (see API auto-promotion).
  Co-located drops are spread on a small ring (~13 m) so stacked
  orders stay clickable (`spreadDeliveries`).
