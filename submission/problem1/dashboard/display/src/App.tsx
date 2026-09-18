import "./App.css";
import "leaflet/dist/leaflet.css";

import { useEffect, useMemo, useState } from "react";
import {
  MapContainer,
  TileLayer,
  CircleMarker,
  Popup,
  Polyline,
  Tooltip,
  useMap,
} from "react-leaflet";

type Location = {
  lat: number;
  lng: number;
  address: string;
};

type Drone = {
  id: number;
  position: {
    lng: number;
    lat: number;
    alt: number;
  };
  battery: number;
  state: string;
  base: Location;
  destination: Location;
  timestamp: number;
};

type Pad = {
  id: number;
  occupied_by: number | null;
  time_remaining: number;
  queue: number[];
};

const LOW_BATTERY_THRESHOLD = 25;

// MATLAB default axes ColorOrder (R2019b+): blue, orange, yellow, purple,
// green, light-blue, red. Markers use the same palette as plot() lines.
const stateColor = (drone: Drone) => {
  if (drone.battery <= LOW_BATTERY_THRESHOLD) return "#A2142F";

  switch (drone.state) {
    case "CRUISE":
    case "TAKEOFF":
    case "APPROACH":
    case "DELIVERY":
      return "#0072BD";
    case "RETURNING":
      return "#D95319";
    case "CHARGING":
    case "LANDED":
    case "OFF":
      return "#7F7F7F";
    default:
      return "#000000";
  }
};

const normalizeState = (drone: Drone) => {
  if (drone.battery <= LOW_BATTERY_THRESHOLD) return "LOW_BATTERY";

  if (["CRUISE", "TAKEOFF", "APPROACH", "DELIVERY"].includes(drone.state)) {
    return "ACTIVE";
  }

  if (drone.state === "RETURNING") return "RETURNING";

  return "IDLE";
};

function FocusDrone({ drone }: { drone: Drone | null }) {
  const map = useMap();

  useEffect(() => {
    if (!drone) return;

    map.flyTo([drone.position.lat, drone.position.lng], 15, {
      duration: 0.8,
    });
  }, [drone, map]);

  return null;
}

export default function App() {
  const [drones, setDrones] = useState<Drone[]>([]);
  const [chargingPads, setChargingPads] = useState<Pad[]>([]);
  const [queuedRequests, setQueuedRequests] = useState<Array<{ package_id: string; status: string }>>([]);
  const [selectedDrone, setSelectedDrone] = useState<Drone | null>(null);
  const [filter, setFilter] = useState<string>("ALL");
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState("");

  useEffect(() => {
    const fetchData = () => {
      fetch("http://127.0.0.1:8000/telemetry")
        .then((response) => {
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          return response.json();
        })
        .then((data) => {
          setDrones(data.drones ?? []);
          setChargingPads(data.charging_pads ?? []);
          setQueuedRequests(data.queued_requests ?? []);
          setLoading(false);
          setError("");
        })
        .catch((err) => {
          console.error("Failed to fetch telemetry:", err);
          setError("Telemetry API offline");
          setLoading(false);
        });
    };

    fetchData();
    const interval = setInterval(fetchData, 2000);

    return () => clearInterval(interval);
  }, []);

  const metrics = useMemo(() => {
    const total = drones.length;
    const active = drones.filter((d) => normalizeState(d) === "ACTIVE").length;
    const returning = drones.filter((d) => normalizeState(d) === "RETURNING").length;
    const lowBattery = drones.filter((d) => normalizeState(d) === "LOW_BATTERY").length;
    const idle = drones.filter((d) => normalizeState(d) === "IDLE").length;
    const avgBattery =
      total === 0
        ? 0
        : Math.round(drones.reduce((sum, d) => sum + d.battery, 0) / total);

    return { total, active, returning, lowBattery, idle, avgBattery };
  }, [drones]);

  const visibleDrones = useMemo(() => {
    if (filter === "ALL") return drones;
    return drones.filter((drone) => normalizeState(drone) === filter);
  }, [drones, filter]);

  if (loading) {
    return <div className="loading">Loading fleet telemetry...</div>;
  }

  return (
    <div className="dashboard">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark" />
          <div>
            <h1>FleetOps</h1>
            <p>Autonomous Drone Control</p>
          </div>
        </div>

        <div className="status-card critical">
          <span>Critical Alerts</span>
          <strong>{metrics.lowBattery}</strong>
          <p>Low battery drones require attention</p>
        </div>

        <div className="fleet-groups">
          <button onClick={() => setFilter("ALL")} className={filter === "ALL" ? "active" : ""}>
            <span>All Drones</span>
            <strong>{metrics.total}</strong>
          </button>

          <button onClick={() => setFilter("ACTIVE")} className={filter === "ACTIVE" ? "active" : ""}>
            <span>Active</span>
            <strong>{metrics.active}</strong>
          </button>

          <button onClick={() => setFilter("RETURNING")} className={filter === "RETURNING" ? "active" : ""}>
            <span>Returning</span>
            <strong>{metrics.returning}</strong>
          </button>

          <button onClick={() => setFilter("LOW_BATTERY")} className={filter === "LOW_BATTERY" ? "active" : ""}>
            <span>Low Battery</span>
            <strong>{metrics.lowBattery}</strong>
          </button>

          <button onClick={() => setFilter("IDLE")} className={filter === "IDLE" ? "active" : ""}>
            <span>Idle</span>
            <strong>{metrics.idle}</strong>
          </button>
        </div>

        {selectedDrone && (
          <div className="selected-panel">
            <p>Selected Drone</p>
            <h2>Drone #{selectedDrone.id}</h2>

            <div className="detail-row">
              <span>Status</span>
              <strong>{selectedDrone.state}</strong>
            </div>

            <div className="detail-row">
              <span>Battery</span>
              <strong>{selectedDrone.battery}%</strong>
            </div>

            <div className="detail-row">
              <span>Altitude</span>
              <strong>{Math.round(selectedDrone.position.alt)} ft</strong>
            </div>

            <div className="destination">
              <span>Destination</span>
              <p>{selectedDrone.destination.address}</p>
            </div>
          </div>
        )}
      </aside>

      <main className="main">
        <header className="topbar">
          <div>
            <span>Total Drones</span>
            <strong>{metrics.total}</strong>
          </div>

          <div>
            <span>Active</span>
            <strong>{metrics.active}</strong>
          </div>

          <div>
            <span>Avg Battery</span>
            <strong>{metrics.avgBattery}%</strong>
          </div>

          <div className={metrics.lowBattery > 0 ? "metric-alert" : ""}>
            <span>Alerts</span>
            <strong>{metrics.lowBattery}</strong>
          </div>
        </header>

        {metrics.lowBattery > 0 && (
          <div className="alert-banner">
            Critical: {metrics.lowBattery} drone{metrics.lowBattery === 1 ? "" : "s"} below{" "}
            {LOW_BATTERY_THRESHOLD}% battery.
          </div>
        )}

        {error && <div className="api-error">{error}</div>}

        <section className="fleet-overview">
          <div className="info-panel">
            <h3>Charging Pads</h3>
            {chargingPads.length === 0 ? (
              <p>No pad telemetry available.</p>
            ) : (
              <ul>
                {chargingPads.map((pad) => (
                  <li key={pad.id}>
                    Pad {pad.id}: {pad.occupied_by ? `Drone ${pad.occupied_by}` : "available"}
                    {pad.time_remaining > 0 ? ` • ${pad.time_remaining} min` : ""}
                  </li>
                ))}
              </ul>
            )}
          </div>
          <div className="info-panel">
            <h3>Queued Requests</h3>
            {queuedRequests.length === 0 ? (
              <p>No requests in queue.</p>
            ) : (
              <ul>
                {queuedRequests.map((request, index) => (
                  <li key={`${request.package_id ?? index}`}>
                    {request.package_id ?? `Request ${index + 1}`}: {request.status}
                  </li>
                ))}
              </ul>
            )}
          </div>
        </section>

        <section className="map-shell">
          <div className="figure-title">
            <span className="figure-dot" />
            <span>Figure 1: fleet_map — lat vs lon ({drones.length} drones)</span>
            <span className="figure-tools">−&nbsp;&nbsp;□&nbsp;&nbsp;×</span>
          </div>
          <MapContainer
            center={[31.7812939, 76.997502]}
            zoom={15}
            className="map"
          >
            <FocusDrone drone={selectedDrone} />

            <TileLayer
              attribution="&copy; OpenStreetMap contributors"
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />

            {visibleDrones.map((drone) => {
              const path: [number, number][] = [
                [drone.base.lat, drone.base.lng],
                [drone.position.lat, drone.position.lng],
                [drone.destination.lat, drone.destination.lng],
              ];

              const selected = selectedDrone?.id === drone.id;

              return (
                <div key={drone.id}>
                  {selected && (
                    <Polyline
                      positions={path}
                      color={stateColor(drone)}
                      weight={4}
                      opacity={0.9}
                      dashArray="8 8"
                    />
                  )}

                  <CircleMarker
                    center={[drone.position.lat, drone.position.lng]}
                    radius={selected ? 9 : 6}
                    color={stateColor(drone)}
                    fillColor={stateColor(drone)}
                    fillOpacity={0.9}
                    weight={selected ? 3 : 1}
                    eventHandlers={{
                      click: () => setSelectedDrone(drone),
                    }}
                  >
                    <Tooltip direction="top" offset={[0, -8]} opacity={1}>
                      <strong>Drone #{drone.id}</strong>
                      <br />
                      Battery: {drone.battery}%
                      <br />
                      Status: {drone.state}
                      <br />
                      Destination: {drone.destination.address}
                    </Tooltip>

                    <Popup>
                      <strong>Drone #{drone.id}</strong>
                      <br />
                      Battery: {drone.battery}%
                      <br />
                      State: {drone.state}
                      <br />
                      Base: {drone.base.address}
                      <br />
                      Destination: {drone.destination.address}
                    </Popup>
                  </CircleMarker>
                </div>
              );
            })}
          </MapContainer>
        </section>
      </main>
    </div>
  );
}