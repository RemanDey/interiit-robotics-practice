import { useState } from "react";
import { Link } from "react-router-dom";
import { MapContainer, TileLayer, Marker, useMapEvents } from "react-leaflet";
import L from "leaflet";

type Decision = {
  accepted: boolean;
  selected_drone: number | null;
  reason: string;
  audit: Array<{
    drone_id: number;
    eligible: boolean;
    score: number;
    reason?: string;
    battery?: number;
    state?: string;
  }>;
};

const pickedIcon = L.divIcon({
  className: "dest-div-icon",
  html: `<div class="dest-badge"><span>★</span></div>`,
  iconSize: [30, 28],
  iconAnchor: [15, 14],
});

function ClickPicker({
  onPick,
}: {
  onPick: (lat: number, lng: number) => void;
}) {
  useMapEvents({
    click(e) {
      onPick(e.latlng.lat, e.latlng.lng);
    },
  });
  return null;
}

export default function RequestPage() {
  const [packageId, setPackageId] = useState("PKG-1");
  const [weight, setWeight] = useState("0.8");
  const [lat, setLat] = useState("31.7905");
  const [lng, setLng] = useState("77.0098");
  const [address, setAddress] = useState("Drop zone");
  const [deadline, setDeadline] = useState("20");
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState("");
  const [decision, setDecision] = useState<Decision | null>(null);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setDecision(null);

    const w = Number(weight);
    const d = Number(deadline);
    const la = Number(lat);
    const ln = Number(lng);
    if (!packageId.trim()) {
      setError("Package ID is required.");
      return;
    }
    if (!Number.isFinite(w) || w <= 0 || w > 2.5) {
      setError("Weight must be > 0 and ≤ 2.5 kg.");
      return;
    }
    if (!Number.isFinite(la) || !Number.isFinite(ln)) {
      setError("Lat/lng must be valid numbers (or click the map).");
      return;
    }
    if (!Number.isFinite(d) || d <= 0) {
      setError("Deadline must be > 0 minutes.");
      return;
    }

    setSubmitting(true);
    try {
      const res = await fetch("http://127.0.0.1:8000/assign-request", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          package_id: packageId.trim(),
          weight: w,
          destination: { lat: la, lng: ln, address: address.trim() || "Drop zone" },
          deadline_minutes: d,
        }),
      });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = (await res.json()) as Decision;
      setDecision(data);
    } catch (err) {
      setError(
        err instanceof Error ? `Request failed: ${err.message}` : "Request failed."
      );
    } finally {
      setSubmitting(false);
    }
  };

  const pickLat = Number(lat);
  const pickLng = Number(lng);
  const hasPick = Number.isFinite(pickLat) && Number.isFinite(pickLng);

  return (
    <div className="dashboard">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark" />
          <div>
            <h1>Drone Fleet Management</h1>
            <p>Autonomous Drone Control</p>
          </div>
        </div>
        <div className="fleet-groups">
          <Link to="/" className="nav-link">
            <span>← Back to Map</span>
          </Link>
        </div>
        <div className="selected-panel">
          <p>How it works</p>
          <div className="destination">
            <p>
              Fill the form or click the picker map. POSTs to{" "}
              <code>POST /assign-request</code>. Weight must be ≤ 2.5 kg.
            </p>
          </div>
        </div>
      </aside>

      <main className="main">
        <header className="topbar">
          <div>
            <span>Request delivery</span>
            <strong>New job</strong>
          </div>
        </header>

        <section className="request-layout">
          <form className="request-form" onSubmit={submit}>
            <h3>Delivery request</h3>
            <label>
              Package ID
              <input
                value={packageId}
                onChange={(e) => setPackageId(e.target.value)}
                placeholder="PKG-1"
              />
            </label>
            <label>
              Weight (kg, max 2.5)
              <input
                value={weight}
                onChange={(e) => setWeight(e.target.value)}
                inputMode="decimal"
                placeholder="0.8"
              />
            </label>
            <div className="request-row">
              <label>
                Lat
                <input value={lat} onChange={(e) => setLat(e.target.value)} inputMode="decimal" />
              </label>
              <label>
                Lng
                <input value={lng} onChange={(e) => setLng(e.target.value)} inputMode="decimal" />
              </label>
            </div>
            <label>
              Address label
              <input value={address} onChange={(e) => setAddress(e.target.value)} placeholder="Drop zone" />
            </label>
            <label>
              Deadline (minutes)
              <input
                value={deadline}
                onChange={(e) => setDeadline(e.target.value)}
                inputMode="numeric"
                placeholder="20"
              />
            </label>
            <button type="submit" disabled={submitting}>
              {submitting ? "Sending…" : "Request delivery"}
            </button>
            {error && <div className="api-error">{error}</div>}
            {decision && (
              <div className={decision.accepted ? "request-result accepted" : "request-result rejected"}>
                <strong>{decision.accepted ? "Accepted" : "Not assigned"}</strong>
                <p>{decision.reason}</p>
                {decision.selected_drone != null && (
                  <p>Drone #{decision.selected_drone} selected.</p>
                )}
                {decision.audit.length > 0 && (
                  <ul>
                    {decision.audit.map((a) => (
                      <li key={a.drone_id}>
                        Drone {a.drone_id}: {a.eligible ? `score ${a.score}` : a.reason ?? "ineligible"}
                      </li>
                    ))}
                  </ul>
                )}
              </div>
            )}
          </form>

          <div className="picker-shell">
            <div className="figure-title">
              <span className="figure-dot" />
              <span>Click map to set destination</span>
            </div>
            <MapContainer center={[31.7812939, 76.997502]} zoom={15} className="map picker-map">
              <TileLayer
                attribution="&copy; OpenStreetMap contributors"
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
              />
              <ClickPicker
                onPick={(la, ln) => {
                  setLat(la.toFixed(6));
                  setLng(ln.toFixed(6));
                }}
              />
              {hasPick && <Marker position={[pickLat, pickLng]} icon={pickedIcon} />}
            </MapContainer>
            <p className="picker-hint">
              Picked: {hasPick ? `${pickLat.toFixed(6)}, ${pickLng.toFixed(6)}` : "none"} — you can
              still edit the fields manually.
            </p>
          </div>
        </section>
      </main>
    </div>
  );
}
