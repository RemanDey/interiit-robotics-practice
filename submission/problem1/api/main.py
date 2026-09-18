from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from fleet_manager import FleetManager

app = FastAPI()
manager = FleetManager()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class DeliveryRequest(BaseModel):
    package_id: str | None = None
    weight: float = 0.0
    destination: dict | None = None
    deadline_minutes: float = 15.0


@app.get("/")
def root():
    return {"message": "Fleet Telemetry System API"}


@app.get("/telemetry")
def get_telemetry():
    payload = manager.get_status()
    return {
        "drones": payload["drones"],
        "charging_pads": payload["charging_pads"],
        "queued_requests": payload["queued_requests"],
        "metrics": payload["metrics"],
        "alerts": payload["alerts"],
    }


@app.get("/status")
def get_status():
    return manager.get_status()


@app.post("/assign-request")
def assign_request(request: DeliveryRequest):
    decision = manager.assign_request(request.model_dump())
    return decision

