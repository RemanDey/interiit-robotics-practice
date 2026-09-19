#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "backend/backend.h"
#include "common/id.h"
#include "data/locations.h"
#include "drone.h"
#include "sim/commands.h"

std::random_device rd;
std::mt19937 gen(rd());

std::uniform_real_distribution<double> offsetDist(-0.0003, 0.0003);
std::uniform_real_distribution<double> speedDist(0.00015, 0.00035);

using std::cout;
using std::endl;
using std::vector;

const int NUM_OF_DRONES = 3;
const double CRUISE_ALTITUDE = 30.0;
const int DELIVERY_WAIT_TICKS = 3;

const int CHARGE_PER_TICK = 4;
const int LOW_BATTERY_THRESHOLD = 20;
const int BATTERY_DRAIN_INTERVAL = 5;
const int MOVE_DRAIN = 2;
const int IDLE_DRAIN = 1;

// On-demand dispatch inbox written by the API (api/fleet_manager.py).
// Drones stay parked at base until an entry addressed to their ID appears.
const std::string COMMANDS_PATH = "backend/commands.json";

double distance2D(double lat1, double lng1, double lat2, double lng2) {
  double dLat = lat2 - lat1;
  double dLng = lng2 - lng1;
  return std::sqrt(dLat * dLat + dLng * dLng);
}

void maybeDrainBattery(Drone& d, int tick, bool isMoving) {
  if (tick % BATTERY_DRAIN_INTERVAL != 0) return;

  if (isMoving) {
    d.drainBattery(MOVE_DRAIN);
  } else {
    d.drainBattery(IDLE_DRAIN);
  }
}

int main() {
  DroneList fleet;
  vector<Drone> droneRegistry;
  vector<int> deliveryTicksRemaining;
  vector<int> chargingTicksRemaining;

  droneRegistry.reserve(NUM_OF_DRONES);
  deliveryTicksRemaining.resize(NUM_OF_DRONES, 0);
  chargingTicksRemaining.resize(NUM_OF_DRONES, 0);

  cout << "Simulation Starting Up..." << endl;
  cout << "Adding drones..." << endl;

  for (int i = 0; i < NUM_OF_DRONES; i++) {
    Location chosenBase = BASES.front();

    double latOffset = offsetDist(gen);
    double lngOffset = offsetDist(gen);

    // Park at base with no mission: drones lift off only when the API drops
    // a command addressed to them into backend/commands.json.
    Drone d(chosenBase, chosenBase, latOffset, lngOffset);
    d.setSpeed(speedDist(gen));
    d.setState(LANDED);

    droneRegistry.push_back(d);
    fleet.addDrone(d);

    cout << "[New Drone Registered] Drone ID: " << d.getId()
         << " | Base: " << chosenBase.addr
         << " | State: LANDED (awaiting delivery request)" << endl;
  }

  cout << "Added drones completed." << endl;
  cout << "Fleet size: " << fleet.size() << endl;
  cout << "\nSimulation Begin\n" << endl;

  int tick = 0;

  while (true) {
    // Poll the on-demand dispatch inbox once per tick (fail-soft: missing
    // or corrupt file simply yields an empty inbox for this tick).
    std::unordered_map<int, Command> inbox;
    for (const Command& c : readCommands(COMMANDS_PATH)) {
      if (inbox.find(c.drone_id) == inbox.end()) inbox[c.drone_id] = c;
    }
    vector<int> consumedIds;

    for (int i = 0; i < NUM_OF_DRONES; i++) {
      auto& d = droneRegistry[i];
      Position p = d.getPosition();
      STATES state = d.getState();

      if (state == OFF) {
        d.setState(LANDED);
      }

      else if (state == TAKEOFF) {
        if (p.alt < CRUISE_ALTITUDE) {
          d.movePos(0.0, 0.0, 5.0);
          maybeDrainBattery(d, tick, false);
        } else {
          d.setState(CRUISE);
        }
      }

      else if (state == CRUISE) {
        if (d.getBattery() <= LOW_BATTERY_THRESHOLD) {
          d.setState(RETURNING);
        } else {
          Location dest = d.getDestination();

          double dLat = dest.lat - p.lat;
          double dLng = dest.lng - p.lng;
          double distance = std::sqrt(dLat * dLat + dLng * dLng);

          if (distance < 0.0001) {
            d.setState(DELIVERY);
            deliveryTicksRemaining[i] = DELIVERY_WAIT_TICKS;
          } else {
            // Clamp to remaining distance: a full-speed step near the
            // target would overshoot it and orbit forever outside the
            // arrival disc above.
            double step = std::min(d.getSpeed(), distance);
            double unitLat = dLat / distance;
            double unitLng = dLng / distance;

            d.movePos(unitLat * step, unitLng * step, 0.0);
            maybeDrainBattery(d, tick, true);
          }
        }
      }

      else if (state == DELIVERY) {
        if (deliveryTicksRemaining[i] > 0) {
          deliveryTicksRemaining[i]--;
          maybeDrainBattery(d, tick, false);
        } else {
          d.setState(RETURNING);
        }
      }

      else if (state == RETURNING) {
        Location base = d.getBase();

        double dLat = base.lat - p.lat;
        double dLng = base.lng - p.lng;
        double distance = std::sqrt(dLat * dLat + dLng * dLng);

        if (distance < 0.0001) {
          if (p.alt > 0.0) {
            double descent = (p.alt >= 5.0) ? -5.0 : -p.alt;
            d.movePos(0.0, 0.0, descent);
            maybeDrainBattery(d, tick, false);
          } else {
            d.setState(LANDED);
          }
        } else {
          // Same anti-overshoot clamp as CRUISE: never step past the base.
          double step = std::min(d.getSpeed(), distance);
          double unitLat = dLat / distance;
          double unitLng = dLng / distance;

          d.movePos(unitLat * step, unitLng * step, 0.0);
          maybeDrainBattery(d, tick, true);
        }
      }

      else if (state == LANDED) {
        auto it = inbox.find(d.getId());
        bool hasCmd = (it != inbox.end());
        if (d.getBattery() < 100 &&
            !(hasCmd && d.getBattery() > LOW_BATTERY_THRESHOLD)) {
          // Opportunistic top-up: park at 100% whenever idle. A waiting
          // command preempts charging only if the battery is healthy
          // enough to survive the trip (golden safety rule).
          d.setState(CHARGING);
          chargingTicksRemaining[i] = 0;
        } else if (hasCmd) {
          const Command& cmd = it->second;
          d.setDestination(Location{cmd.lat, cmd.lng, cmd.address});
          inbox.erase(it);
          consumedIds.push_back(d.getId());
          cout << "[Command] Drone " << d.getId() << " tasked to "
               << cmd.address << " (" << cmd.package_id << ")" << endl;
          d.setState(TAKEOFF);
          // else (no command, battery full): hold at base, no drain.
        }
      }

      else if (state == CHARGING) {
        if (d.getBattery() < 100) {
          d.chargeBattery(CHARGE_PER_TICK);
          chargingTicksRemaining[i]++;
        } else {
          // Full charge => park and wait for the next request.
          // No auto-generated destination, no auto-takeoff.
          d.setState(LANDED);
        }
      }

      fleet.update(DroneState(d));
    }

    if (!consumedIds.empty()) {
      std::vector<Command> remaining;
      for (const auto& kv : inbox) remaining.push_back(kv.second);
      if (!writeCommands(COMMANDS_PATH, remaining))
        cout << "Failed to update command inbox\n";
    }

    int result = fleet.writeTelemetry("backend/telemetry.json");
    if (result != 0) cout << "Failed to write telemetry\n";

    cout << "Tick " << tick++ << " written to telemetry.json" << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  }
}