#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

#include "backend/backend.h"
#include "common/id.h"
#include "data/locations.h"
#include "drone.h"

std::random_device rd;
std::mt19937 gen(rd());

std::uniform_int_distribution<int> destDist(0, DESTINATIONS.size() - 1);
std::uniform_real_distribution<double> offsetDist(-0.0003, 0.0003);
std::uniform_real_distribution<double> speedDist(0.00015, 0.00035);

using std::cout;
using std::endl;
using std::vector;

const int NUM_OF_DRONES = 10;
const double CRUISE_ALTITUDE = 30.0;
const int DELIVERY_WAIT_TICKS = 3;

const int CHARGE_PER_TICK = 4;
const int LOW_BATTERY_THRESHOLD = 20;
const int BATTERY_DRAIN_INTERVAL = 5;
const int MOVE_DRAIN = 2;
const int IDLE_DRAIN = 1;

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
    Location chosenDestination = DESTINATIONS[destDist(gen)];

    double latOffset = offsetDist(gen);
    double lngOffset = offsetDist(gen);

    Drone d(chosenBase, chosenDestination, latOffset, lngOffset);
    d.setSpeed(speedDist(gen));

    droneRegistry.push_back(d);
    fleet.addDrone(d);

    cout << "[New Drone Registered] Drone ID: " << d.getId()
         << " | Base: " << chosenBase.addr
         << " | Destination: " << chosenDestination.addr << endl;
  }

  cout << "Added drones completed." << endl;
  cout << "Fleet size: " << fleet.size() << endl;
  cout << "\nSimulation Begin\n" << endl;

  int tick = 0;

  while (true) {
    for (int i = 0; i < NUM_OF_DRONES; i++) {
      auto& d = droneRegistry[i];
      Position p = d.getPosition();
      STATES state = d.getState();

      if (state == OFF) {
        d.setState(TAKEOFF);
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
            double step = d.getSpeed();
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
          double step = d.getSpeed();
          double unitLat = dLat / distance;
          double unitLng = dLng / distance;

          d.movePos(unitLat * step, unitLng * step, 0.0);
          maybeDrainBattery(d, tick, true);
        }
      }

      else if (state == LANDED) {
        d.setState(CHARGING);
        chargingTicksRemaining[i] = 0;
      }

      else if (state == CHARGING) {
        if (d.getBattery() < 100) {
          d.chargeBattery(CHARGE_PER_TICK);
          chargingTicksRemaining[i]++;
        } else {
          Location newDestination = DESTINATIONS[destDist(gen)];
          d.setDestination(newDestination);

          double latOffset = offsetDist(gen);
          double lngOffset = offsetDist(gen);

          Location base = d.getBase();
          d.setPosition(base.lat + latOffset, base.lng + lngOffset, 0.0);
          d.setState(TAKEOFF);
        }
      }

      fleet.update(DroneState(d));
    }

    int result = fleet.writeTelemetry("backend/telemetry.json");
    if (result != 0) cout << "Failed to write telemetry\n";

    cout << "Tick " << tick++ << " written to telemetry.json" << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}