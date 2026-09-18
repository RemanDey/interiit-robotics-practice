#ifndef BACKEND_H
#define BACKEND_H

#include <chrono>
#include <cstddef>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <string>

#include "../common/types.h"
#include "../sim/drone.h"

inline const char* stateToString(STATES state) {
  switch (state) {
    case OFF: return "OFF";
    case START: return "START";
    case TAKEOFF: return "TAKEOFF";
    case CRUISE: return "CRUISE";
    case APPROACH: return "APPROACH";
    case DELIVERY: return "DELIVERY";
    case RETURNING: return "RETURNING";
    case LANDED: return "LANDED";
    case CHARGING: return "CHARGING";
    default: return "UNKNOWN";
  }
}

class DroneState {
 private:
  int id;
  Position pos;
  int battery;
  STATES state;
  Location destination;
  Location base;
  double speed;
  std::chrono::system_clock::time_point last_updated;

 public:
  DroneState(const Drone& d)
      : id(d.getId()),
        pos(d.getPosition()),
        battery(d.getBattery()),
        state(d.getState()),
        destination(d.getDestination()),
        base(d.getBase()),
        speed(d.getSpeed()),
        last_updated(std::chrono::system_clock::now()) {}

  DroneState(int id, Position pos, int battery, STATES state, Location base, Location destination, double speed)
      : id(id),
        pos(pos),
        battery(battery),
        state(state),
        destination(destination),
        base(base),
        speed(speed),
        last_updated(std::chrono::system_clock::now()) {}

  int getId() const { return id; };
  Position getPosition() const { return pos; };
  int getBattery() const { return battery; };
  STATES getState() const { return state; };
  Location getDestination() const { return destination; };
  Location getBase() const { return base; };
  double getSpeed() const { return speed; };

  long long getLastUpdated() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        last_updated.time_since_epoch()
    ).count();
  }
};

class DroneList {
 private:
  std::unordered_map<int, DroneState> drones;

 public:
  DroneList() {};

  size_t size() const { return drones.size(); };

  void addDrone(const Drone& d) {
    DroneState dc(d);
    auto res = drones.emplace(dc.getId(), dc);
    if (!res.second)
      std::cout << "Line 47 in /backend.h, addDrone(): Drone already exists\n";
  }

  int update(const DroneState& state) {
    auto ditr = drones.find(state.getId());
    if (ditr == drones.end()) return -1;
    ditr->second = state;
    return 0;
  }

  const DroneState* getDroneState(int id) const {
    auto ditr = drones.find(id);
    if (ditr == drones.end()) return nullptr;
    return &ditr->second;
  }

  int writeTelemetry(const std::string& filename = "telemetry.json") const {
    std::ofstream file(filename);
    if (!file.is_open()) return -1;

    file << "{\n";
    file << "  \"drones\": [\n";

    bool first = true;

    for (const auto& pair : drones) {
      const DroneState& drone = pair.second;
      Position p = drone.getPosition();

      if (!first) {
        file << ",\n";
      }

      file << "    {\n";
      file << "      \"id\": " << drone.getId() << ",\n";
      file << "      \"position\": {\n";
      file << "        \"lat\": " << p.lat << ",\n";
      file << "        \"lng\": " << p.lng << ",\n";
      file << "        \"alt\": " << p.alt << "\n";
      file << "      },\n";
      file << "      \"battery\": " << drone.getBattery() << ",\n";
      file << "      \"state\": \"" << stateToString(drone.getState()) << "\",\n";
      file << "      \"timestamp\": " << drone.getLastUpdated() << ",\n";
      file << "      \"base\": {\n";
      file << "        \"lat\": " << drone.getBase().lat << ",\n";
      file << "        \"lng\": " << drone.getBase().lng << ",\n";
      file << "        \"address\": \"" << drone.getBase().addr << "\"\n";
      file << "      },\n";
      file << "      \"destination\": {\n";
      file << "        \"lat\": " << drone.getDestination().lat << ",\n";
      file << "        \"lng\": " << drone.getDestination().lng << ",\n";
      file << "        \"address\": \"" << drone.getDestination().addr << "\"\n";
      file << "      },\n";
      file << "      \"speed\": " << drone.getSpeed() << "\n";
      file << "    }";

      first = false;
    }

    file << "\n  ]\n";
    file << "}\n";

    file.close();
    return 0;
  }
};

#endif