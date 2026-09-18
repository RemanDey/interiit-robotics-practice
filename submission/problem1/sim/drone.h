#ifndef DRONE_H
#define DRONE_H

#include "../common/types.h"
#include "../common/id.h"

class Drone {
  private:
    int id;
    Position pos;
    int battery;
    STATES state;
    Location destination;
    Location base;
    double speed;
  public:
    Drone()
    : id(generateUID()),
      pos{0.0, 0.0, 0.0},
      battery(100),
      state(OFF) {}
    Drone(const Location& base, const Location& destination, double latOffset, double lngOffset)
    : id(generateUID()),
      pos{base.lat + latOffset, base.lng + lngOffset, 0.0},
      battery(100),
      state(OFF),
      destination(destination),
      base(base),
      speed(0.0002) {}

    int getId() const { return id; }
    Position getPosition() const { return pos; }
    int getBattery() const { return battery; }
    STATES getState() const { return state; }
    Location getDestination() const { return destination; };
    Location getBase() const { return base; };
    double getSpeed() const { return speed; };

    void setSpeed(double newSpeed) {
      speed = newSpeed;
    }

    void setBase(const Location& newBase) {
      base = newBase;
    }
    void setDestination(const Location& newDestination) {
      destination = newDestination;
    }

    void movePos(double dLat, double dLng, double dAlt) {
      pos.lat += dLat;
      pos.lng += dLng;
      pos.alt += dAlt;
    }

    void drainBattery(int amount) {
      battery-=amount;
      if (battery < 0) battery = 0;
    };

    void setState(STATES newState) {state = newState;};

    void setPosition(double lat, double lng, double alt) {
      pos.lat = lat;
      pos.lng = lng;
      pos.alt = alt;
    }

    void chargeBattery(int amount) {
      battery += amount;
      if (battery > 100) battery = 100;
    }
};

#endif