#ifndef TYPES_H
#define TYPES_H

#include <string>

enum STATES {
  OFF,
  START,
  TAKEOFF,
  CRUISE,
  APPROACH,
  DELIVERY,
  RETURNING,
  LANDED,
  CHARGING
};

struct Position {
  double lat;
  double lng;
  double alt;
};

struct Location {
  double lat;
  double lng;
  std::string addr;
};

#endif