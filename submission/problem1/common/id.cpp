#include "id.h"

int generateUID() {
  static int staticId = 00;
  return ++staticId;
}
