#include "FuelMath.h"

namespace fuel {

float clampf(float value, float lo, float hi) {
  if (isNotANumber(value)) return lo;
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

uint16_t toU16(float value) {
  if (isNotANumber(value)) return 0;
  if (value <= 0.0f) return 0;
  // Compare against the limit as a float before narrowing; +inf fails this
  // test and saturates, which a direct cast would not do.
  if (value >= 65535.0f) return 65535;
  return (uint16_t)value;
}

uint32_t toU32(float value) {
  if (isNotANumber(value)) return 0;
  if (value <= 0.0f) return 0;
  if (value >= 4294967295.0f) return 4294967295UL;
  return (uint32_t)value;
}

}  // namespace fuel
