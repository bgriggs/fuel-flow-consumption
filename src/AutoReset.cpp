#include "AutoReset.h"

namespace fuel {

AutoReset::AutoReset()
  : dwelling_(false),
    dwellStartedMs_(0),
    dwellMs_(DEFAULT_DWELL_MS),
    minUsedGals_((float)DEFAULT_MIN_USED_GALS),
    maxSpeedMph_((float)DEFAULT_MAX_SPEED_MPH) {}

void AutoReset::rearm() {
  dwelling_ = false;
  dwellStartedMs_ = 0;
}

bool AutoReset::update(uint32_t nowMs, bool fuelFull, float fuelUsedGals,
                       float speedMph) {
  // Every condition has to hold continuously for the whole dwell, not merely
  // at the moment it expires. Checking speed only at expiry meant a tank-full
  // line stuck asserted - a lost sender ground is enough - plus a single
  // momentary dip below the speed limit at pit entry, on the grid or during a
  // spin would zero the counter mid-session with no refuel.
  bool conditionsHold =
      fuelFull &&
      !isNotANumber(fuelUsedGals) && fuelUsedGals > minUsedGals_ &&
      !isNotANumber(speedMph) && speedMph < maxSpeedMph_;

  if (!conditionsHold) {
    rearm();
    return false;
  }

  // Rising edge: start the clock. Tracked with an explicit flag rather than a
  // zero timestamp sentinel, because millis() legitimately returns 0 for the
  // first millisecond after boot and after every rollover.
  if (!dwelling_) {
    dwelling_ = true;
    dwellStartedMs_ = nowMs;
  }

  if (elapsed(nowMs, dwellStartedMs_) < dwellMs_) return false;

  // Conditions met throughout. Re-arm so a tank that simply stays full does
  // not retrigger once the caller has zeroed the counter.
  rearm();
  return true;
}

}  // namespace fuel
