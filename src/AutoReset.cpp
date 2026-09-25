#include "AutoReset.h"

namespace fuel {

AutoReset::AutoReset()
  : armed_(false),
    arming_(false),
    armStartedMs_(0),
    dwelling_(false),
    dwellStartedMs_(0),
    dwellMs_(DEFAULT_DWELL_MS),
    armDwellMs_(DEFAULT_ARM_DWELL_MS),
    minUsedGals_((float)DEFAULT_MIN_USED_GALS),
    maxSpeedMph_((float)DEFAULT_MAX_SPEED_MPH) {}

void AutoReset::clearDwell() {
  dwelling_ = false;
  dwellStartedMs_ = 0;
}

void AutoReset::rearm() {
  clearDwell();
  // Arming in progress is discarded; arming already earned is not.
  arming_ = false;
  armStartedMs_ = 0;
}

void AutoReset::disarm() {
  armed_ = false;
  rearm();  // also drops any low period in progress
}

bool AutoReset::update(uint32_t nowMs, bool fuelFull, float fuelUsedGals,
                       float speedMph) {
  // Holding the line low is what arms the detector. Until that has happened a
  // later assertion says nothing about a refuel, because the signal reads high
  // over a wide range of tank contents.
  //
  // The low period is debounced like the dwell: a single sample is not enough.
  // One spurious low reading from a float switch would otherwise latch the
  // detector for the rest of the session and wipe the counter at the next
  // stop, which is exactly the fault this exists to prevent.
  if (!fuelFull) {
    if (!arming_) {
      arming_ = true;
      armStartedMs_ = nowMs;
    }
    if (elapsed(nowMs, armStartedMs_) >= armDwellMs_) armed_ = true;
    clearDwell();
    return false;
  }

  // Back high, so stop accumulating. Any arming already earned is kept.
  arming_ = false;

  // High, and it has been high ever since the last reset, so this is a tank
  // that was already full rather than one that has just been filled.
  if (!armed_) {
    rearm();
    return false;
  }

  // Every remaining condition has to hold continuously for the whole dwell,
  // not merely at the moment it expires. Checking speed only at expiry meant
  // one momentary dip below the limit - pit entry, the grid, a spin - was
  // enough to zero the counter.
  // NaN needs no separate guard: every comparison against NaN is false, so a
  // NaN reading fails its gate the same way an out-of-range one does. An
  // explicit isNotANumber() check here would be unreachable, and a test could
  // never tell whether it was present.
  bool conditionsHold = fuelUsedGals > minUsedGals_ &&
                        speedMph < maxSpeedMph_;

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

  // Conditions met throughout. Disarm, so the tank going on reading full after
  // the caller zeroes the counter cannot trigger a second reset: another
  // low-to-high transition is required first.
  disarm();
  return true;
}

}  // namespace fuel
