// AutoReset.h - Decides when a refuel has happened and the fuel-used counter
// should go back to zero.
//
// The tank-full line is a level indication, not a refuel event: it reads
// asserted across a broad range of tank contents, so it is high for most of a
// session. Treating "high" as "just refueled" meant any stop of more than the
// dwell, once enough fuel had been burned, zeroed the counter - pit entry, the
// grid, a full course yellow.
//
// So a refuel additionally requires the signal to have been held LOW for a
// sustained period since the last reset. A tank that has been reading full all
// session never produces that transition and can never trigger; a tank run
// down and then filled does. A line stuck permanently high is covered by the
// same rule.
//
// The low period is debounced exactly like the dwell, and for the same reason.
// A float switch that dips low for a single sample under braking is no more
// trustworthy than one that dips high, and an undebounced latch would let one
// such sample in an entire session re-create the very fault this prevents. A
// genuine run-down holds the line low for minutes, so nothing real is lost.
//
// On top of that, all three conditions - tank full, a plausible amount already
// burned, and the car stationary - must hold continuously for the whole dwell.
// Any of them lapsing restarts the clock.
#ifndef AUTO_RESET_H
#define AUTO_RESET_H

#include <stdint.h>
#include "FuelMath.h"

namespace fuel {

class AutoReset {
public:
  // How long "tank full" must hold continuously before it counts as a refuel.
  static const uint32_t DEFAULT_DWELL_MS = 10000UL;
  // ...and how long it must hold LOW before a later assertion is meaningful.
  static const uint32_t DEFAULT_ARM_DWELL_MS = 10000UL;
  // A refuel only makes sense once a plausible amount has been burned.
  static const uint8_t DEFAULT_MIN_USED_GALS = 4;
  // ...and only while stopped, i.e. in the pits.
  static const uint8_t DEFAULT_MAX_SPEED_MPH = 3;

  AutoReset();

  void setDwellMs(uint32_t ms) { dwellMs_ = ms; }
  void setArmDwellMs(uint32_t ms) { armDwellMs_ = ms; }
  void setMinUsedGals(float gals) { minUsedGals_ = gals; }
  void setMaxSpeedMph(float mph) { maxSpeedMph_ = mph; }

  // Drop any accumulated dwell and any arming still in progress, but keep
  // arming that has already been earned. Used while the detector is not being
  // evaluated at all - auto-reset switched off, or the state frames gone stale
  // - which may last indefinitely.
  //
  // Earned arming survives that, because a real refuel did happen and the
  // remaining gates still apply. A partly accumulated low period does not:
  // resuming against a timestamp from before the gap would let two low samples
  // an arbitrary time apart arm the detector, which is exactly the
  // single-sample latch the debounce exists to prevent.
  void rearm();

  // Drop the dwell and the arming, so a fresh low-to-high transition on the
  // tank-full line is required before anything can fire. Used after the
  // counter has been zeroed by any route.
  void disarm();

  // Advance the state machine. Returns true on the single update where all
  // refuel conditions are satisfied; the caller zeroes the fuel counter and
  // this disarms itself, so it cannot fire again until the tank-full line has
  // been held low and then asserted afresh.
  bool update(uint32_t nowMs, bool fuelFull, float fuelUsedGals, float speedMph);

  // True once the tank-full line has been held low long enough that a later
  // assertion of it is meaningful as a refuel.
  bool isArmed() const { return armed_; }

  // True while the line is low and the arming period is still accumulating.
  bool isArming() const { return arming_; }

  // Exposed for tests and diagnostics.
  bool isDwelling() const { return dwelling_; }
  uint32_t dwellElapsedMs(uint32_t nowMs) const {
    return dwelling_ ? elapsed(nowMs, dwellStartedMs_) : 0;
  }

private:
  // Drops only the dwell. Used inside update(), where the arming state is
  // being maintained deliberately and must not be cleared as a side effect.
  void clearDwell();

  bool armed_;               // has "tank full" been held low since the reset?
  bool arming_;              // is it low right now, accumulating toward that?
  uint32_t armStartedMs_;    // when it went low; only valid if arming_
  bool dwelling_;            // is "tank full" currently held?
  uint32_t dwellStartedMs_;  // when it was first seen; only valid if dwelling_
  uint32_t dwellMs_;
  uint32_t armDwellMs_;
  float minUsedGals_;
  float maxSpeedMph_;
};

}  // namespace fuel

#endif  // AUTO_RESET_H
