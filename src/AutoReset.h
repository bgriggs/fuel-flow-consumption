// AutoReset.h - Decides when a refuel has happened and the fuel-used counter
// should go back to zero.
//
// The device is told "tank is full" by a float switch on the CAN bus. That
// switch also reads full transiently when fuel sloshes against it as the car
// comes to a stop, so a bare edge-trigger causes resets mid-session. This
// class requires the signal to be continuously asserted for a dwell period
// before it will call a refuel, and gates on fuel actually having been used
// and the car being stationary.
#ifndef AUTO_RESET_H
#define AUTO_RESET_H

#include <stdint.h>
#include "FuelMath.h"

namespace fuel {

class AutoReset {
public:
  // How long "tank full" must hold continuously before it counts as a refuel.
  static const uint32_t DEFAULT_DWELL_MS = 10000UL;
  // A refuel only makes sense once a plausible amount has been burned.
  static const uint8_t DEFAULT_MIN_USED_GALS = 4;
  // ...and only while stopped, i.e. in the pits.
  static const uint8_t DEFAULT_MAX_SPEED_MPH = 3;

  AutoReset();

  void setDwellMs(uint32_t ms) { dwellMs_ = ms; }
  void setMinUsedGals(float gals) { minUsedGals_ = gals; }
  void setMaxSpeedMph(float mph) { maxSpeedMph_ = mph; }

  // Drop any accumulated dwell. Called after a reset actually happens, and
  // whenever the counter is zeroed by other means, so the next refuel has to
  // earn its dwell from scratch.
  void rearm();

  // Advance the state machine. Returns true on the single update where all
  // refuel conditions are satisfied; the caller zeroes the fuel counter and
  // this re-arms itself, so it will not fire again until conditions are met
  // afresh.
  bool update(uint32_t nowMs, bool fuelFull, float fuelUsedGals, float speedMph);

  // Exposed for tests and diagnostics.
  bool isDwelling() const { return dwelling_; }
  uint32_t dwellElapsedMs(uint32_t nowMs) const {
    return dwelling_ ? elapsed(nowMs, dwellStartedMs_) : 0;
  }

private:
  bool dwelling_;            // is "tank full" currently held?
  uint32_t dwellStartedMs_;  // when it was first seen; only valid if dwelling_
  uint32_t dwellMs_;
  float minUsedGals_;
  float maxSpeedMph_;
};

}  // namespace fuel

#endif  // AUTO_RESET_H
