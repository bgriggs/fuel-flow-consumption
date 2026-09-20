// FaultMonitor.h - Decides when a persistently faulted peripheral should be
// re-initialized.
//
// The MCP2515 latches error conditions: a receive-buffer overflow sets a flag
// that software has to clear, and a bus-off condition takes the controller off
// the wire. Nothing in the old firmware looked at any of this, so a controller
// that got into a bad state stayed there silently until the car was
// power-cycled.
//
// Transient error-passive states on a noisy bus clear by themselves, so this
// only calls for recovery once a fault has been continuously present for a
// good while. Pure logic, so the timing rules are covered by the tests.
#ifndef FAULT_MONITOR_H
#define FAULT_MONITOR_H

#include <stdint.h>
#include "FuelMath.h"

namespace fuel {

class FaultMonitor {
public:
  // How often the peripheral is polled.
  static const uint32_t DEFAULT_CHECK_PERIOD_MS = 5000UL;
  // How long a fault must persist before it is treated as latched rather than
  // transient. Long enough that ordinary bus noise never triggers it.
  static const uint32_t DEFAULT_PERSIST_MS = 30000UL;

  FaultMonitor();

  void setCheckPeriodMs(uint32_t ms) { checkPeriodMs_ = ms; }
  void setPersistMs(uint32_t ms) { persistMs_ = ms; }

  // True when it is time to poll the peripheral again, so the caller does not
  // pay for an SPI read on every pass of the main loop.
  bool shouldCheck(uint32_t nowMs) const;

  // Report the result of a check. Returns true on the single call where the
  // fault has persisted long enough to warrant re-initializing; the monitor
  // then clears itself so recovery is not attempted again until the fault has
  // cleared and returned.
  bool recordCheck(uint32_t nowMs, bool faulted);

  // Forget any fault in progress, e.g. after the caller has re-initialized
  // the peripheral by some other route.
  void clear();

  bool isFaulted() const { return faulted_; }
  uint32_t faultedForMs(uint32_t nowMs) const {
    return faulted_ ? elapsed(nowMs, faultStartedMs_) : 0;
  }
  // True the first time a fault is seen, so the caller can log the onset once
  // rather than on every check.
  bool faultIsNew() const { return faultIsNew_; }

private:
  bool haveChecked_;
  uint32_t lastCheckMs_;
  bool faulted_;
  bool faultIsNew_;
  uint32_t faultStartedMs_;
  uint32_t checkPeriodMs_;
  uint32_t persistMs_;
};

}  // namespace fuel

#endif  // FAULT_MONITOR_H
