#include "FaultMonitor.h"

namespace fuel {

FaultMonitor::FaultMonitor()
  : haveChecked_(false),
    lastCheckMs_(0),
    faulted_(false),
    faultIsNew_(false),
    faultStartedMs_(0),
    checkPeriodMs_(DEFAULT_CHECK_PERIOD_MS),
    persistMs_(DEFAULT_PERSIST_MS) {}

bool FaultMonitor::shouldCheck(uint32_t nowMs) const {
  if (!haveChecked_) return true;
  return elapsed(nowMs, lastCheckMs_) >= checkPeriodMs_;
}

void FaultMonitor::clear() {
  faulted_ = false;
  faultIsNew_ = false;
  faultStartedMs_ = 0;
}

bool FaultMonitor::recordCheck(uint32_t nowMs, bool faulted) {
  lastCheckMs_ = nowMs;
  haveChecked_ = true;
  faultIsNew_ = false;

  if (!faulted) {
    // Healthy again. Anything that was building up is discarded, so a fault
    // has to persist without interruption to count.
    clear();
    return false;
  }

  if (!faulted_) {
    faulted_ = true;
    faultIsNew_ = true;
    faultStartedMs_ = nowMs;
    return false;
  }

  if (elapsed(nowMs, faultStartedMs_) < persistMs_) return false;

  // Latched long enough to be worth recovering from. Reset the state so the
  // next attempt is a fresh persistMs_ away rather than firing every check.
  faulted_ = false;
  faultStartedMs_ = 0;
  return true;
}

}  // namespace fuel
