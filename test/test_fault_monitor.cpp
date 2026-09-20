#include "test_framework.h"
#include "../src/FaultMonitor.h"

using namespace fuel;

TEST(FaultMonitor, checks_on_the_first_pass) {
  FaultMonitor m;
  CHECK(m.shouldCheck(0));
  CHECK(m.shouldCheck(123456));
}

TEST(FaultMonitor, rate_limits_checks) {
  FaultMonitor m;
  m.recordCheck(1000, false);
  CHECK(!m.shouldCheck(1000));
  CHECK(!m.shouldCheck(1000 + FaultMonitor::DEFAULT_CHECK_PERIOD_MS - 1));
  CHECK(m.shouldCheck(1000 + FaultMonitor::DEFAULT_CHECK_PERIOD_MS));
}

TEST(FaultMonitor, healthy_peripheral_never_asks_for_recovery) {
  FaultMonitor m;
  for (uint32_t t = 0; t < 600000UL; t += FaultMonitor::DEFAULT_CHECK_PERIOD_MS) {
    CHECK(!m.recordCheck(t, false));
  }
  CHECK(!m.isFaulted());
}

// A fault has to be latched, not a momentary error-passive blip on a noisy
// bus, before the controller is reinitialized.
TEST(FaultMonitor, does_not_recover_before_the_fault_persists) {
  FaultMonitor m;
  uint32_t t = 0;
  CHECK(!m.recordCheck(t, true));
  CHECK(m.isFaulted());
  for (t = 5000; t < FaultMonitor::DEFAULT_PERSIST_MS; t += 5000) {
    CHECK(!m.recordCheck(t, true));
  }
  CHECK(m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS, true));
}

TEST(FaultMonitor, a_fault_that_clears_resets_the_timer) {
  FaultMonitor m;
  m.recordCheck(0, true);
  m.recordCheck(5000, true);
  m.recordCheck(10000, true);
  CHECK(!m.recordCheck(15000, false));  // recovered on its own
  CHECK(!m.isFaulted());
  // The earlier accumulation must not carry over.
  CHECK(!m.recordCheck(20000, true));
  CHECK(!m.recordCheck(45000, true));
  CHECK(m.recordCheck(50000, true));
}

TEST(FaultMonitor, does_not_recover_repeatedly_while_still_faulted) {
  FaultMonitor m;
  m.recordCheck(0, true);
  CHECK(m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS, true));
  // Next check restarts the clock rather than firing again immediately.
  CHECK(!m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS + 5000, true));
  CHECK(!m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS + 20000, true));
  CHECK(m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS + 5000 +
                          FaultMonitor::DEFAULT_PERSIST_MS, true));
}

// So the console logs the onset once instead of every five seconds.
TEST(FaultMonitor, reports_a_new_fault_only_once) {
  FaultMonitor m;
  m.recordCheck(0, true);
  CHECK(m.faultIsNew());
  m.recordCheck(5000, true);
  CHECK(!m.faultIsNew());
  m.recordCheck(10000, false);
  CHECK(!m.faultIsNew());
  m.recordCheck(15000, true);
  CHECK(m.faultIsNew());
}

TEST(FaultMonitor, tracks_how_long_a_fault_has_been_present) {
  FaultMonitor m;
  m.recordCheck(1000, true);
  CHECK_EQ(m.faultedForMs(1000), 0u);
  CHECK_EQ(m.faultedForMs(9000), 8000u);
  m.recordCheck(9000, false);
  CHECK_EQ(m.faultedForMs(9000), 0u);
}

TEST(FaultMonitor, clear_discards_a_fault_in_progress) {
  FaultMonitor m;
  m.recordCheck(0, true);
  m.clear();
  CHECK(!m.isFaulted());
  CHECK(!m.recordCheck(FaultMonitor::DEFAULT_PERSIST_MS, true));
}

TEST(FaultMonitor, survives_millis_rollover) {
  FaultMonitor m;
  const uint32_t start = 0xFFFFFFFFUL - 10000UL;
  CHECK(!m.recordCheck(start, true));
  CHECK(!m.recordCheck((uint32_t)(start + 15000UL), true));  // past the wrap
  CHECK(m.recordCheck((uint32_t)(start + FaultMonitor::DEFAULT_PERSIST_MS), true));
}

TEST(FaultMonitor, periods_are_configurable) {
  FaultMonitor m;
  m.setCheckPeriodMs(1000);
  m.setPersistMs(3000);
  m.recordCheck(0, true);
  CHECK(m.shouldCheck(1000));
  CHECK(!m.recordCheck(1000, true));
  CHECK(!m.recordCheck(2000, true));
  CHECK(m.recordCheck(3000, true));
}
