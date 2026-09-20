#include "test_framework.h"
#include "../src/FuelMath.h"

using namespace fuel;

namespace {
const float NAN_F = 0.0f / 0.0f;
const float INF_F = 1.0f / 0.0f;
}  // namespace

TEST(FuelMath, elapsed_measures_forward_time) {
  CHECK_EQ(elapsed(1000, 400), 600u);
  CHECK_EQ(elapsed(400, 400), 0u);
}

// millis() wraps to zero every ~49.7 days. Unsigned subtraction keeps giving
// the right answer across the wrap, which is why every duration in the
// firmware goes through this helper.
TEST(FuelMath, elapsed_is_correct_across_rollover) {
  CHECK_EQ(elapsed(5, 0xFFFFFFFFUL - 4), 10u);
  CHECK_EQ(elapsed(0, 0xFFFFFFFFUL), 1u);
  CHECK_EQ(elapsed(999, 0xFFFFFF00UL), 1255u);  // 256 before the wrap, plus 999
}

TEST(FuelMath, converts_units) {
  CHECK_NEAR(gallonsToLiters(1.0f), 3.785411784f, 1e-5);
  CHECK_NEAR(litersToGallons(3.785411784f), 1.0f, 1e-5);
  CHECK_NEAR(kphToMph(100.0f), 62.1371192f, 1e-4);
  CHECK_NEAR(litersToGallons(gallonsToLiters(7.5f)), 7.5f, 1e-4);
}

TEST(FuelMath, detects_nan) {
  CHECK(isNotANumber(NAN_F));
  CHECK(!isNotANumber(0.0f));
  CHECK(!isNotANumber(-1.5f));
  CHECK(!isNotANumber(INF_F));
}

TEST(FuelMath, clampf_bounds_values) {
  CHECK_NEAR(clampf(5.0f, 0.0f, 10.0f), 5.0f, 1e-6);
  CHECK_NEAR(clampf(-1.0f, 0.0f, 10.0f), 0.0f, 1e-6);
  CHECK_NEAR(clampf(11.0f, 0.0f, 10.0f), 10.0f, 1e-6);
  CHECK_NEAR(clampf(INF_F, 0.0f, 10.0f), 10.0f, 1e-6);
  CHECK_NEAR(clampf(-INF_F, 0.0f, 10.0f), 0.0f, 1e-6);
  // NaN has no sensible position in a range; the low bound is the safe pick
  // for the quantities this is used on.
  CHECK_NEAR(clampf(NAN_F, 0.0f, 10.0f), 0.0f, 1e-6);
}

// Casting an out-of-range or NaN float to an unsigned integer is undefined
// behavior. On AVR it produced wrapped values, so an overflowing reading
// could be transmitted as a small, believable number.
TEST(FuelMath, toU16_saturates_instead_of_wrapping) {
  CHECK_EQ(toU16(0.0f), (uint16_t)0);
  CHECK_EQ(toU16(1234.0f), (uint16_t)1234);
  CHECK_EQ(toU16(65535.0f), (uint16_t)65535);
  CHECK_EQ(toU16(65536.0f), (uint16_t)65535);
  CHECK_EQ(toU16(1e9f), (uint16_t)65535);
  CHECK_EQ(toU16(INF_F), (uint16_t)65535);
}

TEST(FuelMath, toU16_maps_negatives_and_nan_to_zero) {
  CHECK_EQ(toU16(-1.0f), (uint16_t)0);
  CHECK_EQ(toU16(-1e9f), (uint16_t)0);
  CHECK_EQ(toU16(-INF_F), (uint16_t)0);
  CHECK_EQ(toU16(NAN_F), (uint16_t)0);
}

TEST(FuelMath, toU16_truncates_towards_zero) {
  CHECK_EQ(toU16(1.9f), (uint16_t)1);
  CHECK_EQ(toU16(0.9f), (uint16_t)0);
}

TEST(FuelMath, toU32_saturates_instead_of_wrapping) {
  CHECK_EQ(toU32(0.0f), 0u);
  CHECK_EQ(toU32(1234.0f), 1234u);
  CHECK_EQ(toU32(1e20f), 4294967295UL);
  CHECK_EQ(toU32(INF_F), 4294967295UL);
  CHECK_EQ(toU32(-1.0f), 0u);
  CHECK_EQ(toU32(NAN_F), 0u);
}

TEST(FuelMath, lap_plausibility_accepts_real_lap_times_and_rejects_junk) {
  CHECK(isPlausibleLapMs(30000));    // 30 s kart lap
  CHECK(isPlausibleLapMs(90000));    // 1:30 road course
  CHECK(isPlausibleLapMs(600000));   // 10 min Nordschleife
  CHECK(!isPlausibleLapMs(0));
  CHECK(!isPlausibleLapMs(90));      // seconds mistaken for milliseconds
  CHECK(!isPlausibleLapMs(0xFFFFFFFFUL));
}
