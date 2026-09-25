#include "CanCodec.h"

namespace fuel {

uint16_t readU16(const uint8_t* buf) {
  return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

uint32_t readU32(const uint8_t* buf) {
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

void writeU16(uint8_t* buf, uint16_t value) {
  buf[0] = (uint8_t)(value >> 8);
  buf[1] = (uint8_t)(value & 0xFF);
}

void writeU32(uint8_t* buf, uint32_t value) {
  buf[0] = (uint8_t)(value >> 24);
  buf[1] = (uint8_t)((value >> 16) & 0xFF);
  buf[2] = (uint8_t)((value >> 8) & 0xFF);
  buf[3] = (uint8_t)(value & 0xFF);
}

static void zeroFrame(uint8_t* frame) {
  for (uint8_t i = 0; i < CAN_FRAME_LEN; i++) frame[i] = 0;
}

uint8_t encodeDiagFlags(const Diagnostics& diag) {
  uint8_t flags = 0;
  if (diag.refuelArmed) flags |= DIAG_REFUEL_ARMED;
  if (diag.refuelDwelling) flags |= DIAG_REFUEL_DWELLING;
  if (diag.interruptCapture) flags |= DIAG_INTERRUPT_CAPTURE;
  if (diag.stateFresh) flags |= DIAG_STATE_FRESH;
  if (diag.canError) flags |= DIAG_CAN_ERROR;
  if (diag.canResetIgnored) flags |= DIAG_CAN_RESET_IGNORED;
  return flags;
}

// Kept for the tests that pin the wire format of the fuel fields alone. The
// firmware always uses the four-argument form: a default Diagnostics encodes
// "polling capture, state frames stale" as positive facts rather than as
// unknown, which would be a lie on the bus.
void encodeStatus(const FuelStatus& status, bool metric, TxFrames& out) {
  Diagnostics none;
  encodeStatus(status, metric, none, out);
}

void encodeStatus(const FuelStatus& status, bool metric,
                  const Diagnostics& diag, TxFrames& out) {
  zeroFrame(out.usage);
  zeroFrame(out.remaining);
  zeroFrame(out.metricLap);

  // 0x100B0003
  writeU32(&out.usage[0], status.fuelPulses);
  float used = metric ? gallonsToLiters(status.fuelUsedGals) : status.fuelUsedGals;
  writeU16(&out.usage[4], toU16(used * 100.0f));
  float rate = metric ? status.fuelConsumptionGalMin * CCSEC_PER_GALMIN
                      : status.fuelConsumptionGalMin * 10000.0f;
  writeU16(&out.usage[6], toU16(rate));

  // 0x100B0004
  float remaining = metric ? gallonsToLiters(status.fuelRemainingGals)
                           : status.fuelRemainingGals;
  writeU16(&out.remaining[0], toU16(remaining * 100.0f));
  writeU16(&out.remaining[2], toU16(status.fuelRemainingMins * 60.0f));
  writeU16(&out.remaining[4],
           toU16(status.lapsRemaining < 0 ? 0.0f : (float)status.lapsRemaining));
  writeU16(&out.remaining[6], toU16(status.fuelConsumptionGalLap * 10000.0f));

  // 0x100B0005 - liters per lap in bytes 0-1, diagnostics in the rest. Those
  // bytes were previously always zero, so anything already decoding this frame
  // is unaffected.
  writeU16(&out.metricLap[0],
           toU16(gallonsToLiters(status.fuelConsumptionGalLap) * 10000.0f));
  out.metricLap[2] = encodeDiagFlags(diag);
  out.metricLap[3] = diag.resetCount;
  out.metricLap[4] = diag.lastResetReason;
  // Seconds, saturating at about 18 hours - far longer than any session.
  uint32_t uptimeSec = diag.uptimeMs / 1000UL;
  writeU16(&out.metricLap[5], uptimeSec > 65535UL ? (uint16_t)65535
                                                  : (uint16_t)uptimeSec);
  // Exact while small, because the question is usually "any noise at all?",
  // then pinned once it is clearly a problem.
  out.metricLap[7] = diag.rejectedEdges > 255UL ? (uint8_t)255
                                                : (uint8_t)diag.rejectedEdges;
}

DecodeResult decodeCapacityFrame(const uint8_t* buf, uint8_t len, bool metric,
                                 Parameters& params) {
  DecodeResult r;
  if (buf == 0 || len < 2) return r;
  r.accepted = true;

  // Wire units are liters when configured for metric.
  float capacity = (float)readU16(&buf[0]) / 100.0f;
  if (metric) capacity = litersToGallons(capacity);
  params.capacityGals = clampf(capacity, 0.0f, MAX_PLAUSIBLE_CAPACITY_GALS);

  // Fields past the received length keep their previous value instead of
  // being filled from whatever happened to be in the receive buffer.
  if (len >= 6) params.lastLapMs = readU32(&buf[2]);
  if (len >= 7) params.autoReset = (buf[6] != 0);
  return r;
}

DecodeResult decodeStateFrame(const uint8_t* buf, uint8_t len, bool metric,
                              Parameters& params) {
  DecodeResult r;
  if (buf == 0 || len < 2) return r;
  r.accepted = true;

  float speed = (float)readU16(&buf[0]) / 10.0f;
  if (metric) speed = kphToMph(speed);
  params.speedMph = clampf(speed, 0.0f, MAX_PLAUSIBLE_SPEED_MPH);

  if (len >= 4) {
    float level = (float)readU16(&buf[2]) / 100.0f;
    if (metric) level = litersToGallons(level);
    params.fuelLevelGals = clampf(level, 0.0f, MAX_PLAUSIBLE_CAPACITY_GALS);
  }
  if (len >= 5) params.fuelFull = (buf[4] != 0);
  // A reset is only honored when the byte that carries it was actually
  // received. Reading buf[5] out of a short frame is what made resets fire at
  // random.
  if (len >= 6) r.resetRequested = (buf[5] != 0);
  return r;
}

DecodeResult decodeFrame(uint32_t canId, const uint8_t* buf, uint8_t len,
                         bool metric, Parameters& params) {
  if (canId == CAN_ID_RX_CAPACITY) {
    return decodeCapacityFrame(buf, len, metric, params);
  }
  if (canId == CAN_ID_RX_STATE) {
    return decodeStateFrame(buf, len, metric, params);
  }
  return DecodeResult();
}

}  // namespace fuel
