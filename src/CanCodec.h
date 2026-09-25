// CanCodec.h - Serialization of the fuel protocol to and from CAN payloads.
//
// Kept separate from the transceiver driver so the wire format can be tested
// byte-for-byte on a host without any CAN hardware.
#ifndef CAN_CODEC_H
#define CAN_CODEC_H

#include <stdint.h>
#include "FuelMath.h"

namespace fuel {

// Extended (29-bit) identifiers.
const uint32_t CAN_ID_RX_CAPACITY = 0x100B0001UL;  // capacity, last lap, auto-reset
const uint32_t CAN_ID_RX_STATE = 0x100B0002UL;     // speed, level, full, reset
const uint32_t CAN_ID_TX_USAGE = 0x100B0003UL;     // pulses, used, rate
const uint32_t CAN_ID_TX_REMAINING = 0x100B0004UL; // remaining, endurance, laps
const uint32_t CAN_ID_TX_METRIC_LAP = 0x100B0005UL;// liters per lap

const uint8_t CAN_FRAME_LEN = 8;

// Why the fuel counter was last zeroed. Logged so a counter that drops can be
// attributed without a laptop attached.
const uint8_t RESET_NONE = 0;
const uint8_t RESET_COMMANDED = 1;  // the reset byte on the state frame
const uint8_t RESET_REFUEL = 2;     // the refuel detector fired
const uint8_t RESET_CONSOLE = 3;    // somebody typed 'reset'
const uint8_t RESET_REASON_MAX = RESET_CONSOLE;

// Bit positions in the diagnostics flags byte.
const uint8_t DIAG_REFUEL_ARMED = 0x01;
const uint8_t DIAG_REFUEL_DWELLING = 0x02;
const uint8_t DIAG_INTERRUPT_CAPTURE = 0x04;
const uint8_t DIAG_STATE_FRESH = 0x08;
const uint8_t DIAG_CAN_ERROR = 0x10;
// Sticky: a reset command arrived on the bus and was ignored. Visible in a
// log, so a counter that did NOT reset can be explained too.
const uint8_t DIAG_CAN_RESET_IGNORED = 0x20;

// Internal state that cannot be inferred from the signals the dash already
// puts on the bus. Rides in the unused bytes of the liters-per-lap frame, so
// it needs no new identifier.
//
// Uptime is the important one: without it a counter that drops cannot be told
// apart from the device having rebooted and restored a stale value from
// EEPROM, which is an electrical fault rather than a logic one.
struct Diagnostics {
  bool refuelArmed;
  bool refuelDwelling;
  bool interruptCapture;
  bool stateFresh;
  bool canError;
  bool canResetIgnored;
  uint8_t resetCount;       // since boot, saturating
  uint8_t lastResetReason;
  uint32_t uptimeMs;
  uint32_t rejectedEdges;

  Diagnostics()
    : refuelArmed(false), refuelDwelling(false), interruptCapture(false),
      stateFresh(false), canError(false), canResetIgnored(false),
      resetCount(0),
      lastResetReason(RESET_NONE), uptimeMs(0), rejectedEdges(0) {}
};

struct TxFrames {
  uint8_t usage[CAN_FRAME_LEN];
  uint8_t remaining[CAN_FRAME_LEN];
  uint8_t metricLap[CAN_FRAME_LEN];
};

// Build all three outbound frames. `metric` switches volumes to liters and the
// instantaneous rate to cc/sec, matching the documented protocol.
void encodeStatus(const FuelStatus& status, bool metric, TxFrames& out);

// As above, and additionally fills the diagnostics bytes of the third frame.
void encodeStatus(const FuelStatus& status, bool metric,
                  const Diagnostics& diag, TxFrames& out);

// Pack the diagnostics flags byte, exposed so a decoder and the tests agree.
uint8_t encodeDiagFlags(const Diagnostics& diag);

// Decode results. `accepted` is false when the frame was too short to carry
// even its first field; fields beyond the received length are left untouched
// rather than filled from uninitialized memory.
struct DecodeResult {
  bool accepted;
  bool resetRequested;
  DecodeResult() : accepted(false), resetRequested(false) {}
};

// 0x100B0001: capacity x100 (0,2), last lap ms (2,4), auto-reset (6,1)
DecodeResult decodeCapacityFrame(const uint8_t* buf, uint8_t len, bool metric,
                                 Parameters& params);

// 0x100B0002: speed x10 (0,2), level x100 (2,2), full (4,1), reset (5,1)
DecodeResult decodeStateFrame(const uint8_t* buf, uint8_t len, bool metric,
                              Parameters& params);

// Dispatch on identifier. Unknown identifiers return accepted == false.
DecodeResult decodeFrame(uint32_t canId, const uint8_t* buf, uint8_t len,
                         bool metric, Parameters& params);

// Big-endian helpers, shared with the tests.
uint16_t readU16(const uint8_t* buf);
uint32_t readU32(const uint8_t* buf);
void writeU16(uint8_t* buf, uint16_t value);
void writeU32(uint8_t* buf, uint32_t value);

}  // namespace fuel

#endif  // CAN_CODEC_H
