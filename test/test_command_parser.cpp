#include "test_framework.h"
#include "../src/CommandParser.h"

using namespace fuel;

TEST(CommandParser, recognizes_every_verb) {
  CHECK_EQ(parseCommand("getcanspeed").id, CMD_GET_CAN_SPEED);
  CHECK_EQ(parseCommand("setcanspeed 16").id, CMD_SET_CAN_SPEED);
  CHECK_EQ(parseCommand("getflowpin").id, CMD_GET_FLOW_PIN);
  CHECK_EQ(parseCommand("setflowpin 0").id, CMD_SET_FLOW_PIN);
  CHECK_EQ(parseCommand("getmetric").id, CMD_GET_METRIC);
  CHECK_EQ(parseCommand("setmetric 1").id, CMD_SET_METRIC);
  CHECK_EQ(parseCommand("getk").id, CMD_GET_K);
  CHECK_EQ(parseCommand("setk 68000").id, CMD_SET_K);
  CHECK_EQ(parseCommand("reset").id, CMD_RESET);
  CHECK_EQ(parseCommand("status").id, CMD_STATUS);
  CHECK_EQ(parseCommand("help").id, CMD_HELP);
  CHECK_EQ(parseCommand("?").id, CMD_HELP);
  CHECK_EQ(parseCommand("getcapture").id, CMD_GET_CAPTURE);
  CHECK_EQ(parseCommand("setcapture 1").id, CMD_SET_CAPTURE);
  CHECK_EQ(parseCommand("getpulseguard").id, CMD_GET_PULSE_GUARD);
  CHECK_EQ(parseCommand("setpulseguard 400").id, CMD_SET_PULSE_GUARD);
  CHECK_EQ(parseCommand("debug 1").id, CMD_SET_DEBUG);
}

TEST(CommandParser, blank_input_is_not_a_command) {
  CHECK_EQ(parseCommand("").id, CMD_NONE);
  CHECK_EQ(parseCommand("   ").id, CMD_NONE);
  CHECK_EQ(parseCommand("\r\n").id, CMD_NONE);
  CHECK_EQ(parseCommand(0).id, CMD_NONE);
}

// The old parser used startsWith(), so any line beginning with "reset" cleared
// the trip counter - including typos and anything a noisy serial link
// delivered with characters appended.
TEST(CommandParser, verbs_must_match_exactly) {
  CHECK_EQ(parseCommand("resetfoo").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("resets").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("getkk").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("setcanspeedx 1").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("xreset").id, CMD_UNKNOWN);
}

// "reset 5" is still a reset: the verb matched and the argument is ignored.
TEST(CommandParser, argument_is_separated_by_whitespace) {
  Command c = parseCommand("reset 5");
  CHECK_EQ(c.id, CMD_RESET);
  CHECK(c.hasArg);
}

TEST(CommandParser, is_case_insensitive) {
  CHECK_EQ(parseCommand("SETK 68000").id, CMD_SET_K);
  CHECK_EQ(parseCommand("SetCanSpeed 16").id, CMD_SET_CAN_SPEED);
  CHECK_EQ(parseCommand("ReSeT").id, CMD_RESET);
}

TEST(CommandParser, tolerates_surrounding_whitespace) {
  Command c = parseCommand("   setk   68000  \r\n");
  CHECK_EQ(c.id, CMD_SET_K);
  CHECK(c.argValid);
  CHECK_EQ(c.arg, 68000L);
}

TEST(CommandParser, extracts_arguments) {
  Command c = parseCommand("setcanspeed 16");
  CHECK_EQ(c.id, CMD_SET_CAN_SPEED);
  CHECK(c.hasArg);
  CHECK(c.argValid);
  CHECK_EQ(c.arg, 16L);

  c = parseCommand("setk 68000");
  CHECK(c.argValid);
  CHECK_EQ(c.arg, 68000L);
}

TEST(CommandParser, reports_a_missing_argument) {
  Command c = parseCommand("setk");
  CHECK_EQ(c.id, CMD_SET_K);
  CHECK(!c.hasArg);
  CHECK(!c.argValid);
}

// Arduino's toInt() returns 0 for unparseable text, so "setflowpin abc"
// silently reconfigured the device to pin 0 and "setk abc" to a K of 0.
TEST(CommandParser, non_numeric_arguments_are_rejected_not_read_as_zero) {
  Command c = parseCommand("setflowpin abc");
  CHECK_EQ(c.id, CMD_SET_FLOW_PIN);
  CHECK(c.hasArg);
  CHECK(!c.argValid);

  c = parseCommand("setk abc");
  CHECK(c.hasArg);
  CHECK(!c.argValid);

  c = parseCommand("setk 12abc");
  CHECK(!c.argValid);

  c = parseCommand("setk 12 34");
  CHECK(!c.argValid);
}

TEST(CommandParser, parses_whole_numbers) {
  int32_t v = 0;
  CHECK(parseInt32("0", v));
  CHECK_EQ(v, 0L);
  CHECK(parseInt32("68000", v));
  CHECK_EQ(v, 68000L);
  CHECK(parseInt32("  42  ", v));
  CHECK_EQ(v, 42L);
  CHECK(parseInt32("+7", v));
  CHECK_EQ(v, 7L);
  CHECK(parseInt32("-7", v));
  CHECK_EQ(v, -7L);
}

TEST(CommandParser, rejects_malformed_numbers) {
  int32_t v = 0;
  CHECK(!parseInt32("", v));
  CHECK(!parseInt32("   ", v));
  CHECK(!parseInt32("abc", v));
  CHECK(!parseInt32("1.5", v));
  CHECK(!parseInt32("0x10", v));
  CHECK(!parseInt32("- 7", v));
  CHECK(!parseInt32("--7", v));
  CHECK(!parseInt32(0, v));
}

TEST(CommandParser, rejects_numbers_that_do_not_fit) {
  int32_t v = 0;
  CHECK(parseInt32("2147483647", v));
  CHECK_EQ(v, 2147483647L);
  CHECK(!parseInt32("2147483648", v));
  CHECK(!parseInt32("4294967296", v));
  CHECK(!parseInt32("99999999999999999999", v));
  CHECK(parseInt32("-2147483648", v));
  CHECK(!parseInt32("-2147483649", v));
}

TEST(CommandParser, unknown_verbs_are_reported) {
  CHECK_EQ(parseCommand("frobnicate").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("set").id, CMD_UNKNOWN);
  CHECK_EQ(parseCommand("get 1").id, CMD_UNKNOWN);
}

// A serial line picking up noise must not be able to trip a destructive
// command, whatever bytes arrive.
TEST(CommandParser, garbage_input_never_resolves_to_reset) {
  const char* junk[] = {
    "\x01\x02\x03", "reset\x01", "!@#$%", "0000", "re set",
    "RESETX", "  reset_", "res"
  };
  for (size_t i = 0; i < sizeof(junk) / sizeof(junk[0]); i++) {
    CHECK(parseCommand(junk[i]).id != CMD_RESET);
  }
}
