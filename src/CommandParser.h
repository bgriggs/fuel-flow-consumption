// CommandParser.h - Parses serial console commands.
//
// Works on a plain char buffer rather than Arduino's String. String allocates
// on a heap that has only a couple of kilobytes to share with everything else,
// and repeated command entry fragments it. Matching is also exact rather than
// by prefix, so "resetfoo" is an error instead of silently resetting the trip.
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>

namespace fuel {

enum CommandId {
  CMD_NONE = 0,   // blank line
  CMD_UNKNOWN,    // verb not recognized
  CMD_HELP,
  CMD_STATUS,
  CMD_GET_CAN_SPEED,
  CMD_SET_CAN_SPEED,
  CMD_GET_FLOW_PIN,
  CMD_SET_FLOW_PIN,
  CMD_GET_METRIC,
  CMD_SET_METRIC,
  CMD_GET_K,
  CMD_SET_K,
  CMD_RESET,
  CMD_GET_CAPTURE,
  CMD_SET_CAPTURE,
  CMD_SET_DEBUG
};

struct Command {
  CommandId id;
  int32_t arg;
  bool hasArg;    // an argument token followed the verb
  bool argValid;  // ...and it parsed as a whole number

  Command() : id(CMD_NONE), arg(0), hasArg(false), argValid(false) {}
};

// Parse one line. The input is not modified and case is ignored.
Command parseCommand(const char* line);

// Strict whole-number parse: the token must be all digits after an optional
// sign, and must fit in an int32. Unlike Arduino's toInt(), "abc" is rejected
// rather than silently read as 0 - which is what let "setflowpin abc" quietly
// reconfigure the device to pin 0.
bool parseInt32(const char* text, int32_t& out);

}  // namespace fuel

#endif  // COMMAND_PARSER_H
