#include "CommandParser.h"

namespace fuel {

static bool isSpace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static char lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

// Case-insensitive compare of a token delimited by whitespace or NUL against a
// lowercase literal. Requires the whole token to match, not just a prefix.
static bool tokenEquals(const char* token, uint8_t tokenLen, const char* literal) {
  uint8_t i = 0;
  for (; i < tokenLen; i++) {
    if (literal[i] == '\0') return false;
    if (lower(token[i]) != literal[i]) return false;
  }
  return literal[i] == '\0';
}

bool parseInt32(const char* text, int32_t& out) {
  if (text == 0) return false;
  while (*text && isSpace(*text)) text++;
  if (*text == '\0') return false;

  bool negative = false;
  if (*text == '+' || *text == '-') {
    negative = (*text == '-');
    text++;
  }
  if (*text < '0' || *text > '9') return false;

  uint32_t value = 0;
  while (*text >= '0' && *text <= '9') {
    uint32_t digit = (uint32_t)(*text - '0');
    // Refuse anything that would wrap; a silently truncated K-factor or pin
    // number is worse than a rejected command.
    const uint32_t kMaxU32 = 4294967295U;
    if (value > (uint32_t)((kMaxU32 - digit) / 10U)) return false;
    value = (uint32_t)(value * 10U + digit);
    text++;
  }
  while (*text && isSpace(*text)) text++;
  if (*text != '\0') return false;  // trailing junk

  if (negative) {
    if (value > 2147483648UL) return false;
    out = (value == 2147483648UL) ? (-2147483647L - 1L) : -(int32_t)value;
  } else {
    if (value > 2147483647UL) return false;
    out = (int32_t)value;
  }
  return true;
}

struct VerbEntry {
  const char* name;
  CommandId id;
};

Command parseCommand(const char* line) {
  Command cmd;
  if (line == 0) return cmd;

  while (*line && isSpace(*line)) line++;
  if (*line == '\0') return cmd;  // CMD_NONE

  const char* verb = line;
  uint8_t verbLen = 0;
  while (line[verbLen] && !isSpace(line[verbLen])) verbLen++;

  static const VerbEntry verbs[] = {
    { "help", CMD_HELP },
    { "?", CMD_HELP },
    { "status", CMD_STATUS },
    { "getcanspeed", CMD_GET_CAN_SPEED },
    { "setcanspeed", CMD_SET_CAN_SPEED },
    { "getflowpin", CMD_GET_FLOW_PIN },
    { "setflowpin", CMD_SET_FLOW_PIN },
    { "getmetric", CMD_GET_METRIC },
    { "setmetric", CMD_SET_METRIC },
    { "getk", CMD_GET_K },
    { "setk", CMD_SET_K },
    { "reset", CMD_RESET },
    { "getcapture", CMD_GET_CAPTURE },
    { "setcapture", CMD_SET_CAPTURE },
    { "getcanreset", CMD_GET_CAN_RESET },
    { "setcanreset", CMD_SET_CAN_RESET },
    { "getpulseguard", CMD_GET_PULSE_GUARD },
    { "setpulseguard", CMD_SET_PULSE_GUARD },
    { "debug", CMD_SET_DEBUG }
  };
  const uint8_t verbCount = (uint8_t)(sizeof(verbs) / sizeof(verbs[0]));

  cmd.id = CMD_UNKNOWN;
  for (uint8_t i = 0; i < verbCount; i++) {
    if (tokenEquals(verb, verbLen, verbs[i].name)) {
      cmd.id = verbs[i].id;
      break;
    }
  }

  const char* rest = verb + verbLen;
  while (*rest && isSpace(*rest)) rest++;
  if (*rest != '\0') {
    cmd.hasArg = true;
    cmd.argValid = parseInt32(rest, cmd.arg);
  }
  return cmd;
}

}  // namespace fuel
