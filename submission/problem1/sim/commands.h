#ifndef COMMANDS_H
#define COMMANDS_H

// Inbox for on-demand dispatch: the API appends one entry per accepted
// request to backend/commands.json; the sim polls it each tick and consumes
// the entry when an idle drone lifts off for it.
//
// Schema (written by api/fleet_manager.py):
//   [{"drone_id": 1, "lat": 31.79, "lng": 77.0,
//     "address": "Drop zone", "package_id": "PKG-1"}, ...]
//
// All readers/writers are fail-soft: missing or corrupt file => empty inbox,
// tick is skipped, never a crash. Writes are atomic (tmp file + rename) so a
// concurrent reader never sees a torn file.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Command {
  int drone_id = -1;
  double lat = 0.0;
  double lng = 0.0;
  std::string address;
  std::string package_id;
};

namespace cmd_detail {

// Read entire file into a string. Returns false on any I/O failure.
inline bool readFile(const std::string& path, std::string& out) {
  std::ifstream f(path.c_str());
  if (!f.is_open()) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

// Parse a JSON number starting at s[pos] (after whitespace). Advances pos.
inline bool parseNumber(const std::string& s, size_t& pos, double& out) {
  while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' ||
                            s[pos] == '\n' || s[pos] == '\r'))
    pos++;
  const char* start = s.c_str() + pos;
  char* end = nullptr;
  double v = std::strtod(start, &end);
  if (end == start) return false;
  pos = static_cast<size_t>(end - s.c_str());
  out = v;
  return true;
}

// Parse a JSON string starting at s[pos] (expects opening quote).
// Handles \" and \\ escapes. Advances pos past closing quote.
inline bool parseString(const std::string& s, size_t& pos, std::string& out) {
  while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' ||
                            s[pos] == '\n' || s[pos] == '\r'))
    pos++;
  if (pos >= s.size() || s[pos] != '"') return false;
  pos++;
  std::string acc;
  while (pos < s.size()) {
    char c = s[pos];
    if (c == '\\' && pos + 1 < s.size()) {
      char n = s[pos + 1];
      if (n == '"' || n == '\\' || n == '/') acc += n;
      else if (n == 'n') acc += '\n';
      else if (n == 't') acc += '\t';
      else { acc += c; acc += n; }
      pos += 2;
    } else if (c == '"') {
      pos++;
      out = acc;
      return true;
    } else {
      acc += c;
      pos++;
    }
  }
  return false;
}

// Find `"key"` at or after `from`, then advance past the following ':'.
inline bool seekKey(const std::string& s, const char* key, size_t& pos) {
  std::string pat = std::string("\"") + key + "\"";
  size_t k = s.find(pat, pos);
  if (k == std::string::npos) return false;
  size_t c = s.find(':', k + pat.size());
  if (c == std::string::npos) return false;
  pos = c + 1;
  return true;
}

inline void escapeInto(const std::string& in, std::string& out) {
  for (char c : in) {
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else out += c;
  }
}

}  // namespace cmd_detail

// Read all inbox entries. Empty vector on missing/corrupt file.
inline std::vector<Command> readCommands(const std::string& path) {
  std::vector<Command> cmds;
  std::string s;
  if (!cmd_detail::readFile(path, s)) return cmds;

  size_t pos = 0;
  while (true) {
    Command c;
    double num = 0.0;
    if (!cmd_detail::seekKey(s, "drone_id", pos)) break;
    if (!cmd_detail::parseNumber(s, pos, num)) break;
    c.drone_id = static_cast<int>(num);
    if (!cmd_detail::seekKey(s, "lat", pos)) break;
    if (!cmd_detail::parseNumber(s, pos, c.lat)) break;
    if (!cmd_detail::seekKey(s, "lng", pos)) break;
    if (!cmd_detail::parseNumber(s, pos, c.lng)) break;
    if (!cmd_detail::seekKey(s, "address", pos)) break;
    if (!cmd_detail::parseString(s, pos, c.address)) break;
    if (!cmd_detail::seekKey(s, "package_id", pos)) break;
    if (!cmd_detail::parseString(s, pos, c.package_id)) break;
    cmds.push_back(c);
  }
  return cmds;
}

// Atomically rewrite the inbox with the given entries (used to drop
// consumed ones). Writes "[]" when empty so readers always see valid JSON.
inline bool writeCommands(const std::string& path,
                          const std::vector<Command>& cmds) {
  std::string tmp = path + ".tmp";
  std::ofstream f(tmp.c_str(), std::ios::trunc);
  if (!f.is_open()) return false;
  f << "[";
  for (size_t i = 0; i < cmds.size(); i++) {
    const Command& c = cmds[i];
    std::string addr, pkg;
    cmd_detail::escapeInto(c.address, addr);
    cmd_detail::escapeInto(c.package_id, pkg);
    if (i > 0) f << ", ";
    f << "{\"drone_id\": " << c.drone_id << ", \"lat\": " << c.lat
      << ", \"lng\": " << c.lng << ", \"address\": \"" << addr
      << "\", \"package_id\": \"" << pkg << "\"}";
  }
  f << "]\n";
  f.close();
  if (!f) return false;
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    std::remove(tmp.c_str());
    return false;
  }
  return true;
}

#endif
