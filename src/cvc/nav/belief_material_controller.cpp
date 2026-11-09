/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// belief_material_controller.cpp — the belief-material controller's contract and inputs
// (belief_material_controller.h, C1 of NAV-BELIEF-MATERIAL-CONTROLLER-ROADMAP.md). BIT tier
// against GRL-SNAM's Python controller: built with -ffp-contract=off (CMake), and every float op
// in the token builder and visible_obstacles is pinned to its numpy twin's order. Do not "tidy"
// the arithmetic.

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/nav/belief_material_controller.h>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {

// ---------------------------------------------------------------------------
// Python's float repr
// ---------------------------------------------------------------------------

namespace {

// The decimal string `digits` (no sign, no leading zeros) with its last digit moved by delta
// (+1 / -1), carrying; empty when it would lose its leading digit or gain one.
std::string step_last_digit(std::string digits, int delta) {
  int i = static_cast<int>(digits.size()) - 1;
  while (i >= 0) {
    const int d = digits[i] - '0' + delta;
    if (d >= 0 && d <= 9) {
      digits[i] = static_cast<char>('0' + d);
      break;
    }
    digits[i] = delta > 0 ? '0' : '9';
    --i;
  }
  if (i < 0 || digits[0] == '0')
    return std::string();
  return digits;
}

// The shortest decimal digits that read back as mag (> 0, finite) and the decimal exponent of the
// first digit: mag ~= 0.d1d2... * 10^(exp10 + 1). %.*e gives the correctly rounded p digits; at a
// power of two the rounding interval is lopsided, so the p-digit neighbour may read back when the
// rounded digits do not (as in state_exec's format_number_shortest_fallback). Candidates are read
// back as "<digits>e<exp>", with no radix character, so the locale does not matter.
void shortest_digits(double mag, std::string &digits, int &exp10) {
  digits.clear();
  exp10 = 0;
  for (int p = 1; p <= 17 && digits.empty(); ++p) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*e", p - 1, mag);
    std::string m;
    const char *c = buf;
    for (; *c && *c != 'e' && *c != 'E'; ++c)
      if (*c >= '0' && *c <= '9')
        m += *c;
    const int e = *c ? std::atoi(c + 1) : 0;
    const auto reads_back = [&](const std::string &cand) {
      const std::string text = cand + "e" + std::to_string(e - (p - 1));
      return std::strtod(text.c_str(), nullptr) == mag;
    };
    for (const std::string &cand : {m, step_last_digit(m, -1), step_last_digit(m, +1)})
      if (!cand.empty() && reads_back(cand)) {
        digits = cand;
        exp10 = e;
        break;
      }
  }
  while (digits.size() > 1 && digits.back() == '0')
    digits.pop_back();
}

} // namespace

std::string python_float_repr(double v) {
  if (std::isnan(v))
    return "NaN";
  if (std::isinf(v))
    return v < 0 ? "-Infinity" : "Infinity";
  const std::string sign = std::signbit(v) ? "-" : "";
  const double mag = std::fabs(v);
  if (mag == 0.0)
    return sign + "0.0";
  std::string digits;
  int exp10 = 0;
  shortest_digits(mag, digits, exp10);
  const int n = static_cast<int>(digits.size());
  const int decpt = exp10 + 1; // mag = 0.digits * 10^decpt, as dtoa reports it
  // Python's float_repr_style 'short' (format_float_short, mode 'r', Py_DTSF_ADD_DOT_0).
  if (decpt <= -4 || decpt > 16) {
    std::string s = sign + digits.substr(0, 1);
    if (n > 1)
      s += "." + digits.substr(1);
    const int ae = exp10 < 0 ? -exp10 : exp10;
    s += std::string(exp10 < 0 ? "e-" : "e+") + (ae < 10 ? "0" : "") + std::to_string(ae);
    return s;
  }
  if (decpt <= 0)
    return sign + "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
  if (decpt >= n)
    return sign + digits + std::string(static_cast<std::size_t>(decpt - n), '0') + ".0";
  return sign + digits.substr(0, static_cast<std::size_t>(decpt)) + "." +
         digits.substr(static_cast<std::size_t>(decpt));
}

// ---------------------------------------------------------------------------
// A JSON reader scoped to the meta record
// ---------------------------------------------------------------------------

namespace {

// A parsed JSON value. Objects keep their members in order, duplicates included; lookups take the
// last, as a Python dict built by json.loads does.
struct json_value {
  enum class kind { null, boolean, number, string, array, object } k = kind::null;
  bool b = false;
  double num = 0.0;
  bool integer = false; // a number written with no fraction or exponent
  std::string str;
  std::vector<json_value> arr;
  // An object's members, keys and values in parallel (a vector of pair<string, json_value> needs
  // json_value complete here, and clang with libstdc++ refuses it).
  std::vector<std::string> obj_keys;
  std::vector<json_value> obj_vals;

  const json_value *find(const std::string &key) const {
    const json_value *found = nullptr;
    for (std::size_t m = 0; m < obj_keys.size(); ++m)
      if (obj_keys[m] == key)
        found = &obj_vals[m];
    return found;
  }
};

struct json_error {};

// Python's json.loads (strict): its whitespace, its number grammar
// -?(0|[1-9]\d*)(\.\d+)?([eE][-+]?\d+)?, its escapes (\uXXXX with surrogate pairs), no raw control
// characters in strings, and the NaN / Infinity / -Infinity constants. Throws json_error.
class json_reader {
public:
  explicit json_reader(const std::string &text) : s_(text) {}

  json_value document() {
    json_value v = value(0);
    skip_ws();
    if (i_ != s_.size())
      throw json_error(); // "Extra data"
    return v;
  }

private:
  const std::string &s_;
  std::size_t i_ = 0;
  static constexpr int kMaxDepth = 512;

  bool at_end() const { return i_ >= s_.size(); }
  char peek() const { return at_end() ? '\0' : s_[i_]; }
  void skip_ws() {
    while (!at_end() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r'))
      ++i_;
  }
  bool literal(const char *word) {
    const std::size_t n = std::strlen(word);
    if (s_.compare(i_, n, word) != 0)
      return false;
    i_ += n;
    return true;
  }

  json_value value(int depth) {
    if (depth > kMaxDepth)
      throw json_error();
    skip_ws();
    json_value v;
    const char c = peek();
    if (c == '{') {
      ++i_;
      v.k = json_value::kind::object;
      skip_ws();
      if (peek() == '}') {
        ++i_;
        return v;
      }
      for (;;) {
        skip_ws();
        if (peek() != '"')
          throw json_error();
        std::string key = string();
        skip_ws();
        if (peek() != ':')
          throw json_error();
        ++i_;
        json_value member = value(depth + 1);
        v.obj_keys.push_back(std::move(key));
        v.obj_vals.push_back(std::move(member));
        skip_ws();
        if (peek() == ',') {
          ++i_;
          continue;
        }
        if (peek() == '}') {
          ++i_;
          return v;
        }
        throw json_error();
      }
    }
    if (c == '[') {
      ++i_;
      v.k = json_value::kind::array;
      skip_ws();
      if (peek() == ']') {
        ++i_;
        return v;
      }
      for (;;) {
        v.arr.push_back(value(depth + 1));
        skip_ws();
        if (peek() == ',') {
          ++i_;
          continue;
        }
        if (peek() == ']') {
          ++i_;
          return v;
        }
        throw json_error();
      }
    }
    if (c == '"') {
      v.k = json_value::kind::string;
      v.str = string();
      return v;
    }
    if (literal("null"))
      return v;
    if (literal("true")) {
      v.k = json_value::kind::boolean;
      v.b = true;
      return v;
    }
    if (literal("false")) {
      v.k = json_value::kind::boolean;
      return v;
    }
    v.k = json_value::kind::number;
    if (literal("NaN")) {
      v.num = std::nan("");
      return v;
    }
    if (literal("Infinity")) {
      v.num = HUGE_VAL;
      return v;
    }
    if (literal("-Infinity")) {
      v.num = -HUGE_VAL;
      return v;
    }
    v.num = number(v.integer);
    return v;
  }

  // The number at i_, read with strtod from "<sign><digits>e<exponent>": no radix character, so
  // the C locale's decimal point does not matter, and strtod rounds correctly as float() does.
  double number(bool &integer) {
    std::string sign, digits;
    long long exp_adjust = 0;
    if (peek() == '-') {
      sign = "-";
      ++i_;
    }
    if (peek() == '0') {
      digits = "0";
      ++i_;
    } else if (peek() >= '1' && peek() <= '9') {
      while (peek() >= '0' && peek() <= '9')
        digits += s_[i_++];
    } else {
      throw json_error();
    }
    integer = true;
    if (peek() == '.' && i_ + 1 < s_.size() && s_[i_ + 1] >= '0' && s_[i_ + 1] <= '9') {
      integer = false;
      ++i_;
      while (peek() >= '0' && peek() <= '9') {
        digits += s_[i_++];
        --exp_adjust;
      }
    }
    long long exponent = 0;
    if (peek() == 'e' || peek() == 'E') {
      std::size_t j = i_ + 1;
      bool negative = false;
      if (j < s_.size() && (s_[j] == '+' || s_[j] == '-'))
        negative = s_[j++] == '-';
      if (j < s_.size() && s_[j] >= '0' && s_[j] <= '9') {
        integer = false;
        i_ = j;
        while (peek() >= '0' && peek() <= '9') {
          if (exponent < 100000000)
            exponent = exponent * 10 + (s_[i_] - '0');
          ++i_;
        }
        if (negative)
          exponent = -exponent;
      }
      // else: Python's regex stops before the 'e', and the parse fails on the next token.
    }
    const std::string text = sign + digits + "e" + std::to_string(exponent + exp_adjust);
    const double v = std::strtod(text.c_str(), nullptr);
    // An integer is a Python int, and int("-0") is 0: float() of it is +0.0, not -0.0.
    return integer && v == 0.0 ? 0.0 : v;
  }

  static int hex4(const std::string &s, std::size_t at) {
    if (at + 4 > s.size())
      throw json_error();
    int v = 0;
    for (std::size_t k = at; k < at + 4; ++k) {
      const char c = s[k];
      v <<= 4;
      if (c >= '0' && c <= '9')
        v |= c - '0';
      else if (c >= 'a' && c <= 'f')
        v |= c - 'a' + 10;
      else if (c >= 'A' && c <= 'F')
        v |= c - 'A' + 10;
      else
        throw json_error();
    }
    return v;
  }

  static void put_utf8(std::string &out, unsigned cp) {
    if (cp < 0x80) {
      out += static_cast<char>(cp);
    } else if (cp < 0x800) {
      out += static_cast<char>(0xC0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) { // a lone surrogate too: Python keeps it in the str
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    }
  }

  std::string string() {
    ++i_; // the opening quote
    std::string out;
    for (;;) {
      if (at_end())
        throw json_error();
      const unsigned char c = static_cast<unsigned char>(s_[i_++]);
      if (c == '"')
        return out;
      if (c < 0x20)
        throw json_error(); // "Invalid control character"
      if (c != '\\') {
        out += static_cast<char>(c);
        continue;
      }
      if (at_end())
        throw json_error();
      const char e = s_[i_++];
      switch (e) {
      case '"':
        out += '"';
        break;
      case '\\':
        out += '\\';
        break;
      case '/':
        out += '/';
        break;
      case 'b':
        out += '\b';
        break;
      case 'f':
        out += '\f';
        break;
      case 'n':
        out += '\n';
        break;
      case 'r':
        out += '\r';
        break;
      case 't':
        out += '\t';
        break;
      case 'u': {
        unsigned cp = static_cast<unsigned>(hex4(s_, i_));
        i_ += 4;
        if (cp >= 0xD800 && cp <= 0xDBFF && i_ + 1 < s_.size() && s_[i_] == '\\' &&
            s_[i_ + 1] == 'u') {
          const unsigned lo = static_cast<unsigned>(hex4(s_, i_ + 2));
          if (lo >= 0xDC00 && lo <= 0xDFFF) {
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
            i_ += 6;
          }
        }
        put_utf8(out, cp);
        break;
      }
      default:
        throw json_error(); // "Invalid \\escape"
      }
    }
  }
};

constexpr const char *kContractFields[] = {"d_hat_sdf_m",     "feature_frame",
                                           "feature_scale_m", "geometry_d_hat_m",
                                           "k_sharp",         "material_gate_trigger"};

bool is_contract_field(const std::string &k) {
  for (const char *f : kContractFields)
    if (k == f)
      return true;
  return false;
}

// Python's repr of a str, for messages: single quotes unless the text holds one.
std::string py_str_repr(const std::string &s) {
  const char q = s.find('\'') != std::string::npos && s.find('"') == std::string::npos ? '"' : '\'';
  std::string out(1, q);
  for (const char c : s) {
    if (c == q || c == '\\')
      out += '\\';
    out += c;
  }
  return out + q;
}

std::string json_repr(const json_value &v) {
  switch (v.k) {
  case json_value::kind::null:
    return "None";
  case json_value::kind::boolean:
    return v.b ? "True" : "False";
  case json_value::kind::number:
    return python_float_repr(v.num);
  case json_value::kind::string:
    return py_str_repr(v.str);
  case json_value::kind::array:
    return "[...]";
  case json_value::kind::object:
    return "{...}";
  }
  return "?";
}

// abs 1e-6, as math.isclose(a, b, rel_tol=0, abs_tol=1e-6): equal values (infinities included)
// are close; NaN is close to nothing.
bool close6(double a, double b) { return a == b || std::fabs(a - b) <= 1e-6; }

std::string join_mismatches(const std::vector<belief_contract_mismatch> &m, const char *word) {
  std::string s;
  for (std::size_t k = 0; k < m.size(); ++k) {
    if (k)
      s += ", ";
    s += m[k].field + "=" + m[k].actual + " (" + word + " " + m[k].expected + ")";
  }
  return s;
}

const char *py_bool(bool b) { return b ? "True" : "False"; }

} // namespace

// ---------------------------------------------------------------------------
// The contract
// ---------------------------------------------------------------------------

void belief_material_contract::validate() const {
  if (feature_frame != kBeliefMaterialRobotRelative)
    throw std::invalid_argument(std::string("feature_frame must be 'robot_relative', got ") +
                                py_str_repr(feature_frame));
  const std::pair<const char *, double> positive[] = {{"feature_scale_m", feature_scale_m},
                                                      {"k_sharp", k_sharp},
                                                      {"d_hat_sdf_m", d_hat_sdf_m},
                                                      {"geometry_d_hat_m", geometry_d_hat_m}};
  for (const auto &f : positive)
    if (!std::isfinite(f.second) || f.second <= 0.0)
      throw std::invalid_argument(std::string(f.first) + " must be finite and > 0");
  if (!std::isfinite(material_gate_trigger) || material_gate_trigger < 0.0 ||
      material_gate_trigger > 1.0)
    throw std::invalid_argument("material_gate_trigger must be finite and in [0, 1]");
}

belief_material_contract belief_material_contract::from_meta(const std::string &meta,
                                                             const std::string &source) {
  // json.loads(meta) if meta else {}; a decode error also reads as {}.
  json_value root;
  if (!meta.empty()) {
    try {
      root = json_reader(meta).document();
    } catch (const json_error &) {
      root = json_value();
    }
  }
  const json_value *record =
      root.k == json_value::kind::object ? root.find(kBeliefMaterialMetaKey) : nullptr;
  if (!record || record->k != json_value::kind::object)
    throw std::invalid_argument(source + ": no '" + kBeliefMaterialMetaKey +
                                "' record in the .cvcnm meta, so its geometry feature frame and "
                                "material constants are unknown");

  std::vector<std::string> unknown;
  for (const auto &key : record->obj_keys)
    if (!is_contract_field(key) && std::find(unknown.begin(), unknown.end(), key) == unknown.end())
      unknown.push_back(key);
  if (!unknown.empty()) {
    std::sort(unknown.begin(), unknown.end());
    std::string list;
    for (std::size_t k = 0; k < unknown.size(); ++k)
      list += (k ? ", " : "") + py_str_repr(unknown[k]);
    throw std::invalid_argument(source + ": unknown belief-material contract fields [" + list +
                                "]");
  }

  // The dataclass's __post_init__ order: the frame, the value types, then the value rules.
  belief_material_contract c;
  try {
    if (const json_value *f = record->find("feature_frame")) {
      if (f->k != json_value::kind::string || f->str != kBeliefMaterialRobotRelative)
        throw std::invalid_argument("feature_frame must be 'robot_relative', got " + json_repr(*f));
      c.feature_frame = f->str;
    }
    const std::pair<const char *, double *> numbers[] = {
        {"feature_scale_m", &c.feature_scale_m},
        {"k_sharp", &c.k_sharp},
        {"d_hat_sdf_m", &c.d_hat_sdf_m},
        {"material_gate_trigger", &c.material_gate_trigger},
        {"geometry_d_hat_m", &c.geometry_d_hat_m}};
    for (const auto &n : numbers)
      if (const json_value *f = record->find(n.first)) {
        if (f->k != json_value::kind::number)
          throw std::invalid_argument(std::string(n.first) + " must be a number, got " +
                                      json_repr(*f));
        *n.second = f->num;
      }
    c.validate();
  } catch (const std::invalid_argument &e) {
    throw std::invalid_argument(source + ": " + e.what());
  }
  return c;
}

std::string belief_material_contract::to_meta() const {
  validate();
  // json.dumps({key: asdict(self)}, sort_keys=True): the frame is "robot_relative" (validated), so
  // it needs no escaping.
  return std::string("{\"") + kBeliefMaterialMetaKey +
         "\": {\"d_hat_sdf_m\": " + python_float_repr(d_hat_sdf_m) + ", \"feature_frame\": \"" +
         feature_frame + "\", \"feature_scale_m\": " + python_float_repr(feature_scale_m) +
         ", \"geometry_d_hat_m\": " + python_float_repr(geometry_d_hat_m) +
         ", \"k_sharp\": " + python_float_repr(k_sharp) +
         ", \"material_gate_trigger\": " + python_float_repr(material_gate_trigger) + "}}";
}

bool operator==(const belief_material_contract &a, const belief_material_contract &b) {
  return a.feature_frame == b.feature_frame && a.feature_scale_m == b.feature_scale_m &&
         a.k_sharp == b.k_sharp && a.d_hat_sdf_m == b.d_hat_sdf_m &&
         a.material_gate_trigger == b.material_gate_trigger &&
         a.geometry_d_hat_m == b.geometry_d_hat_m;
}

// ---------------------------------------------------------------------------
// The checks
// ---------------------------------------------------------------------------

std::vector<belief_contract_mismatch> belief_header_mismatches(const coef_energy_net &net,
                                                               const belief_material_params &bp,
                                                               int patch_size,
                                                               double patch_extent_m) {
  const cvcnm_belief_header &h = net.belief_header();
  std::vector<belief_contract_mismatch> m;
  const auto check_int = [&](const char *name, long long actual, long long expected) {
    if (actual != expected)
      m.push_back({name, std::to_string(actual), std::to_string(expected)});
  };
  const auto check_bool = [&](const char *name, bool actual, bool expected) {
    if (actual != expected)
      m.push_back({name, py_bool(actual), py_bool(expected)});
  };
  const auto check_float = [&](const char *name, double actual, double expected) {
    if (!close6(actual, expected))
      m.push_back({name, python_float_repr(actual), python_float_repr(expected)});
  };
  check_int("risk_channels", h.risk_channels, 3);
  check_int("seen_rule", h.seen_rule, bp.rule == seen_rule::raycast ? 0 : 1);
  check_float("unknown_risk", h.unknown_risk, bp.unknown_risk);
  check_bool("unknown_hard", h.unknown_hard != 0, bp.unknown_hard != 0);
  check_bool("unknown_gate_hard", h.unknown_gate_hard != 0, bp.unknown_gate_hard);
  check_float("material_range_m", h.material_range_m, bp.material_range_m);
  check_float("patch_extent_m", h.patch_extent_m, patch_extent_m);
  check_int("patch_size", net.patch_size(), patch_size);
  return m;
}

std::vector<belief_contract_mismatch> belief_runtime_mismatches(const belief_material_contract &c,
                                                                const material_config &mc) {
  std::vector<belief_contract_mismatch> m;
  const std::pair<const char *, std::pair<double, double>> fields[] = {
      {"k_sharp", {mc.k_sharp, c.k_sharp}},
      {"d_hat_sdf_m", {mc.d_hat_m, c.d_hat_sdf_m}},
      {"material_trigger", {mc.gate.material_trigger, c.material_gate_trigger}}};
  for (const auto &f : fields)
    if (!close6(f.second.first, f.second.second))
      m.push_back({f.first, python_float_repr(f.second.first), python_float_repr(f.second.second)});
  return m;
}

material_config material_config_for(const belief_material_contract &c, float lam_soft,
                                    float lam_hard, double cell_w) {
  if (!std::isfinite(cell_w) || cell_w <= 0.0)
    throw std::invalid_argument("cvc::nav::material_config_for: cell_w must be finite and > 0");
  c.validate();
  // Python's GateParams.horizon_m, which MaterialRuntime turns into max(1, round(25 / cell_w))
  // cells. round() is half-even, as nearbyint is in the default rounding mode.
  constexpr double kGateHorizonM = 25.0;
  const double cells = std::nearbyint(kGateHorizonM / cell_w);
  if (cells > static_cast<double>(INT_MAX))
    throw std::invalid_argument("cvc::nav::material_config_for: cell_w is too small for the "
                                "gate horizon in cells");
  material_config mc;
  mc.lam_soft = lam_soft;
  mc.lam_hard = lam_hard;
  mc.k_sharp = static_cast<float>(c.k_sharp);
  mc.d_hat_m = static_cast<float>(c.d_hat_sdf_m);
  mc.sigma = 1.0;
  mc.gate.material_trigger = c.material_gate_trigger;
  mc.gate.horizon_cells = std::max(1, static_cast<int>(cells));
  mc.gate.hard_margin_m = 0.0; // sim_world: <= 0 means 2 cells, Python's None
  return mc;
}

// ---------------------------------------------------------------------------
// The controller
// ---------------------------------------------------------------------------

belief_material_controller::belief_material_controller(coef_energy_net net,
                                                       const belief_material_params &bp,
                                                       int patch_size, double patch_extent_m,
                                                       const std::string &source)
    : net_(std::move(net)), belief_(bp), patch_size_(patch_size), patch_extent_m_(patch_extent_m) {
  // from_cvcnm's order: the 3-channel v2 requirement, the header check, the meta contract; then
  // the constructor's own checks.
  if (net_.risk_channels() != 3)
    throw std::invalid_argument(source +
                                ": belief controller requires a 3-channel .cvcnm version 2");
  const auto m = belief_header_mismatches(net_, bp, patch_size, patch_extent_m);
  if (!m.empty())
    throw std::invalid_argument(
        source + ": incompatible belief checkpoint: " + join_mismatches(m, "expected"));
  contract_ = belief_material_contract::from_meta(net_.meta(), source);
  if (patch_size < 2)
    throw std::invalid_argument(source + ": patch_size must be >= 2");
  if (!std::isfinite(patch_extent_m) || patch_extent_m <= 0.0)
    throw std::invalid_argument(source + ": patch_extent_m must be finite and > 0");
  if (bp.rule != seen_rule::raycast)
    throw std::invalid_argument(source + ": belief controller requires belief seen_rule raycast");
  validate(bp);
}

belief_material_controller belief_material_controller::from_cvcnm(const std::string &path,
                                                                  const belief_material_params &bp,
                                                                  int patch_size,
                                                                  double patch_extent_m) {
  return belief_material_controller(coef_energy_net::load(path), bp, patch_size, patch_extent_m,
                                    path);
}

void belief_material_controller::check_runtime(const material_config &mc) const {
  const auto m = belief_runtime_mismatches(contract_, mc);
  if (!m.empty())
    throw std::invalid_argument("material config does not match the checkpoint contract: " +
                                join_mismatches(m, "contract"));
}

// ---------------------------------------------------------------------------
// The inputs
// ---------------------------------------------------------------------------

void build_belief_material_controller_inputs(double px, double py, double gx, double gy,
                                             const float *C, const float *R, const float *W, int N,
                                             const belief_material_view &planes,
                                             double patch_extent_m, int P, double feature_scale_m,
                                             const belief_material_params &bp,
                                             belief_material_controller_inputs &out) {
  static const char *who = "cvc::nav::build_belief_material_controller_inputs: ";
  if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(gx) || !std::isfinite(gy))
    throw std::invalid_argument(std::string(who) + "position and goal must be finite");
  if (!std::isfinite(feature_scale_m) || feature_scale_m <= 0.0)
    throw std::invalid_argument(std::string(who) + "feature_scale_m must be finite and > 0");
  if (N < 0 || (N > 0 && (!C || !R || !W)))
    throw std::invalid_argument(std::string(who) + "N must be >= 0, with C, R and W when N > 0");
  if (!planes.risk || !planes.hard_b || !planes.seen)
    throw std::invalid_argument(std::string(who) + "the belief planes must be set");

  // np.asarray(position_world, float32): a round to nearest from double.
  const float pfx = static_cast<float>(px), pfy = static_cast<float>(py);
  const float gfx = static_cast<float>(gx), gfy = static_cast<float>(gy);
  const float s = static_cast<float>(feature_scale_m);

  // goal_delta = (goal - position) / float32(scale), one float32 op at a time.
  const float dx = (gfx - pfx) / s;
  const float dy = (gfy - pfy) / s;
  const float dxx = dx * dx;
  const float dyy = dy * dy;
  out.goal_feats[0] = dx;
  out.goal_feats[1] = dy;
  out.goal_feats[2] = std::sqrt(dxx + dyy); // np.linalg.norm: sqrt(dot(d, d)); 1 ulp (BLAS)
  out.goal_feats[3] = 1.0f;

  out.obs_feats.assign(static_cast<std::size_t>(N) * 6, 0.0f);
  out.obs_mask.assign(static_cast<std::size_t>(N), 1);
  for (int k = 0; k < N; ++k) {
    const float lx = (C[2 * k] - pfx) / s;
    const float ly = (C[2 * k + 1] - pfy) / s;
    float *o = out.obs_feats.data() + static_cast<std::size_t>(k) * 6;
    o[0] = lx;
    o[1] = ly;
    o[2] = R[k] / s;
    o[3] = W[k];
    o[4] = dx - lx;
    o[5] = dy - ly;
  }

  // The crop is centred on the float32 position (Python passes the float32 array).
  out.risk_patch.assign(static_cast<std::size_t>(3) * (P > 0 ? P : 0) * (P > 0 ? P : 0), 0.0f);
  material_patch_belief(planes.risk, planes.hard_b, planes.seen, planes.rows, planes.cols,
                        planes.min_x, planes.min_y, planes.max_x, planes.max_y,
                        static_cast<double>(pfx), static_cast<double>(pfy), patch_extent_m, P, bp,
                        out.risk_patch.data());
}

int visible_obstacles(const std::uint8_t *last_visible, const std::uint8_t *truth, int rows,
                      int cols, double min_x, double min_y, double max_x, double max_y, double px,
                      double py, int max_obstacles, float *C, float *R, float *W) {
  static const char *who = "cvc::nav::visible_obstacles: ";
  if (!last_visible || !truth)
    throw std::invalid_argument(std::string(who) + "last_visible and truth must be set");
  if (rows < 2 || cols < 2)
    throw std::invalid_argument(std::string(who) + "rows and cols must be >= 2");
  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
      !std::isfinite(max_y) || !(max_x > min_x) || !(max_y > min_y))
    throw std::invalid_argument(std::string(who) + "the bounds must be finite with max > min");
  if (!std::isfinite(px) || !std::isfinite(py))
    throw std::invalid_argument(std::string(who) + "the position must be finite");
  if (max_obstacles < 0 || (max_obstacles > 0 && (!C || !R || !W)))
    throw std::invalid_argument(std::string(who) +
                                "max_obstacles must be >= 0, with C, R and W when > 0");

  // np.argwhere(last_visible & truth): row-major. _cell_to_world in float64, stored as float32.
  std::vector<float> centers;
  std::vector<double> dist;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * cols + c;
      if (!last_visible[k] || !truth[k])
        continue;
      const double wx =
          min_x + static_cast<double>(c) / static_cast<double>(cols - 1) * (max_x - min_x);
      const double wy =
          min_y + static_cast<double>(r) / static_cast<double>(rows - 1) * (max_y - min_y);
      const float fx = static_cast<float>(wx), fy = static_cast<float>(wy);
      centers.push_back(fx);
      centers.push_back(fy);
      // norm(centers - position, axis=1): float64 (the float32 centre promoted), x*x + y*y.
      const double ddx = static_cast<double>(fx) - px;
      const double ddy = static_cast<double>(fy) - py;
      const double xx = ddx * ddx;
      const double yy = ddy * ddy;
      dist.push_back(std::sqrt(xx + yy));
    }

  // argsort(kind="stable")[:max_obstacles]: equal distances keep the row-major order (E5).
  std::vector<std::size_t> order(dist.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) { return dist[a] < dist[b]; });
  const int k = static_cast<int>(std::min<std::size_t>(order.size(), max_obstacles));

  const double cell_w = (max_x - min_x) / (cols - 1);
  const double cell_h = (max_y - min_y) / (rows - 1);
  const float radius = static_cast<float>(0.5 * std::min(cell_w, cell_h));
  for (int i = 0; i < k; ++i) {
    C[2 * i] = centers[2 * order[i]];
    C[2 * i + 1] = centers[2 * order[i] + 1];
    R[i] = radius;
    W[i] = 1.0f;
  }
  return k;
}

} // namespace nav
} // namespace cvc
