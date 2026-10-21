// Document arguments — see inc/cvc/ariadne/doc_args.h.

#include <algorithm>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/doc_args.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h> // value_t, make_list
#include <memory>
#include <sstream>

namespace cvc {
namespace ariadne {

const char *arg_type_name(ArgSpec::Type type) {
  switch (type) {
  case ArgSpec::Type::Bool:
    return "bool";
  case ArgSpec::Type::Int:
    return "int";
  case ArgSpec::Type::Float:
    return "float";
  case ArgSpec::Type::String:
    break;
  }
  return "string";
}

bool canonical_arg_value(ArgSpec::Type type, const std::string &in, std::string &out) {
  switch (type) {
  case ArgSpec::Type::String:
    out = in;
    return true;
  case ArgSpec::Type::Bool:
    if (in == "true" || in == "1" || in == "yes" || in == "on") {
      out = "true";
      return true;
    }
    if (in == "false" || in == "0" || in == "no" || in == "off") {
      out = "false";
      return true;
    }
    return false;
  case ArgSpec::Type::Int: {
    std::int64_t v = 0;
    const char *b = in.data(), *e = in.data() + in.size();
    const auto [end, ec] = std::from_chars(b, e, v);
    if (in.empty() || ec != std::errc() || end != e)
      return false;
    out = std::to_string(v);
    return true;
  }
  case ArgSpec::Type::Float: {
    if (in.empty())
      return false;
    char *end = nullptr;
    (void)std::strtod(in.c_str(), &end);
    if (end != in.c_str() + in.size())
      return false;
    out = in;
    return true;
  }
  }
  return false;
}

bool valid_arg_name(const std::string &name) {
  if (name.empty() || name == "help")
    return false;
  const auto alpha = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); };
  if (!alpha(name[0]))
    return false;
  for (char c : name)
    if (!alpha(c) && !(c >= '0' && c <= '9') && c != '-' && c != '_')
      return false;
  return true;
}

namespace {

// "a string", "an int", ... for messages.
std::string a_type(ArgSpec::Type type) {
  return std::string(type == ArgSpec::Type::Int ? "an " : "a ") + arg_type_name(type);
}

const ArgSpec *find_option(const std::vector<ArgSpec> &spec, const std::string &name) {
  for (const ArgSpec &a : spec)
    if (!a.positional && a.name == name)
      return &a;
  return nullptr;
}

void set_value(DocumentArgs &out, const std::string &name, std::string v) {
  for (auto &kv : out.values)
    if (kv.first == name) {
      kv.second = std::move(v); // a repeated option: the last one wins
      return;
    }
  out.values.emplace_back(name, std::move(v));
}

bool has_value(const DocumentArgs &out, const std::string &name) {
  for (const auto &kv : out.values)
    if (kv.first == name)
      return true;
  return false;
}

// A token that reads as a negative number is a value, not an option ("--offset -3", or "-3" as a
// positional).
bool looks_numeric(const std::string &t) {
  std::string unused;
  return canonical_arg_value(ArgSpec::Type::Float, t, unused);
}

} // namespace

DocumentArgs parse_document_args(const std::vector<ArgSpec> &spec,
                                 const std::vector<std::string> &argv) {
  DocumentArgs out;
  std::vector<const ArgSpec *> positionals;
  for (const ArgSpec &a : spec)
    if (a.positional)
      positionals.push_back(&a);
  std::size_t next_positional = 0;
  bool options_done = false;

  const auto take_positional = [&](const std::string &tok) {
    if (next_positional >= positionals.size()) {
      out.errors.push_back("unexpected argument '" + tok + "'");
      return;
    }
    const ArgSpec &a = *positionals[next_positional++];
    std::string v;
    if (!canonical_arg_value(a.type, tok, v))
      out.errors.push_back("<" + a.name + ">: '" + tok + "' is not " + a_type(a.type));
    else
      set_value(out, a.name, std::move(v));
  };

  for (std::size_t i = 0; i < argv.size(); ++i) {
    const std::string &tok = argv[i];
    if (options_done || tok.size() < 2 || tok[0] != '-' || looks_numeric(tok)) {
      take_positional(tok);
      continue;
    }
    if (tok == "--") {
      options_done = true;
      continue;
    }
    if (tok == "-h" || tok == "--help") {
      out.help = true;
      continue;
    }
    if (tok.rfind("--", 0) != 0) {
      out.errors.push_back("unknown option '" + tok + "'");
      continue;
    }
    std::string name = tok.substr(2), text;
    bool has_text = false;
    if (const std::size_t eq = name.find('='); eq != std::string::npos) {
      text = name.substr(eq + 1);
      name.resize(eq);
      has_text = true;
    }
    const ArgSpec *a = find_option(spec, name);
    if (!a && !has_text && name.rfind("no-", 0) == 0) { // --no-flag clears a bool
      const ArgSpec *neg = find_option(spec, name.substr(3));
      if (neg && neg->type == ArgSpec::Type::Bool) {
        set_value(out, neg->name, "false");
        continue;
      }
    }
    if (!a) {
      out.errors.push_back("unknown option '--" + name + "'");
      continue;
    }
    if (a->type == ArgSpec::Type::Bool && !has_text) {
      set_value(out, a->name, "true"); // a bare --flag never consumes the next token
      continue;
    }
    if (!has_text) {
      if (i + 1 >= argv.size()) {
        out.errors.push_back("--" + name + " needs " + a_type(a->type) + " value");
        continue;
      }
      text = argv[++i];
    }
    std::string v;
    if (!canonical_arg_value(a->type, text, v))
      out.errors.push_back("--" + name + ": '" + text + "' is not " + a_type(a->type));
    else
      set_value(out, a->name, std::move(v));
  }

  for (const ArgSpec &a : spec) {
    if (has_value(out, a.name))
      continue;
    if (a.has_default)
      out.values.emplace_back(a.name, a.default_value);
    else if (a.required)
      out.errors.push_back(a.positional ? "missing <" + a.name + ">"
                                        : "missing required --" + a.name);
  }
  return out;
}

std::string format_document_usage(const std::string &invocation, const std::string &description,
                                  const std::vector<ArgSpec> &spec) {
  std::ostringstream os;
  os << "usage: " << invocation;
  bool any_option = false;
  for (const ArgSpec &a : spec)
    any_option = any_option || !a.positional;
  if (any_option)
    os << " [options]";
  for (const ArgSpec &a : spec)
    if (a.positional)
      os << (a.required ? " <" + a.name + ">" : " [" + a.name + "]");
  os << "\n";
  if (!description.empty())
    os << "\n" << description << "\n";
  if (spec.empty())
    return os.str();
  os << "\n";
  std::vector<std::pair<std::string, std::string>> rows;
  std::size_t width = 0;
  for (const ArgSpec &a : spec) {
    std::string left = a.positional ? "<" + a.name + ">"
                                    : "--" + a.name +
                                          (a.type == ArgSpec::Type::Bool
                                               ? ""
                                               : " <" + std::string(arg_type_name(a.type)) + ">");
    std::string right = a.help;
    if (a.has_default)
      right += (right.empty() ? "" : " ") + std::string("(default: ") + a.default_value + ")";
    else if (a.required)
      right += (right.empty() ? "" : " ") + std::string("(required)");
    width = std::max(width, left.size());
    rows.emplace_back(std::move(left), std::move(right));
  }
  rows.emplace_back("-h, --help", "show this help");
  width = std::max(width, rows.back().first.size());
  for (const auto &r : rows)
    os << "  " << r.first << std::string(width - r.first.size() + 2, ' ') << r.second << "\n";
  return os.str();
}

namespace {

namespace se = cvc::state_exec;

se::value_t typed_value(ArgSpec::Type type, const std::string &canonical) {
  switch (type) {
  case ArgSpec::Type::Bool:
    return se::value_t(canonical == "true");
  case ArgSpec::Type::Int: {
    std::int64_t v = 0;
    std::from_chars(canonical.data(), canonical.data() + canonical.size(), v);
    return se::value_t(v);
  }
  case ArgSpec::Type::Float:
    return se::value_t(std::strtod(canonical.c_str(), nullptr));
  case ArgSpec::Type::String:
    break;
  }
  return se::value_t(std::string(canonical));
}

// Write one read-only node (value + data), clearing read-only first so a reload can rewrite it.
void write_read_only(cvc::state &node, const std::string &value, const se::value_t &data) {
  node.readOnly(false);
  node.value(value);
  node.data(boost::any(data));
  node.readOnly(true);
}

} // namespace

void publish_document_args(cvc::app &app, const std::string &root,
                           const std::vector<std::string> &argv, const std::vector<ArgSpec> *spec,
                           const DocumentArgs *parsed) {
  cvc::state &tree = cvc::state::instance(app);
  cvc::state &base = root.empty() ? tree : tree(root);

  std::vector<se::value_t> items;
  std::string joined;
  for (const std::string &a : argv) {
    items.emplace_back(std::string(a));
    joined += (joined.empty() ? "" : " ") + a;
  }
  write_read_only(base("argv"), joined, se::make_list(std::move(items)));

  // args holds exactly what this call parsed: drop what an earlier one wrote (a reload).
  if (cvc::state *old = base.findDescendant("args")) {
    old->readOnly(false);
    old->data(boost::any());
    for (const std::string &child : old->children()) {
      cvc::state &n = (*old)(child);
      n.readOnly(false);
      n.expireAt(boost::posix_time::microsec_clock::universal_time());
    }
    old->sweepExpired();
  }
  if (!spec || !parsed)
    return;
  // One node per argument for direct reads and binds, and the whole set as a dict on args itself:
  // a hole links a single node, so that dict is what one `link: {args: ...}` grant hands a module.
  std::vector<std::pair<std::string, se::value_t>> all;
  for (const auto &kv : parsed->values) {
    ArgSpec::Type type = ArgSpec::Type::String;
    for (const ArgSpec &a : *spec)
      if (a.name == kv.first)
        type = a.type;
    const se::value_t v = typed_value(type, kv.second);
    write_read_only(base("args")(kv.first), kv.second, v);
    all.emplace_back(kv.first, v);
  }
  write_read_only(base("args"), std::string(), se::make_dict(std::move(all)));
}

} // namespace ariadne
} // namespace cvc
