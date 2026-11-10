/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A params struct's CLI flags, from its cvc::param_table (cvc/core/param_table.h), for a `cvc`
// subcommand built on boost::program_options:
//
//   po::options_description desc("cvc blur - ...");
//   desc.add_options()("help,h", "show help")("input,i", ...);
//   cvc::add_param_options<blur_params>(desc);            // --sigma, --passes, --edges
//   desc.add_options()("radius", po::value<std::string>(), "legacy alias of --sigma"); // by hand
//   po::store(po::parse_command_line(argc, argv, desc), vm);
//   cvc::param_result<blur_params> r = cvc::params_from_options<blur_params>(vm);
//   if (!r.ok()) throw std::runtime_error("cvc blur: " + r.message());
//
// Each field is one flag, named param_flag_name(field) with an optional prefix ("--seg-t-low"
// for prefix "seg-"), taking its value as text (param_table.h's forms: 2.5, true, clamp, [1, 2]).
// A bool flag alone means true (--quantize-u8), and takes =false. Its help is the field's doc,
// then its unit, range, choices and default in brackets. Legacy aliases are added by hand.

#ifndef CVC_CORE_PARAM_CLI_H
#define CVC_CORE_PARAM_CLI_H

#include <boost/program_options.hpp>
#include <cmath>
#include <cvc/core/param_table.h>
#include <string>
#include <type_traits>

namespace cvc {

// The help text of one field: "<doc> [unit voxel; 0 .. 64; one of a, b; default 1]".
inline std::string param_help(const param_field_info &i) {
  std::string extra;
  const auto add = [&](const std::string &s) { extra += (extra.empty() ? "" : "; ") + s; };
  if (!i.unit.empty())
    add("unit " + i.unit);
  if (!std::isnan(i.min) || !std::isnan(i.max))
    add((std::isnan(i.min) ? std::string() : param_detail::format_double(i.min)) + " .. " +
        (std::isnan(i.max) ? std::string() : param_detail::format_double(i.max)));
  if (!i.choices.empty()) {
    std::string c = i.kind == "list" ? "each one of " : "one of ";
    for (std::size_t k = 0; k < i.choices.size(); ++k)
      c += (k ? ", " : "") + i.choices[k];
    add(c);
  }
  if (i.kind == "list")
    add("a list: [a, b] or a,b");
  std::string def = i.default_text.empty() ? std::string("\"\"") : i.default_text;
  if (i.kind == "bool")
    def = def == "1" ? "true" : "false";
  add("default " + def);
  return (i.doc.empty() ? std::string() : i.doc + " ") + "[" + extra + "]";
}

// Add one flag per field of P's table to `desc`.
template <class P>
void add_param_options(boost::program_options::options_description &desc,
                       const std::string &prefix = std::string()) {
  namespace po = boost::program_options;
  const std::vector<param_field_info> infos = param_field_infos<P>();
  for (const param_field_info &i : infos) {
    const std::string flag = prefix + param_flag_name(i.name);
    po::typed_value<std::string> *v = po::value<std::string>();
    if (i.kind == "bool")
      v->implicit_value("true");
    // program_options keeps the help text by value; a description is built once per command.
    desc.add_options()(flag.c_str(), v, param_help(i).c_str());
  }
}

// The struct from the flags given in `vm` over `base` (flags not given keep base's values), then
// validated: `errors` names each flag that did not parse and each value out of range.
template <class P>
param_result<P> params_from_options(const boost::program_options::variables_map &vm,
                                    const std::string &prefix = std::string(), P base = {}) {
  param_result<P> r;
  r.value = std::move(base);
  for_each_param<P>([&](const auto &f) {
    const std::string flag = prefix + param_flag_name(f.name);
    auto it = vm.find(flag);
    if (it == vm.end() || it->second.empty() || it->second.defaulted())
      return;
    std::string text;
    try {
      text = it->second.template as<std::string>();
    } catch (const std::exception &) {
      r.errors.push_back("--" + flag + ": not a string");
      return;
    }
    std::string why;
    if (!param_detail::value_from_text(text, r.value.*(f.member), why))
      r.errors.push_back("--" + flag + ": " + why);
  });
  for (std::string &p : params_validate(r.value))
    r.errors.push_back(std::move(p));
  return r;
}

} // namespace cvc

#endif // CVC_CORE_PARAM_CLI_H
