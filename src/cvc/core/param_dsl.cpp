/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The app's registry of parameter tables and (params.defaults) / (params.check). See
// cvc/core/param_dsl.h. The registry lives in the app's own data map (as sliced_task.cpp's task
// registry does), so it is created and destroyed with the app and a later app at the same address
// never sees it.

#include <algorithm>
#include <cvc/core/app.h>
#include <cvc/core/param_dsl.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace {

namespace se = cvc::state_exec;

const char kRegistryKey[] = "__param_tables";

struct param_registry {
  std::mutex mu;
  std::map<std::string, std::shared_ptr<const param_table_entry>> tables;
};

std::shared_ptr<param_registry> registry_of(cvc::app &app, bool create) {
  const cvc::mutex_ptr guard = app.mutex(kRegistryKey);
  boost::mutex::scoped_lock lock(*guard);
  std::shared_ptr<param_registry> reg;
  try {
    reg = app.data<std::shared_ptr<param_registry>>(kRegistryKey);
  } catch (const std::exception &) {
    // not yet present
  }
  if (!reg && create) {
    reg = std::make_shared<param_registry>();
    app.data(kRegistryKey, reg);
  }
  return reg;
}

const std::string &string_arg(std::span<const se::value_t> args, std::size_t i, const char *fn) {
  if (i >= args.size())
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": missing argument " + std::to_string(i + 1), "native");
  const std::string *s = std::get_if<std::string>(&args[i].v);
  if (!s)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) +
                            " is a table's name (a string), not " + param_detail::dsl_kind(args[i]),
                        "native");
  return *s;
}

std::shared_ptr<const param_table_entry> table_or_throw(cvc::app &app, const std::string &name,
                                                        const char *fn) {
  std::shared_ptr<const param_table_entry> t = find_param_table(app, name);
  if (t)
    return t;
  std::string known;
  for (const std::string &n : param_table_names(app))
    known += (known.empty() ? "" : ", ") + n;
  throw std::invalid_argument(
      std::string(fn) + ": no parameter table named '" + name + "'" +
      (known.empty() ? std::string(" (none is registered)") : " (registered: " + known + ")"));
}

} // namespace

void register_param_table(cvc::app &app, param_table_entry entry) {
  std::shared_ptr<param_registry> reg = registry_of(app, true);
  auto e = std::make_shared<const param_table_entry>(std::move(entry));
  std::lock_guard<std::mutex> lk(reg->mu);
  reg->tables[e->name] = std::move(e);
}

std::shared_ptr<const param_table_entry> find_param_table(cvc::app &app, const std::string &name) {
  std::shared_ptr<param_registry> reg = registry_of(app, false);
  if (!reg)
    return nullptr;
  std::lock_guard<std::mutex> lk(reg->mu);
  auto it = reg->tables.find(name);
  return it == reg->tables.end() ? nullptr : it->second;
}

std::vector<std::string> param_table_names(cvc::app &app) {
  std::vector<std::string> names;
  std::shared_ptr<param_registry> reg = registry_of(app, false);
  if (!reg)
    return names;
  std::lock_guard<std::mutex> lk(reg->mu);
  for (const auto &kv : reg->tables)
    names.push_back(kv.first);
  return names; // std::map: sorted
}

void register_param_intrinsics(se::environment_ptr env, cvc::app &app) {
  cvc::app *const a = &app;
  se::builtins::register_fn(
      env, "params.defaults", [a](std::span<const se::value_t> args) -> se::value_t {
        if (args.size() != 1)
          throw se::dsl_error("TypeError", "params.defaults: takes a table's name", "native");
        const std::string &name = string_arg(args, 0, "params.defaults");
        return table_or_throw(*a, name, "params.defaults")->defaults();
      });
  se::builtins::register_fn(
      env, "params.check", [a](std::span<const se::value_t> args) -> se::value_t {
        if (args.size() != 2)
          throw se::dsl_error(
              "TypeError", "params.check: takes a table's name and a dict of parameters", "native");
        const std::string &name = string_arg(args, 0, "params.check");
        const std::vector<std::string> problems =
            table_or_throw(*a, name, "params.check")->check(args[1]);
        std::vector<se::value_t> out;
        out.reserve(problems.size());
        for (const std::string &p : problems)
          out.emplace_back(p);
        return se::make_list(std::move(out));
      });
}

} // namespace cvc
