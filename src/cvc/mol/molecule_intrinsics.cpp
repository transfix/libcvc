/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// cvc::mol in the DSL. See inc/cvc/mol/molecule_intrinsics.h.
#include <cmath>
#include <cvc/ariadne/ariadne.h>
#include <cvc/geometry/geometry.h>
#include <cvc/mol/blur.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/molecule_intrinsics.h>
#include <cvc/mol/torsions.h>
#include <cvc/mol/view.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace se = state_exec;

namespace {

se::value_t number_or_nil(double v) { return std::isnan(v) ? se::value_t() : se::value_t(v); }

se::value_t text(char c) { return se::value_t(c == ' ' ? std::string() : std::string(1, c)); }

se::value_t info(const mol::molecule &m) {
  std::size_t residues = 0, hetero = 0;
  const mol::atom *last = nullptr;
  for (const mol::atom &a : m.atoms) {
    if (!last || last->model != a.model || last->chain != a.chain ||
        last->residue_seq != a.residue_seq || last->insertion != a.insertion)
      ++residues;
    last = &a;
    hetero += a.hetero ? 1 : 0;
  }
  std::int64_t helices = 0, sheets = 0, turns = 0;
  for (const mol::secondary_structure &s : m.structures) {
    helices += s.type == mol::secondary_structure::kind::helix ? 1 : 0;
    sheets += s.type == mol::secondary_structure::kind::sheet ? 1 : 0;
    turns += s.type == mol::secondary_structure::kind::turn ? 1 : 0;
  }
  std::vector<se::value_t> bounds;
  for (double b : m.bounds())
    bounds.emplace_back(b);
  return se::make_dict({{"id", se::value_t(m.id)},
                        {"title", se::value_t(m.title)},
                        {"atoms", se::value_t(static_cast<std::int64_t>(m.atoms.size()))},
                        {"models", se::value_t(static_cast<std::int64_t>(m.models))},
                        {"chains", se::value_t(mol::chain_ids(m))},
                        {"residues", se::value_t(static_cast<std::int64_t>(residues))},
                        {"helices", se::value_t(helices)},
                        {"sheets", se::value_t(sheets)},
                        {"turns", se::value_t(turns)},
                        {"hetero", se::value_t(static_cast<std::int64_t>(hetero))},
                        {"bounds", se::make_list(std::move(bounds))}});
}

se::value_t torsions_value(std::vector<mol::residue_torsions> &&rs) {
  std::vector<se::value_t> out;
  out.reserve(rs.size());
  for (const mol::residue_torsions &r : rs) {
    std::vector<std::pair<std::string, se::value_t>> d = {
        {"chain", text(r.chain)},
        {"seq", se::value_t(static_cast<std::int64_t>(r.residue_seq))},
        {"insertion", text(r.insertion)},
        {"residue", se::value_t(r.residue)}};
    switch (r.type) {
    case mol::residue_torsions::kind::amino_acid:
      d.push_back({"kind", se::value_t(std::string("amino_acid"))});
      d.push_back({"phi", number_or_nil(r.phi)});
      d.push_back({"psi", number_or_nil(r.psi)});
      d.push_back({"omega", number_or_nil(r.omega)});
      break;
    case mol::residue_torsions::kind::nucleotide: {
      d.push_back({"kind", se::value_t(std::string("nucleotide"))});
      d.push_back({"alpha", number_or_nil(r.alpha)});
      d.push_back({"beta", number_or_nil(r.beta)});
      d.push_back({"gamma", number_or_nil(r.gamma)});
      d.push_back({"delta", number_or_nil(r.delta)});
      d.push_back({"epsilon", number_or_nil(r.epsilon)});
      d.push_back({"zeta", number_or_nil(r.zeta)});
      std::vector<se::value_t> nu;
      for (double v : r.nu)
        nu.push_back(number_or_nil(v));
      d.push_back({"nu", se::make_list(std::move(nu))});
      d.push_back({"chi", number_or_nil(r.chi)});
      d.push_back({"eta", number_or_nil(r.eta)});
      d.push_back({"theta", number_or_nil(r.theta)});
      break;
    }
    default:
      d.push_back({"kind", se::value_t(std::string("other"))});
      break;
    }
    out.push_back(se::make_dict(std::move(d)));
  }
  return se::make_list(std::move(out));
}

se::value_t object(boost::any payload, const char *type) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload = std::move(payload);
  obj->type_name = type;
  return se::value_t(std::move(obj));
}

std::string subject_of(const job_call &c, const char *what) {
  const std::string *path = std::get_if<std::string>(&c.input(0).v);
  return path ? *path : std::string(what);
}

} // namespace

job_op molecule_torsions_op(sliced_threading threading) {
  job_op op = make_job_op<no_params, std::vector<mol::residue_torsions>>(
      "mol.torsions", 1,
      [](const job_call &c, const no_params &) {
        return mol::torsions_job(c.input_as<mol::molecule>(0));
      },
      [](std::vector<mol::residue_torsions> &&r) { return torsions_value(std::move(r)); },
      [](const job_call &c, const no_params &) { return subject_of(c, "a molecule"); });
  op.threading = threading;
  return op;
}

job_op molecule_blur_op(sliced_threading threading) {
  job_op op = make_job_op<mol::blur_params, volume>(
      "mol.blur", 1,
      [](const job_call &c, const mol::blur_params &p) {
        return mol::blur_job(c.app(), *c.input_as<mol::molecule>(0), p);
      },
      [](volume &&v) {
        return object(std::shared_ptr<const volume>(std::make_shared<volume>(std::move(v))),
                      "cvc::volume");
      },
      [](const job_call &c, const mol::blur_params &) { return subject_of(c, "a molecule"); });
  op.threading = threading;
  return op;
}

job_op molecule_view_op(sliced_threading threading) {
  job_op op = make_job_op<mol::view_params, mol::molecule_view>(
      "mol.view", 1,
      [](const job_call &c, const mol::view_params &p) {
        return mol::build_view_job(c.app(), c.input_as<mol::molecule>(0), p);
      },
      [](mol::molecule_view &&v) {
        return se::make_dict({{"spheres", object(std::move(v.spheres), "cvc::geometry")},
                              {"sticks", object(std::move(v.sticks), "cvc::geometry")},
                              {"cartoon", object(std::move(v.cartoon), "cvc::geometry")},
                              {"atoms", se::value_t(static_cast<std::int64_t>(v.atoms_drawn))},
                              {"bonds", se::value_t(static_cast<std::int64_t>(v.bonds.size()))},
                              {"residues", se::value_t(static_cast<std::int64_t>(v.residues))}});
      },
      [](const job_call &c, const mol::view_params &) { return subject_of(c, "a molecule"); });
  op.threading = threading;
  return op;
}

void register_molecule_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                  cvc::app &app) {
  register_job_op(env, ictx, app, molecule_torsions_op());
  register_job_op(env, ictx, app, molecule_blur_op());
  register_job_op(env, ictx, app, molecule_view_op());
  register_job_intrinsics(env, app);
  cvc::app *const a = &app;
  se::intrinsics_context *const c = &ictx;
  se::builtins::register_fn(env, "mol.info", [a, c](std::span<const se::value_t> args) {
    if (args.size() != 1)
      throw se::dsl_error("TypeError", "mol.info: takes a molecule", "native");
    const job_call call(*a, *c, "mol.info", std::vector<se::value_t>(args.begin(), args.end()),
                        se::value_t());
    return info(*call.input_as<mol::molecule>(0));
  });
}

void install_molecule_intrinsics(cvc::app &app) {
  cvc::app *const a = &app;
  ariadne::register_action_intrinsics(
      app, "cvc.molecule", [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_molecule_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
